"""
Asynchronous WebSocket Manager for ESP32 Wi-Fi Communication with Qt Signal bindings.
"""

import time
import json
import asyncio
import threading
import queue
import websockets
from PySide6.QtCore import QObject, Signal
import config

class WebSocketManager(QObject):
    # PySide6 Qt Signals for thread-safe UI updates
    connection_status_changed = Signal(bool, str)  # (is_connected, status_text)
    telemetry_received = Signal(dict)               # Parsed JSON telemetry from ESP32
    log_emitted = Signal(str, str)                  # (tag, message)

    def __init__(self, ws_url=config.WS_URL):
        super().__init__()
        self.ws_url = ws_url
        self.is_connected = False
        self.sequence_num = 0
        
        self.tx_queue = queue.Queue(maxsize=10)
        self.is_running = False
        self.thread = None
        self.loop = None
        self.ws_client = None

    def start(self):
        """Starts background asyncio loop thread."""
        if self.is_running:
            return
        self.is_running = True
        self.thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.thread.start()

    def _run_async_loop(self):
        """Asyncio event loop running inside background thread."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._websocket_client_task())

    async def _websocket_client_task(self):
        """Async task handling WebSocket connection, re-connection, TX queue, and RX messages."""
        while self.is_running:
            try:
                self.log_emitted.emit("WS", f"Connecting to {self.ws_url}...")
                async with websockets.connect(self.ws_url, ping_interval=5, ping_timeout=3) as ws:
                    self.ws_client = ws
                    self.is_connected = True
                    self.connection_status_changed.emit(True, f"Connected to {self.ws_url}")
                    self.log_emitted.emit("WS", "WebSocket connection established!")

                    # Run TX and RX concurrently
                    tx_task = asyncio.create_task(self._tx_producer(ws))
                    rx_task = asyncio.create_task(self._rx_consumer(ws))

                    done, pending = await asyncio.wait(
                        [tx_task, rx_task],
                        return_when=asyncio.FIRST_COMPLETED
                    )
                    for task in pending:
                        task.cancel()

            except Exception as e:
                self.is_connected = False
                self.ws_client = None
                self.connection_status_changed.emit(False, f"Disconnected: {str(e)}")
                self.log_emitted.emit("WS_ERROR", f"Connection error: {str(e)}")

            if self.is_running:
                await asyncio.sleep(2.0)  # Reconnect delay

    async def _tx_producer(self, ws):
        """Async task sending JSON packets from queue to WebSocket."""
        while self.is_running and self.is_connected:
            try:
                # Poll queue non-blockingly via loop.run_in_executor
                packet = await self.loop.run_in_executor(None, self._pop_tx_queue)
                if packet:
                    payload = json.dumps(packet)
                    await ws.send(payload)
                    self.log_emitted.emit("TX", payload)
                else:
                    await asyncio.sleep(0.01)
            except Exception as e:
                self.log_emitted.emit("TX_ERROR", f"Send failed: {str(e)}")
                break

    def _pop_tx_queue(self):
        try:
            return self.tx_queue.get(timeout=0.05)
        except queue.Empty:
            return None

    async def _rx_consumer(self, ws):
        """Async task receiving telemetry JSON messages from ESP32."""
        async for message in ws:
            try:
                data = json.loads(message)
                self.telemetry_received.emit(data)
                self.log_emitted.emit("RX", message)
            except json.JSONDecodeError:
                self.log_emitted.emit("RX_RAW", message)
            except Exception as e:
                self.log_emitted.emit("RX_ERROR", f"Receive error: {str(e)}")

    def send_command(self, cmd_char, speed=config.DEFAULT_MOTOR_SPEED):
        """Queues a command packet to send to ESP32."""
        if not self.is_connected:
            return False

        self.sequence_num += 1
        packet = {
            "command": cmd_char,
            "speed": int(speed),
            "sequence": self.sequence_num,
            "timestamp": int(time.time() * 1000)
        }

        # Clear old items if queue is full
        if self.tx_queue.full():
            try:
                self.tx_queue.get_nowait()
            except queue.Empty:
                pass

        try:
            self.tx_queue.put_nowait(packet)
            return True
        except queue.Full:
            return False

    def stop(self):
        """Stops WebSocket manager and closes connection cleanly."""
        self.is_running = False
        self.is_connected = False
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
