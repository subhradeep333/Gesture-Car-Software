"""
Asynchronous WebSocket Manager for ESP32 Wi-Fi Communication with Qt Signal bindings.
Optimized with asyncio.Queue and zero-delay WebSocket dispatches.
"""

import time
import json
import asyncio
import threading
import websockets
from PySide6.QtCore import QObject, Signal
import config

class WebSocketManager(QObject):
    # PySide6 Qt Signals for thread-safe UI updates
    connection_status_changed = Signal(bool, str)  # (is_connected, status_text)
    telemetry_received = Signal(dict)               # Parsed JSON telemetry from ESP32
    radar_telemetry_received = Signal(dict)         # Parsed Ultrasonic Radar data from ESP32
    log_emitted = Signal(str, str)                  # (tag, message)

    def __init__(self, ws_url=config.WS_URL):
        super().__init__()
        self.ws_url = ws_url
        self.is_connected = False
        self.sequence_num = 0
        
        self.is_running = False
        self.thread = None
        self.loop = None
        self.async_queue = None

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
        self.async_queue = asyncio.Queue(maxsize=10)
        self.loop.run_until_complete(self._websocket_client_task())

    async def _websocket_client_task(self):
        """Async task handling WebSocket connection, re-connection, TX queue, and RX messages."""
        while self.is_running:
            try:
                self.log_emitted.emit("WS", f"Connecting to {self.ws_url}...")
                # Connect with compression disabled for ultra-low latency & CPU saving
                async with websockets.connect(
                    self.ws_url, 
                    ping_interval=1.5, 
                    ping_timeout=1.0,
                    compression=None
                ) as ws:
                    self.is_connected = True
                    self.connection_status_changed.emit(True, f"Connected to {self.ws_url}")
                    self.log_emitted.emit("WS", "WebSocket connection established!")

                    # Run TX and RX tasks concurrently
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
                self.connection_status_changed.emit(False, f"Disconnected: {str(e)}")
                self.log_emitted.emit("WS_ERROR", f"Connection error: {str(e)}")

            if self.is_running:
                await asyncio.sleep(0.5)  # Fast reconnect retry (0.5s) for instant link recovery

    async def _tx_producer(self, ws):
        """Async task pulling JSON packets from asyncio.Queue instantly with 0ms delay."""
        while self.is_running and self.is_connected:
            try:
                packet = await self.async_queue.get()
                payload = json.dumps(packet)
                await ws.send(payload)
                self.log_emitted.emit("TX", payload)
                self.async_queue.task_done()
            except Exception as e:
                self.log_emitted.emit("TX_ERROR", f"Send failed: {str(e)}")
                break

    async def _rx_consumer(self, ws):
        """Async task receiving telemetry JSON messages from ESP32."""
        async for message in ws:
            try:
                data = json.loads(message)
                self.telemetry_received.emit(data)
                if "radar" in data and isinstance(data["radar"], dict):
                    self.radar_telemetry_received.emit(data["radar"])
                self.log_emitted.emit("RX", message)
            except json.JSONDecodeError:
                self.log_emitted.emit("RX_RAW", message)
            except Exception as e:
                self.log_emitted.emit("RX_ERROR", f"Receive error: {str(e)}")

    def send_command(self, cmd_char, speed=config.DEFAULT_MOTOR_SPEED):
        """Queues a command packet to send to ESP32 instantly via thread-safe call."""
        if not self.is_connected or not self.loop or not self.async_queue:
            return False

        self.sequence_num += 1
        packet = {
            "command": cmd_char,
            "speed": int(speed),
            "sequence": self.sequence_num,
            "timestamp": int(time.time() * 1000)
        }

        # Dispatch item into asyncio.Queue safely from PySide6 GUI thread
        self.loop.call_soon_threadsafe(self._enqueue_packet_safe, packet)
        return True

    def _enqueue_packet_safe(self, packet):
        if self.async_queue.full():
            try:
                self.async_queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
        try:
            self.async_queue.put_nowait(packet)
        except asyncio.QueueFull:
            pass

    def stop(self):
        """Stops WebSocket manager and closes connection cleanly."""
        self.is_running = False
        self.is_connected = False
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
