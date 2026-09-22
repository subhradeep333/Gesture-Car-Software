"""
Serial Manager for non-blocking communication with the Arduino Transmitter.
"""

import time
import glob
import sys
import threading
import queue
import serial
import serial.tools.list_ports
import config

class SerialManager:
    def __init__(self, baud_rate=config.DEFAULT_BAUD_RATE):
        self.baud_rate = baud_rate
        self.ser = None
        self.is_connected = False
        self.current_port = None
        
        self.tx_queue = queue.Queue(maxsize=10)
        self.rx_thread = None
        self.tx_thread = None
        self.is_running = False

        self.last_ack_time = 0.0
        self.telemetry_callback = None
        self.status_callback = None

    @staticmethod
    def list_ports():
        """Scans system for available serial ports."""
        ports = []
        if sys.platform.startswith('darwin'):
            # macOS USB serial patterns
            patterns = ['/dev/cu.usbserial*', '/dev/cu.usbmodem*', '/dev/cu.wchusbserial*']
            for pattern in patterns:
                ports.extend(glob.glob(pattern))
        elif sys.platform.startswith('linux'):
            ports.extend(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        elif sys.platform.startswith('win'):
            ports.extend([p.device for p in serial.tools.list_ports.comports()])

        # Fallback to PySerial port enumeration
        if not ports:
            for p in serial.tools.list_ports.comports():
                ports.append(p.device)

        return sorted(list(set(ports)))

    def register_callbacks(self, telemetry_cb=None, status_cb=None):
        """Registers callbacks for receiving serial logs and connection status updates."""
        self.telemetry_callback = telemetry_cb
        self.status_callback = status_cb

    def connect(self, port_name):
        """Connects to specified serial port."""
        if self.is_connected:
            self.disconnect()

        try:
            self.ser = serial.Serial(port_name, self.baud_rate, timeout=config.SERIAL_TIMEOUT)
            time.sleep(1.8)  # Wait for Arduino auto-reset on DTR toggle
            
            self.current_port = port_name
            self.is_connected = True
            self.is_running = True

            # Start worker threads
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self.rx_thread.start()
            self.tx_thread.start()

            if self.status_callback:
                self.status_callback(True, f"Connected to {port_name} @ {self.baud_rate} baud")
            return True, f"Connected to {port_name}"
        except Exception as e:
            self.is_connected = False
            self.current_port = None
            if self.status_callback:
                self.status_callback(False, f"Connection failed: {str(e)}")
            return False, f"Connection failed: {str(e)}"

    def send_command(self, cmd_char):
        """Queues a single-character command for transmission."""
        if not self.is_connected or not self.is_running:
            return False

        # Drain queue if full to prioritize latest command
        if self.tx_queue.full():
            try:
                self.tx_queue.get_nowait()
            except queue.Empty:
                pass

        try:
            self.tx_queue.put_nowait(cmd_char)
            return True
        except queue.Full:
            return False

    def _tx_loop(self):
        """TX Worker thread to send commands over Serial."""
        while self.is_running and self.is_connected:
            try:
                cmd = self.tx_queue.get(timeout=0.1)
                if self.ser and self.ser.is_open:
                    payload = f"{cmd}\n".encode('ascii')
                    self.ser.write(payload)
                    self.ser.flush()
                    if self.telemetry_callback:
                        self.telemetry_callback("TX", f"Sent command '{cmd}'")
                self.tx_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                if self.telemetry_callback:
                    self.telemetry_callback("ERROR", f"Serial TX error: {str(e)}")
                self.is_connected = False
                break

    def _rx_loop(self):
        """RX Worker thread to receive telemetry/ACKs from Arduino."""
        while self.is_running and self.is_connected:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        self.last_ack_time = time.time()
                        if self.telemetry_callback:
                            self.telemetry_callback("RX", line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.telemetry_callback:
                    self.telemetry_callback("ERROR", f"Serial RX error: {str(e)}")
                self.is_connected = False
                break

    def disconnect(self):
        """Disconnects serial port and cleans up worker threads."""
        self.is_running = False
        self.is_connected = False
        
        # Clear TX queue
        while not self.tx_queue.empty():
            try:
                self.tx_queue.get_nowait()
            except queue.Empty:
                break

        if self.ser:
            try:
                if self.ser.is_open:
                    # Send STOP command before closing
                    self.ser.write(b"S\n")
                    self.ser.flush()
                    time.sleep(0.1)
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

        self.current_port = None
        if self.status_callback:
            self.status_callback(False, "Serial disconnected")
