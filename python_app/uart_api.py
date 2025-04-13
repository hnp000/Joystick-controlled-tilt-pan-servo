"""
uart_api.py

A UART communication API that handles connecting to a serial port,
sending framed data, and receiving data asynchronously using a background thread.
"""

import serial
import threading
import time
import queue


class UART_API:
    def __init__(self):
        """Initialize the UART API instance."""
        self.ser = None
        self.running = False
        self.read_thread = None
        self.data_queue = queue.Queue()  # Thread-safe queue for incoming data

    def connect(self, port, baud):
        """
        Connect to the specified serial port with the given baud rate.
        
        :param port: Serial port identifier (e.g., 'COM5' or '/dev/ttyUSB0')
        :param baud: Baud rate as an integer (e.g., 115200)
        :return: True if connection succeeds, False otherwise.
        """
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            print(f"Connected to {port} at {baud} baud")
            return True
        except Exception as e:
            print("Error connecting:", e)
            return False

    def disconnect(self):
        """Disconnect from the serial port and stop the read thread."""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            print("Disconnected")

    def send_frame(self, command, data, data_length=None):
        """
        Send a structured frame over UART.
        
        Frame format:
          - Header: 0xAA, 0x55, 0xAA
          - Command: 1 byte
          - Length: 1 byte (data length)
          - Data: Variable length
        
        :param command: Command byte as an integer.
        :param data: Data to send (int, list of ints, or bytes).
        :param data_length: Optional length override. If not provided, the full length is used.
        """
        if self.ser and self.ser.is_open:
            header = bytes([0xAA, 0x55, 0xAA])

            # Convert data to bytes.
            if isinstance(data, int):
                # Convert an integer to a single byte (or more if needed)
                data_bytes = data.to_bytes((data.bit_length() + 7) // 8 or 1, 'little')
            elif isinstance(data, list):
                data_bytes = bytes(data)
            elif isinstance(data, bytes):
                data_bytes = data
            else:
                raise ValueError("Unsupported data type for UART transmission.")

            # Determine the length.
            if data_length is None:
                data_length = len(data_bytes)
            else:
                data_bytes = data_bytes[:data_length]

            frame = header + bytes([command]) + bytes([data_length]) + data_bytes
            try:
                self.ser.write(frame)
                print(f"TX: {frame.hex().upper()}")
            except Exception as e:
                print("Error sending frame:", e)

    def _read_loop(self):
        """
        Background thread loop to continuously read from the UART.
        Received data is placed into a thread-safe queue.
        """
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    self.data_queue.put(data)
                    print(f"RX: {data.hex().upper()}")
                time.sleep(0.01)
            except Exception as e:
                print("Error reading:", e)
                break

    def get_received_data(self):
        """
        Retrieve and concatenate all received UART data from the internal queue.
        
        :return: A bytes object containing all received data, or None if no data.
        """
        received_data = []
        while not self.data_queue.empty():
            received_data.append(self.data_queue.get())
        return b"".join(received_data) if received_data else None


if __name__ == '__main__':
    # Quick test of the UART API.
    uart = UART_API()
    if uart.connect("COM5", 115200):
        uart.send_frame(0x01, b'\x55')
        time.sleep(2)
        data = uart.get_received_data()
        if data:
            print("Received:", data)
        uart.disconnect()
