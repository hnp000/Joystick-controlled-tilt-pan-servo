"""
gui.py

A simple Tkinter GUI to interface with the UART_API for controlling servos.
It allows the user to connect/disconnect, send base/tilt servo angles,
and display debug output.
"""

import tkinter as tk
from tkinter import ttk
import threading
import queue
import time
from uart_api import UART_API

# Create an instance of UART_API and a thread-safe queue for received data
uart = UART_API()
data_queue = queue.Queue()


def receive_uart_data():
    """
    Background thread function to continuously receive UART data
    and enqueue it for GUI processing.
    """
    while True:
        data = uart.get_received_data()
        if data:
            data_queue.put(data)
        time.sleep(0.1)  # Adjust sleep to balance CPU usage and responsiveness


def process_received_data():
    """
    Periodically check the data queue and update the debug output.
    """
    while not data_queue.empty():
        data = data_queue.get()
        log_message(f"RX: {data.hex().upper()}")
    root.after(100, process_received_data)


def connect_disconnect():
    """
    Toggle the connection state of the UART.
    Connect if not connected, disconnect if connected.
    """
    if connect_button["text"] == "Connect":
        port = com_entry.get()
        try:
            baud = int(baud_entry.get())
        except ValueError:
            log_message("Invalid baud rate")
            return
        if uart.connect(port, baud):
            connect_button.config(text="Disconnect")
            log_message(f"Connected to {port} at {baud} baud")
        else:
            log_message("Connection failed")
    else:
        uart.disconnect()
        connect_button.config(text="Connect")
        log_message("Disconnected")


def log_message(message):
    """
    Append a message to the debug text widget and auto-scroll.
    """
    debug_text.insert(tk.END, message + "\n")
    debug_text.see(tk.END)


def send_base():
    """
    Send a base servo angle via UART using command 0x01.
    """
    try:
        degree = int(base_entry.get())
        if 0 <= degree <= 180:
            frame = bytes([degree])
            uart.send_frame(0x01, frame, 1)
            log_message(f"TX: 01 {frame.hex().upper()}")
        else:
            log_message("Base servo degree must be between 0 and 180")
    except ValueError:
        log_message("Invalid base servo degree")


def send_tilt():
    """
    Send a tilt servo angle via UART using command 0x02.
    """
    try:
        degree = int(tilt_entry.get())
        if 0 <= degree <= 180:
            frame = bytes([degree])
            uart.send_frame(0x02, frame, 1)
            log_message(f"TX: 02 {frame.hex().upper()}")
        else:
            log_message("Tilt servo degree must be between 0 and 180")
    except ValueError:
        log_message("Invalid tilt servo degree")


# ----------------------- GUI Setup -----------------------
root = tk.Tk()
root.title("Servo Controller")

mainframe = ttk.Frame(root, padding="10")
mainframe.grid(row=0, column=0, sticky=(tk.N, tk.W, tk.E, tk.S))

# COM Port and Baud Rate
ttk.Label(mainframe, text="COM Port:").grid(row=0, column=0, sticky=tk.W)
com_entry = ttk.Entry(mainframe, width=10)
com_entry.grid(row=0, column=1, sticky=tk.W)
com_entry.insert(0, "COM5")  # Default port

ttk.Label(mainframe, text="Baud Rate:").grid(row=1, column=0, sticky=tk.W)
baud_entry = ttk.Entry(mainframe, width=10)
baud_entry.grid(row=1, column=1, sticky=tk.W)
baud_entry.insert(0, "115200")  # Default baud rate

connect_button = ttk.Button(mainframe, text="Connect", command=connect_disconnect)
connect_button.grid(row=0, column=2, rowspan=2, padx=5)

# Base Servo Controls
ttk.Label(mainframe, text="Base Servo Degree:").grid(row=2, column=0, sticky=tk.W)
base_entry = ttk.Entry(mainframe, width=10)
base_entry.grid(row=2, column=1, sticky=tk.W)
base_entry.insert(0, "90")
base_send_button = ttk.Button(mainframe, text="Send Base", command=send_base)
base_send_button.grid(row=2, column=2, padx=5)

# Tilt Servo Controls
ttk.Label(mainframe, text="Tilt Servo Degree:").grid(row=3, column=0, sticky=tk.W)
tilt_entry = ttk.Entry(mainframe, width=10)
tilt_entry.grid(row=3, column=1, sticky=tk.W)
tilt_entry.insert(0, "90")
tilt_send_button = ttk.Button(mainframe, text="Send Tilt", command=send_tilt)
tilt_send_button.grid(row=3, column=2, padx=5)

# Debug Output
debug_frame = ttk.LabelFrame(mainframe, text="Debug Output", padding="5")
debug_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E))

debug_text = tk.Text(debug_frame, height=10, width=50, wrap=tk.WORD)
debug_text.grid(row=0, column=0, sticky=(tk.W, tk.E))

scrollbar = ttk.Scrollbar(debug_frame, orient=tk.VERTICAL, command=debug_text.yview)
scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
debug_text["yscrollcommand"] = scrollbar.set

# ----------------------- Start Threads and GUI Loop -----------------------
uart_thread = threading.Thread(target=receive_uart_data, daemon=True)
uart_thread.start()
root.after(100, process_received_data)
root.mainloop()
