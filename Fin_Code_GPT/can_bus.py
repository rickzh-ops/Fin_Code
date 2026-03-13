"""
Module: CAN Bus Communication
Author: He
Original Ref: fin.cpp / hal_it.cpp
Description: 
    CAN bus communication module. Responsible for 
    parsing wind speed messages from the vehicle 
    network and packaging the system's current status
    (such as wing surface angle and elevator position) 
    for transmission to the CAN bus.
"""
import can
import time

_bus = None

#Initialize
def init_can(channel='can0', bitrate=500000):
    global _bus
    try:
        _bus = can.interface.Bus(channel=channel, bustype='socketcan', bitrate=bitrate)
        print(f"CAN Interface {channel} initialized at {bitrate}bps")
    except Exception as e:
        print(f"Failed to init CAN: {e}")
        _bus = None

#Send
def send_can_message(msg_id, data):
    if _bus is None:
        return
    msg = can.Message(
        arbitration_id=msg_id,
        data=data,
        is_extended_id=False
    )
    try:
        _bus.send(msg)
    except can.CanError as e:
        print(f"Message failed to send: {e}")

#Read
def read_can_message(timeout=0.01):
    if _bus is None:
        return None
    try:
        return _bus.recv(timeout)
    except Exception:
        return None

#Close
def close_can():
    global _bus
    if _bus:
        _bus.shutdown()
        _bus = None
