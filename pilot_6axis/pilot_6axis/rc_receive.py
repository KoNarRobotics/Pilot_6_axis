# 
# Copyright (c) 2025 Patryk Dudziński
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# 
#  Authors: Patryk Dudziński
# 


import socket
import threading
from typing import Callable
import time
import random


BUFFER_SISE = 1024


class UDPReceiver:
  def __init__(self, port: int):
    self.port = port
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.socket.bind(("", self.port))
    self.receive_event = None
    self.prevoious_time = time.time()

  def start(self):
    threading.Thread(target=self._receive_loop, daemon=True).start()

  def on_receive(self, callback: Callable[["RemoteControler6D"], None]):
    self.receive_event = callback

  def _receive_loop(self):
    while True:
      try:
        data, _ = self.socket.recvfrom(BUFFER_SISE)
        frequency = 1 / (time.time() - self.prevoious_time)
        self.prevoious_time = time.time()
        try:
          data_structure = RemoteControler6D(bytes_data=data.decode("ascii"), signal_frequency=frequency)
          if self.receive_event:
            self.receive_event(data_structure)
        except ValueError:
            pass
      except Exception as e:
          print(f"Error {e}")
          pass


class RemoteControler6D:
  def __init__(self,bytes_data: str =None,signal_frequency: float = 0.0):
    if bytes_data is not None:
      self.__init__from_bytes(bytes_data,signal_frequency)
    else:
      self.__init__from_freq(signal_frequency)

  def __init__from_bytes(self, bytes_data: str, signal_frequency: float):
    self.__bytes = bytes_data
    self.frequency = signal_frequency
    if len(bytes_data) != 62:
        raise ValueError("Invalid data length")

    splited_bytes = bytes_data.split(":")
    if len(splited_bytes) != 18 or splited_bytes[0] != "$RC" or splited_bytes[17] != "#\r":
        raise ValueError("data Error")

    self.joystick_1_x = int(splited_bytes[1])
    self.joystick_1_y = int(splited_bytes[2])
    self.joystick_1_z = int(splited_bytes[3])
    self.joystick_1_btn = int(splited_bytes[4])
    self.joystick_2_x = int(splited_bytes[5])
    self.joystick_2_y = int(splited_bytes[6])
    self.joystick_2_z = int(splited_bytes[7])
    self.joystick_2_btn = int(splited_bytes[8])
    self.button_1 = int(splited_bytes[9])
    self.button_2 = int(splited_bytes[10])
    self.button_3 = int(splited_bytes[11])
    self.button_4 = int(splited_bytes[12])
    self.button_5 = int(splited_bytes[13])
    self.button_6 = int(splited_bytes[14])
    self.button_7 = int(splited_bytes[15])
    self.button_8 = int(splited_bytes[16])

  def __init__from_freq(self,frequency: float = 0.0):
    self.frequency = frequency
    self.button_1 = 0
    self.button_2 = 0
    self.button_3 = 0
    self.button_4 = 0
    self.button_5 = 0
    self.button_6 = 0
    self.button_7 = 0
    self.button_8 = 0
    self.joystick_1_x = 0
    self.joystick_1_y = 0
    self.joystick_1_z = 0
    self.joystick_1_btn = 0
    self.joystick_2_x = 0
    self.joystick_2_y = 0
    self.joystick_2_z = 0
    self.joystick_2_btn = 0

  def __str__(self) -> str:
    return f"frequency={self.frequency:.2f}Hz   Data={self.create_udp_package()}"

  def create_udp_package(self) -> bytes:
    package = (
      f"$RC:{str(int(self.joystick_1_x)).rjust(5)}:{str(int(self.joystick_1_y)).rjust(5)}:{str(int(self.joystick_1_z)).rjust(5)}:"
      f"{self.joystick_1_btn}:{str(int(self.joystick_2_x)).rjust(5)}:{str(int(self.joystick_2_y)).rjust(5)}:"
      f"{str(int(self.joystick_2_z)).rjust(5)}:{self.joystick_2_btn}:{self.button_1}:"
      f"{self.button_2}:{self.button_3}:{self.button_4}:{self.button_5}:"
      f"{self.button_6}:{self.button_7}:{self.button_8}:#\r"
    )
    return package.encode("ascii")