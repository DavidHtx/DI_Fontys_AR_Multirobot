import serial
import threading
import time

SERIAL_TIMEOUT = 0.1  # Serial read timeout (seconds)

class SerialError(Exception):
  pass

class SerialInterface(object):

  """A wrapper around pySerial for use with robot platform"""

  def __init__(self, tty, baudrate):
    self.ser = None
    try:
      self.ser = serial.Serial(tty, baudrate, 8, 'N', 1, SERIAL_TIMEOUT)
    except Exception, err:
    #if self.ser == None:
      raise SerialError('ERROR trying to open serial port')
    self.lock = threading.RLock()

  def __del__(self):
    if not self.ser == None:
      self.ser.close()

  def send(self, bytes):
    """Send a string to the robot."""
    sent = self.ser.write(bytes)
    if not sent==len(bytes):
      raise SerialError('ERROR writing to serial port.')

  def read(self):
    data = self.ser.readline()
    if len(data) > 0:
        # print('in raw:' + str(data))
        if (data[0] == 255):
            data[0] = 32
        data_decoded = data.decode('utf-8')  # decode to ascii
        # print('in:' + data_decoded)
        return(data_decoded)  # parse incoming string

  def flush_input(self):
    """Flush input buffer."""
    self.ser.flushInput()


def send_receive(ser, command):
  ser.send(command +  "\r\n")
  waiting = True
  while waiting:
    data = ser.ser.readline()
    if len(data) > 0:
      waiting = False
  return data


class robot(object):
  def __init__(self):
    start = int(round(time.time() * 1000))
    #ser.send("$0")
    #ser.send("$1")
    self.ser = SerialInterface("/dev/ttyUSB0", "115200")
    data = send_receive(self.ser, "$29")
    data = send_receive(self.ser, "$10")
    print(data)
    data = send_receive(self.ser, "$11")
    print(data)
    data = send_receive(self.ser, "$59")
    print(data)
    data = send_receive(self.ser, "$58")
    print(data)
    data = send_receive(self.ser, "$14")
    print(data)
    data = send_receive(self.ser, "$15")
    print(data)
    data = send_receive(self.ser, "$2, 8, 0")
    print(data)



  def send_speed(self, speed):
    data = send_receive(self.ser, "$29")
    data = send_receive(self.ser, "$10")
    print(data)
    data = send_receive(self.ser, "$11")
    print(data)
    data = send_receive(self.ser, "$59")
    print(data)
    data = send_receive(self.ser, "$58")
    print(data)
    data = send_receive(self.ser, "$14")
    print(data)
    data = send_receive(self.ser, "$15")
    print(data)
    data = send_receive(self.ser, "$2, 8, 0")
    print(data)
