from typing import override
from lego_spike_interface.interfaces import LegoInterfaceNode
import serial


class SerialInterfaceNode(LegoInterfaceNode):
    """Handles bidirectional communication over the USB virtual com port of the Lego Hub"""

    def __init__(self, port="/dev/lego", baud=115200, verbose=False):
        super().__init__()
        self.logger = self.get_logger()
        self.verbose = verbose

        self.port = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
        )

    def open(self):
        print("open")
        if self.port.isOpen():
            self.logger.debug("port is already open")
            return
        try:
            self.port.open()
        except serial.SerialException as err:
            self.logger.error(err)

    def close(self):
        if not self.port.isOpen():
            self.logger.debug("port is already closed")
            return
        try:
            self.port.close()
        except Exception as err:
            self.logger.error(err)

    @override
    def read_line(self):
        "Reads a single line of text from the micropython interpreter"
        l = self.port.readline()
        l = l.decode("utf-8").rstrip()
        if self.verbose:
            self.logger.info(l)
        return l

    def write_line(self, txt):
        "Writes a single line of text with newline terminator to the micropython interpreter"
        data = bytes(txt, encoding="utf-8")
        self.port.write(data)

    def write_byte(self, ch):
        "Writes a single character to the micropython interpreter"
        self.port.write(ch)
