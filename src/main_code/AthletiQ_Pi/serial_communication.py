from utils import *


class SerialCommunicator:
    """
    Handles UART communication with Arduino to perform an initial startup handshake,
    control servo rotation, and receive the current mode from Arduino.

    Attributes
    ----------
    port : str or None
        The serial port to connect to.
    baud : int or None
        The baud rate for the serial connection.
    ser : serial.Serial or None
        The Serial object used for communication.
    """

    def __init__(self):
        """
        Constructor for SerialCommunicator class.
        """

        self.port = None
        self.baud = None
        self.ser = None

    def comm_init(self, port, baud):
        """
        Initializes the serial connection with the Arduino using the specified port and baud rate.

        :param port: The serial port to connect to the Arduino.
        :type port: str
        :param baud: The baud rate for communication.
        :type baud: int
        :return: None
        """

        self.port = port
        self.baud = baud
        self.ser = serial.Serial(self.port, self.baud)

    def transmit_uart(self, rotateL=False, rotateR=False, sync_start=False):
        """
        Sends a start byte to Arduino for initial handshake,
        or a command to Arduino to rotate the servo.

        Transmits '0x55' for start sync,
        'l' if the person is at the left margin of the frame,
        'r' if the person is at the right margin of the frame,
        or does nothing if the person is in the middle of the frame.

        :param rotateL: True to command Arduino to rotate the servo left.
        :type rotateL: bool
        :param rotateR: True to command Arduino to rotate the servo right.
        :type rotateR: bool
        :param sync_start: True to send a start byte to Arduino to synchronize the system initialization.
        :type sync_start: bool
        :return: The transmitted byte if successful,
                 False if transmission failed,
                 or None if no byte was transmitted.
        :rtype: bytes or bool or None
        """

        if sync_start:
            tx_data = b'\x55'
        elif rotateL:
            tx_data = b'l'
        elif rotateR:
            tx_data = b'r'
        else:
            return None

        try:
            self.ser.write(tx_data)
            return tx_data
        except serial.SerialException as e:
            print(f"UART TX FAILED: {e}")
            return False

    def receive_uart(self):
        """
        Receives the most recent byte via UART communication.
        Expects an ACK byte (0xAA), or the latest mode byte from Arduino and decodes it as UTF-8.

        :return: The decoded mode string if data is available and is UTF-8,
                 raw byte if the data is not UTF-8,
                 or None if no data.
        :rtype: str or byte or None
        """

        if self.ser.in_waiting > 0:
            rx_data = self.ser.read(self.ser.in_waiting)
            rx_data = rx_data[-1:]

            try:
                decoded_rx_data = rx_data.decode('utf-8')
                return decoded_rx_data
            except UnicodeDecodeError:
                return rx_data
        else:
            return None
