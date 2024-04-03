import serial
import threading
import queue
from handle_msg import handleMsg


class SerialReader:
    def __init__(self, port, baudrate, external_queue):
        """
        初始化串口读取器。

        :param port: 串口名称，如 'COM3' 或 '/dev/ttyUSB0'
        :param baudrate: 波特率，如 9600
        :param external_queue: 用于存储接收到的字节的外部队列
        """
        self.port = port
        self.baudrate = baudrate
        self.external_queue = external_queue
        self.running = False
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"已连接到串口 {self.port}，波特率 {self.baudrate}")
        except serial.SerialException as e:
            print(f"打开串口时发生错误: {e}")
            raise

    def start_reading(self):
        """
        开始读取串口数据。
        """
        self.running = True
        self.thread = threading.Thread(target=self.read_from_port)
        self.thread.daemon = True
        self.thread.start()

    def read_from_port(self):
        """
        从串口读取数据，并将数据放入外部队列。
        """
        def find_frame_end():
            while self.running:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(1)
                    if ord(data) == 0x2A:
                        break
        self.ser.reset_input_buffer()
        try:
            find_frame_end()
            while self.running:
                if self.ser.in_waiting >= 22:
                    data = self.ser.read(22)
                    if data[0] != 0x2B:
                        # self.ser.reset_input_buffer()
                        find_frame_end()
                        continue
                    self.external_queue.put(data)
        except serial.SerialException as e:
            print(f"从串口读取数据时发生错误: {e}")
            self.running = False
        finally:
            self.ser.close()

    def stop_reading(self):
        """
        停止读取数据。
        """
        self.running = False
        if self.thread.is_alive():
            self.thread.join()


# 示例使用
if __name__ == "__main__":
    q = queue.Queue()
    serial_reader = SerialReader('COM6', 115200, q)
    serial_reader.start_reading()
    msg = handleMsg("Serial")

    try:
        while True:
            if not q.empty():
                byte_datas = q.get()
                code = msg.calc(byte_datas, "Serial")
                if code == -1:
                    print("SOF or EOF Error")
                elif code == -2:
                    print("CRC32 Error")
                elif code == -3:
                    pass
                else:
                    msg.print()

    except KeyboardInterrupt:
        print("停止读取串口数据")
        serial_reader.stop_reading()
