import copy
import queue
import threading
import time

from handle_msg import handleMsg
from Socket import TCPSocket
from UART import SerialReader

current_message = handleMsg("Undefined")
lock = threading.Lock()
exit_signal =  threading.Event()


def update_message(new_message):
    global current_message
    with lock:
        # 根据Stamp值和来源更新当前消息
        if (new_message.stamp > current_message.stamp) or \
                (new_message.stamp == current_message.stamp and new_message.source == "TCP"):
            current_message = copy.deepcopy(new_message)
            current_message.print()


def tcp_receiver(exit_signal):
    tcp_socket = TCPSocket("192.168.4.1", 3456)
    tcp_socket.connect()
    q = tcp_socket.get_queue()
    new_msg = handleMsg("TCP")
    try:
        while not exit_signal.is_set():
            if not q.empty():
                byte_datas = q.get()
                code = new_msg.calc(byte_datas, "TCP")
                if code == -1:
                    print("SOF or EOF Error")
                elif code == -2:
                    print("CRC32 Error")
                else:
                    update_message(new_msg)
    finally:
        print("Stopping")
        tcp_socket.disconnect()


def serial_receiver(exit_signal):
    q = queue.Queue()
    serial_reader = SerialReader('COM6', 115200, q)
    serial_reader.start_reading()
    new_msg_serial = handleMsg("Serial")
    try:
        while not exit_signal.is_set():
            if not q.empty():
                byte_datas = q.get()
                code = new_msg_serial.calc(byte_datas, "Serial")
                if code == -1:
                    print("SOF or EOF Error")
                elif code == -2:
                    print("CRC32 Error")
                elif code == -3:
                    pass
                else:
                    update_message(new_msg_serial)
    finally:
        print("停止读取串口数据")
        serial_reader.stop_reading()


if __name__ == "__main__":
    # 创建线程
    tcp_thread = threading.Thread(target=tcp_receiver, args=(exit_signal,))
    # serial_thread = threading.Thread(target=serial_receiver, args=(exit_signal,))

    # 启动线程
    tcp_thread.start()
    # serial_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        exit_signal.set()
        tcp_thread.join()
        # serial_thread.join()
    print("Exited")
