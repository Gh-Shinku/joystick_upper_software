import socket
import threading
import queue
from handle_msg import handleMsg

class TCPSocket:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = None
        self.receive_thread = None
        self.running = False
        self.queue = queue.Queue()

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(0.5)
            self.socket.connect((self.host, self.port))
            self.running = True
            self.receive_thread = threading.Thread(target=self.receive_data)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            print(f"Connected to {self.host}:{self.port}")
        except Exception as e:
            print(f"Failed to connect to {self.host}:{self.port}, error: {e}")

    def receive_data(self):
        try:
            while self.running:
                self.socket.send(bytes("1".encode()))
                data = self.socket.recv(1024)
                if data:
                    self.queue.put(data)
                else:
                    break
        except Exception as e:
            print(f"Error receiving data: {e}")
        finally:
            self.disconnect()

    def disconnect(self):
        self.running = False
        if self.socket:
            self.socket.close()
            self.socket = None
        print("Disconnected")

    def get_queue(self):
        return self.queue


# Example usage
if __name__ == "__main__":
    tcp_socket = TCPSocket("192.168.4.1", 3456)
    tcp_socket.connect()
    q = tcp_socket.get_queue()
    msg = handleMsg("TCP")

    # Example of processing data from the queue
    try:
        while True:
            if not q.empty():
                byte_datas = q.get()
                # print(buffer)
                code = msg.calc(byte_datas,"TCP")
                if code == -1:
                    print("SOF or EOF Error")
                elif code == -2:
                    print("CRC32 Error")
                else:
                    msg.print()
    except KeyboardInterrupt:
        print("Stopping")
        tcp_socket.disconnect()
