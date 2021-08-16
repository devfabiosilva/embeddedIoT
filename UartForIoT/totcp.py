import socket

class ToTcp:
    def __init__(self, address, port):
        sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address=(address, port)
        print('Connecting to {} port {}'.format(*server_address))
        sock.connect(server_address)
        self.sock=sock

    def close(self):
        self.sock.close()

    def send(self, message):
        self.sock.sendall(message)