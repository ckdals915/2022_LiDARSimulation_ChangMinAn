import socket

print(socket.gethostbyname(socket.gethostname()))
# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server_socket.bind(('192.168.1.100', 2368))
# server_socket.listen(0)

# client_socket, addr = server_socket.accept()

# data = client_socket.recv(66635)

# print("받은 데이터:", data.decode())
