
import socket

number = 0

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)

server.bind(('192.168.1.3', 5901))
server.listen(5)
print('Socket now listening')

while True:
    conn, addr = server.accept()
    ip, port = str(addr[0]), str(addr[1])
    print('Accepting connection from ' + ip + ':' + port)
    while True:
        input_from_client_bytes = conn.recv(1024)
        result_string = input_from_client_bytes.decode("utf8")
        print('Received Data = {}{}'.format(result_string, number))
        number = number + 1
    conn.close()
server.close()
