# -*- coding: utf-8 -*-

from time import sleep
import socket

data = "Hello World"
data = data + str(100)

print(data)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

host = '192.168.1.9'
port = 5900

client.connect((host, port))
received_bytes = client.recv(1024)
result_string = received_bytes.decode("utf8")
print("Received Data = {}".format(result_string))
#client_input = input("What do you want ot proceed with Server?\n");
number = 0

while(1):
    client_input = str(number)
    client.send(client_input.encode("utf8"));
    received_bytes = client.recv(1024)
    result_string = received_bytes.decode("utf8")
    print("Received Data = {}".format(result_string))
    number = number + 1

# Close socket
client.close()
