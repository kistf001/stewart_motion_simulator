import struct
import socket
import time

msgFromClient       = struct.pack('!fff'+'fff',323,123,321,323,123,321)
bytesToSend         = msgFromClient
serverAddressPort   = ("127.0.0.1", 20001)
bufferSize          = 1024

# 클라이언트 쪽에서 UDP 소켓 생성
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

for afd in range(0,2):
    # 생성된 UDP 소켓을 사용하여 서버로 전송
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)

    msgFromServer = UDPClientSocket.recvfrom(bufferSize)

    msg = "Message from Server {}".format(msgFromServer[0])
    print(msg)



## 클라이언트 쪽에서 UDP 소켓 생성
#UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)