
############################
# ==== SERVER RECIVED ==== #
# char identifier
# char version
# char operationId
#
# identifier
# 0 : reserved
# 1 : reserved
# 2 : reserved
# version
# 0 : reserved
# 1 : now
# 2 : reserved
# operationId
# 0 : handShake
# 1 : live cheack
# 2 : motion action
# 3 : setting
#
# ==== operationId => motion action => (only)
# float acc[3]  *operationId 2 only
# float gyo[3]  *operationId 2 only
#
# acc Array
# * : XYZ accelation
# gyo Array
# * : XYZ angulation
#
############################


############################
# ==== SERVER SEND handShake ==== #
# char motion_name
# char motion_type
# ==== SERVER SEND live cheack ==== #
# char SEND = 39
# ==== SERVER SEND motion action ==== #
# ==== SERVER SEND setting ==== #
# int A
# int B
# int C
# int D
############################


#localIP     = "127.0.0.1"
#localPort   = 20001
#bufferSize  = 1024
#
#msgFromServer       = "Hello UDP Client"
#bytesToSend         = str.encode(msgFromServer)
#
## 데이터그램 소켓을 생성
#UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
#
## 주소와 IP로 Bind
#UDPServerSocket.bind((localIP, localPort))
#
#print("I AM MOTION SIMULATOR")
#
## 들어오는 데이터그램 Listen
#for afd in range(0,2):
#    import time
#    start = time.time()  # 시작 시간 저장
#
#    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
#    message = bytesAddressPair[0]
#    address = bytesAddressPair[1]
#
#    print("message : ", struct.unpack('!ffffff',message))
#    print("address : ", address)
#
#    # Sending a reply to client
#    UDPServerSocket.sendto(bytesToSend, address)
#    print("time :", time.time() - start)  # 현재시각 - 시작시간 = 실행 시간
#