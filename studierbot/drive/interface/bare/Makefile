CC=g++
CFLAGS=-Wall -std=c++11 -pthread -g
INC=..
SRC_COMMON=../SerialPort.cpp ../control.cpp
HDR_COMMON=../SerialPort.h ../protocol.h ../control.h
SRC_CAN=../MotorControllerCAN.cpp ../SocketCAN.cpp ../SocketCANObserver.cpp
HDR_CAN=../MotorControllerCAN.h ../SocketCAN.h ../SocketCANObserver.h
SRC_OPEN_LOOP=step_response_open_loop.cpp
SRC_CLOSED_LOOP=step_response_closed_loop.cpp
SRC_CAN_TEST=can_test.cpp
EXECUTABLE_OPEN_LOOP=$(SRC_OPEN_LOOP:.cpp=)
EXECUTABLE_CLOSED_LOOP=$(SRC_CLOSED_LOOP:.cpp=)
EXECUTABLE_CAN_TEST=$(SRC_CAN_TEST:.cpp=)

all: $(EXECUTABLE_OPEN_LOOP) $(EXECUTABLE_CLOSED_LOOP) $(EXECUTABLE_CAN_TEST)

$(EXECUTABLE_OPEN_LOOP): $(SRC_OPEN_LOOP) $(SRC_COMMON) $(HDR_COMMON)
	$(CC) $(CFLAGS) -I $(INC) $(SRC_OPEN_LOOP) $(SRC_COMMON) -o $@

$(EXECUTABLE_CLOSED_LOOP): $(SRC_CLOSED_LOOP) $(SRC_COMMON) $(HDR_COMMON)
	$(CC) $(CFLAGS) -I $(INC) $(SRC_CLOSED_LOOP) $(SRC_COMMON) -o $@

$(EXECUTABLE_CAN_TEST): $(SRC_CAN_TEST) $(SRC_CAN) $(HDR_CAN)
	$(CC) $(CFLAGS) -I $(INC) $(SRC_CAN_TEST) $(SRC_CAN) -o $@
		
clean:
	rm $(EXECUTABLE_OPEN_LOOP) $(EXECUTABLE_CLOSED_LOOP) $(EXECUTABLE_CAN_TEST)