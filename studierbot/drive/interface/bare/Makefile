CC=g++
CFLAGS=-c -Wall
INC=..
SRC_COMMON=../SerialPort.cpp ../SerialPort.h ../protocol.h ../control.cpp ../control.h
SRC_CAN=../MotorControllerCAN.cpp ../MotorControllerCAN.h ../MotorController.cpp ../MotorController.h
SRC_OPEN_LOOP=step_response_open_loop.cpp
SRC_CLOSED_LOOP=step_response_closed_loop.cpp
SRC_CAN_TEST=can_test.cpp
EXECUTABLE_OPEN_LOOP=$(SRC_OPEN_LOOP:.cpp=)
EXECUTABLE_CLOSED_LOOP=$(SRC_CLOSED_LOOP:.cpp=)
EXECUTABLE_CAN_TEST=$(SRC_CAN_TEST:.cpp=)

all: $(EXECUTABLE_OPEN_LOOP) $(EXECUTABLE_CLOSED_LOOP) $(EXECUTABLE_CAN_TEST)

$(EXECUTABLE_OPEN_LOOP): $(SRC_OPEN_LOOP) $(SRC_COMMON)
	$(CC) -I $(INC) $(SRC_OPEN_LOOP) $(SRC_COMMON) -o $@

$(EXECUTABLE_CLOSED_LOOP): $(SRC_CLOSED_LOOP) $(SRC_COMMON)
	$(CC) -I $(INC) $(SRC_CLOSED_LOOP) $(SRC_COMMON) -o $@

$(EXECUTABLE_CAN_TEST): $(SRC_CAN_TEST) $(SRC_CAN)
	$(CC) -I $(INC) $(SRC_CAN_TEST) $(SRC_CAN) -o $@
		
clean:
	rm $(EXECUTABLE_OPEN_LOOP) $(EXECUTABLE_CLOSED_LOOP) $(EXECUTABLE_CAN_TEST)