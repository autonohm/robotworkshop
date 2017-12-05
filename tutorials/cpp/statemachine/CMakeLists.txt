cmake_minimum_required (VERSION 2.6)
PROJECT(STATEMACHINE)
INCLUDE_DIRECTORIES(. states)

ADD_EXECUTABLE(statemachine_test
               main.cpp
               StateMachine.cpp
               states/StatePing.cpp
               states/StatePong.cpp
               states/StateBase.cpp)