# Quick Start
For enabling CAN communication from a Linux host using the USBtin module, one needs to attach and startup the socket can interface.
Ensure, that the package can-utils is installed.
```bash
    $ sudo apt install can-utils
```
In order to get a fixed file device interface, you need to activate specific udev rules. Please copy the slcand udev file to your system folder:
```bash
    $ sudo cp drive/udev/77-evo-slcand.rules /etc/udev/rules.d/
    $ sudo udevadm control --reload-rules
```
Replug the USBtin module or reboot your computer. Then you can execute the following script:
```bash
    $ ./drive/interface/can_up.sh
```
If CAN communication is enabled, the nodes for your specific kinematics (mechanum or skid steering) can be executed.
Adjust the launch files before.

# Reference design

## Component diagram
The lowest communication layer is based on the Control Area Network (CAN-Bus). Either, the robot's main controller provides a CAN-interface or a piece of additional hardware is needed. Below mentioned is a USBtin module, that can be connected via USB to a commodity linux hardware, e.g. an Intel NUC or a Jetson TX2 board. Interfacing the USBtin hardware is done with the Linux SocketCAN implementation.

![See umlet diagram for an overview](doc/evorobot_component.png "Component diagram of reference design")

## Class diagram
The communication to the hardware is handled by the class `SocketCAN`. The `registerObserver` method takes a pointer to an instance implementing the interface `SocketCANObserver`. Two concrete classes are drawn below using this interface: `MotorcontrollerCAN` and `AddonShieldCAN`, which relate to the Evocortex DC Motorcontroller and the Evocortex DCDC-Converter board.

Further, two different kinematic concepts are implemented: `MechanumSteering` and `SkidSteering`. The mechanum concept uses two motorcontrollers (having four motors). The skid steering concept related to a six-wheeled robot (but can be easily adapted to a chain-driven or four-wheeled robot concept).

![See umlet diagram for an overview](doc/evorobot_class.png "Class diagram of reference design")
