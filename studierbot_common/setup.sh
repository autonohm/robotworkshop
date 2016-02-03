sudo echo "18" > /sys/class/gpio/export
sudo echo "23" > /sys/class/gpio/export
sudo echo "24" > /sys/class/gpio/export
sudo echo "25" > /sys/class/gpio/export

sudo echo "in" > /sys/class/gpio/gpio18/direction
sudo echo "in" > /sys/class/gpio/gpio23/direction
sudo echo "in" > /sys/class/gpio/gpio24/direction
sudo echo "in" > /sys/class/gpio/gpio25/direction
