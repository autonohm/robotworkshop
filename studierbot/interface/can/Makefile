all:
		echo 'No rule for make all existing. Please run "sudo make install" to install the udev rule and systemd service.'

install:
		install -D -m644 77-evo-slcand.rules /etc/udev/rules.d/77-evo-slcand.rules
		install -D -m644 slcand.service /etc/systemd/system/slcand.service

remove:
		remove /etc/udev/rules.d/77-evo-slcand.rules
		remove /etc/systemd/system/slcand.service