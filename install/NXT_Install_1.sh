#!/bin/bash
sudo groupadd lego

sudo usermod -a -G lego $SUDO_USER

echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0694\", GROUP=\"lego\", MODE=\"0660\"" > /tmp/70-lego.rules && sudo mv /tmp/70-lego.rules /etc/udev/rules.d/70-lego.rules

sudo restart udev

echo "Please onfigure your Ubuntu repositories to allow 'restricted,' 'universe,' and 'multiverse.' by clicking System, Administration, Software sources. Enable 'Community maintained...' and 'Software restricted...'"


