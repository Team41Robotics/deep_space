#!/bin/bash

echo "Installing necessary services..."
sudo cp ./files/*.sh /usr/local/bin/
sudo cp ./files/*.service /etc/systemd/system/

echo "Refreshing systemctl daemon..."
sudo systemctl daemon-reload

echo "Starting and enabling services..."
for filename in ./files/*.service; do
	servicename=$(basename $filename .service)
	echo "  Starting $servicename"
	sudo systemctl restart $servicename
	echo "  Enabling $servicename"
	sudo systemctl enable $servicename
done

echo "Done."
