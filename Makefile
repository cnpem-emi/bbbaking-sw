PREFIX ?= /usr/local

SERVICE_FILE_DEST = /etc/systemd/system
SERVICE_NAME = bbbaking

.PHONY: install uninstall


install:
	# Install Service
	cp --preserve=mode service_beaglebone/${SERVICE_NAME}.service ${SERVICE_FILE_DEST}

	systemctl daemon-reload
	
	cd src/spixxcon_library; python setup.py install; cd ../../

	systemctl stop ${SERVICE_NAME}
	systemctl start ${SERVICE_NAME}
	systemctl enable ${SERVICE_NAME}


uninstall:
	systemctl stop ${SERVICE_NAME}
	rm -f ${SERVICE_FILE_DEST}/${SERVICE_NAME}.service

	systemctl daemon-reload
