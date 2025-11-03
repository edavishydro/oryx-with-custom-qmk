FROM debian:latest

RUN apt update && apt install -y git python3 python3-pip sudo wget

RUN python3 -m pip install qmk appdirs --break-system-packages

WORKDIR /root

# Copy the qmk_firmware submodule from the host
# This is your fork with Oryx + community modules
COPY qmk_firmware /root/qmk_firmware

# Install QMK dependencies
RUN cd qmk_firmware && util/qmk_install.sh

WORKDIR /root/qmk_firmware
