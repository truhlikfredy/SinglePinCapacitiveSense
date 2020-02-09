mkdir build-uno
mkdir build-due
mkdir build-nano
mkdir build-mega
ln -s ./ SinglePinCapacitiveSense
docker run -it --rm -v $(pwd):/workspace -u $(id -u):$(id -g)  aarontc/arduino-builder arduino-builder -compile -hardware /arduino/hardware -tools /arduino/hardware/tools/avr -tools /arduino/tools-builder -fqbn arduino:avr:uno -libraries /workspace/ -verbose -build-path /workspace/build-uno /workspace/examples/PollingThreePins/PollingThreePins.ino
docker run -it --rm -v $(pwd):/workspace -u $(id -u):$(id -g)  aarontc/arduino-builder arduino-builder -compile -hardware /arduino/hardware -tools /arduino/hardware/tools/avr -tools /arduino/tools-builder -fqbn arduino:avr:diecimila:cpu=atmega328 -libraries /workspace/ -verbose -build-path /workspace/build-due /workspace/examples/PollingThreePins/PollingThreePins.ino
docker run -it --rm -v $(pwd):/workspace -u $(id -u):$(id -g)  aarontc/arduino-builder arduino-builder -compile -hardware /arduino/hardware -tools /arduino/hardware/tools/avr -tools /arduino/tools-builder -fqbn arduino:avr:nano:cpu=atmega328 -libraries /workspace/ -verbose -build-path /workspace/build-nano /workspace/examples/PollingThreePins/PollingThreePins.ino
docker run -it --rm -v $(pwd):/workspace -u $(id -u):$(id -g)  aarontc/arduino-builder arduino-builder -compile -hardware /arduino/hardware -tools /arduino/hardware/tools/avr -tools /arduino/tools-builder -fqbn arduino:avr:mega:cpu=atmega2560 -libraries /workspace/ -verbose -build-path /workspace/build-mega /workspace/examples/PollingThreePins/PollingThreePins.ino

docker run -it --rm -v $(pwd):/workspace -u $(id -u):$(id -g)  aarontc/arduino-builder /arduino/hardware/tools/avr/bin/avr-objdump --source --all-headers --demangle --line-numbers --wide ./build-due/PollingThreePins.ino.elf > ./build-due/map-listing.txt
