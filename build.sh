# So the example canbe used to test the library
ln -s ./examples/PollingThreePins/PollingThreePins.ino SinglePinCapacitiveSense.ino
#ln -s ./examples/PollingThreePins/MegaADK.ino SinglePinCapacitiveSense.ino
#ln -s ./examples/PollingThreePins/Debounced.ino SinglePinCapacitiveSense.ino

mkdir build-uno
mkdir build-due
mkdir build-nano
mkdir build-mega
mkdir build-megaADK

# So the CLI will see in the project's root folder a library with the expected name
ln -s ./ SinglePinCapacitiveSense

# For the FQBN definitions look at this file:
# <ARDUINO_PATH>/hardware/arduino/avr/boards.txt

echo "Compile uno"
docker run -it --rm -v $(pwd):/workspace -u $(id -u):$(id -g)  aarontc/arduino-builder arduino-builder -compile -hardware /arduino/hardware -tools /arduino/hardware/tools/avr -tools /arduino/tools-builder -fqbn arduino:avr:uno -libraries /workspace/ -verbose -build-path /workspace/build-uno /workspace/examples/PollingThreePins/PollingThreePins.ino

echo "Compile due"
docker run -it --rm -v $(pwd):/workspace -u $(id -u):$(id -g)  aarontc/arduino-builder arduino-builder -compile -hardware /arduino/hardware -tools /arduino/hardware/tools/avr -tools /arduino/tools-builder -fqbn arduino:avr:diecimila:cpu=atmega328 -libraries /workspace/ -verbose -build-path /workspace/build-due /workspace/examples/PollingThreePins/PollingThreePins.ino

echo "Compile nano"
docker run -it --rm -v $(pwd):/workspace -u $(id -u):$(id -g)  aarontc/arduino-builder arduino-builder -compile -hardware /arduino/hardware -tools /arduino/hardware/tools/avr -tools /arduino/tools-builder -fqbn arduino:avr:nano:cpu=atmega328 -libraries /workspace/ -verbose -build-path /workspace/build-nano /workspace/examples/PollingThreePins/PollingThreePins.ino

echo "Compile mega2560"
docker run -it --rm -v $(pwd):/workspace -u $(id -u):$(id -g)  aarontc/arduino-builder arduino-builder -compile -hardware /arduino/hardware -tools /arduino/hardware/tools/avr -tools /arduino/tools-builder -fqbn arduino:avr:mega:cpu=atmega2560 -libraries /workspace/ -verbose -build-path /workspace/build-mega /workspace/examples/PollingThreePins/PollingThreePins.ino

echo "Compile megaADK"
docker run -it --rm -v $(pwd):/workspace -u $(id -u):$(id -g)  aarontc/arduino-builder arduino-builder -compile -hardware /arduino/hardware -tools /arduino/hardware/tools/avr -tools /arduino/tools-builder -fqbn arduino:avr:megaADK -libraries /workspace/ -verbose -build-path /workspace/build-megaADK /workspace/examples/MegaADK/MegaADK.ino

echo "Generate map list for due"
docker run -it --rm -v $(pwd):/workspace -u $(id -u):$(id -g)  aarontc/arduino-builder /arduino/hardware/tools/avr/bin/avr-objdump --source --all-headers --demangle --line-numbers --wide ./build-due/PollingThreePins.ino.elf > ./build-due/map-listing.txt


