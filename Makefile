# CREATED BY VIM-PIO
all:
	platformio -f -c vim run

upload:
	platformio -f -c vim run --target upload

clean:
	platformio -f -c vim run --target clean

program:
	platformio -f -c vim run --target program

uploadfs:
	platformio -f -c vim run --target uploadfs

serial:
	platformio device monitor -b 115200

runserial:
	platformio -f -c vim run --target upload
	platformio device monitor -b 115200

index:
	make clean
	bear -- make
