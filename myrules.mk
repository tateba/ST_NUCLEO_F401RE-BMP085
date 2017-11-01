
program:
	@echo Trying to flash STM32F401RE NUCLEO Board.
	st-flash write build/ch.bin 0x8000000
	@echo
	@echo Done

connect:
	picocom -b 115200 /dev/ttyACM0

fullclean:
	@echo Delate all vim backup files.
	rm *~
	@echo
	@echo Done.
