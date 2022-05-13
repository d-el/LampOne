add_library(drivers STATIC
	src/adc.c
	src/clock.c
	src/drivers.c
	src/flash.c
	src/gpio.c
	src/pwm.c
	src/uart.c
	)

target_include_directories(drivers PUBLIC
	include
	)

target_link_libraries(drivers cmsis)

target_link_options(drivers PUBLIC
	"-u DMA1_Channel1_IRQHandler"
	)