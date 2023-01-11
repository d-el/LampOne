add_library(task STATIC
	src/adcTSK.cpp
	src/modbusTSK.cpp
	src/monitorTSK.c
	src/systemTSK.cpp
	)

target_include_directories(task PUBLIC
	include
	)
target_compile_options(task PUBLIC -Wno-missing-field-initializers)
target_link_libraries(task freertos utils drivers dev systems prmsystem freemodbus math pid-cpp)
