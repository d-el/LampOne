add_library(dev STATIC
	src/d.c
	)

target_include_directories(dev PUBLIC
	include
	)

target_link_libraries(dev drivers utils freertos)
