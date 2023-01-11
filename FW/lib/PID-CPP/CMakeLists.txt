add_library(pid-cpp STATIC
	src/pid.cpp
	)

target_include_directories(pid-cpp
	PUBLIC src
	)
