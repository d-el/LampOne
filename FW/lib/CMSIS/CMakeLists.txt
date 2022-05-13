add_library(cmsis STATIC
	Device/ST/STM32G0xx/Source/Templates/system_stm32g0xx.c
	
	DSP/Source/ControllerFunctions/arm_pid_init_q15.c
	DSP/Source/ControllerFunctions/arm_pid_init_q31.c
	DSP/Source/ControllerFunctions/arm_pid_reset_q15.c
	DSP/Source/ControllerFunctions/arm_pid_reset_q31.c
	
	DSP/Source/SupportFunctions/arm_copy_f32.c
	DSP/Source/SupportFunctions/arm_copy_q15.c
	DSP/Source/SupportFunctions/arm_copy_q31.c
	DSP/Source/SupportFunctions/arm_copy_q7.c
	DSP/Source/SupportFunctions/arm_fill_f32.c
	DSP/Source/SupportFunctions/arm_fill_q15.c
	DSP/Source/SupportFunctions/arm_fill_q31.c
	DSP/Source/SupportFunctions/arm_fill_q7.c
	DSP/Source/SupportFunctions/arm_float_to_q15.c
	DSP/Source/SupportFunctions/arm_float_to_q31.c
	DSP/Source/SupportFunctions/arm_float_to_q7.c
	DSP/Source/SupportFunctions/arm_q15_to_float.c
	DSP/Source/SupportFunctions/arm_q15_to_q31.c
	DSP/Source/SupportFunctions/arm_q15_to_q7.c
	DSP/Source/SupportFunctions/arm_q31_to_float.c
	DSP/Source/SupportFunctions/arm_q31_to_q15.c
	DSP/Source/SupportFunctions/arm_q31_to_q7.c
	DSP/Source/SupportFunctions/arm_q7_to_float.c
	DSP/Source/SupportFunctions/arm_q7_to_q15.c
	DSP/Source/SupportFunctions/arm_q7_to_q31.c
	DSP/Source/SupportFunctions/SupportFunctions.c
	)

target_include_directories(cmsis PUBLIC
	Include
	Device/ST/STM32G0xx/Include
	DSP/Include
	)
