add_library(systems STATIC
	src/debugCore.c
	src/memmgr.c
	src/printp.c
	src/semihosting.c
	src/startup_stm32g030xx.c
	src/syscall.c
	)

target_include_directories(systems PUBLIC
	include
	)

target_link_libraries(systems cmsis drivers freertos)

target_link_options(systems PUBLIC
	-Wl,--undefined=__wrap__malloc_r
	-Wl,--wrap,_malloc
	-Wl,--wrap,_free
	-Wl,--wrap,_calloc
	-Wl,--wrap,_realloc
	-Wl,--wrap,_malloc_r
	-Wl,--wrap,_free_r
	-Wl,--wrap,_calloc_r
	-Wl,--wrap,_realloc_r
	)
