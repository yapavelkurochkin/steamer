AM_CFLAGS = -Wall

dfu-version.h:
	echo -e '#ifndef DFU_UTIL_VERSION' \
	'\n#define DFU_UTIL_VERSION "'`svnversion`'"' \
	'\n#endif' > dfu-version.h
BUILT_SOURCES = dfu-version.h

bin_PROGRAMS = dfu-util dfu-util_static
dfu_util_SOURCES = main.c dfu.c dfu.h stm32dfu.c

dfu_util_static_SOURCES = main.c dfu.c dfu.h stm32dfu.c
dfu_util_static_LDFLAGS = -static

