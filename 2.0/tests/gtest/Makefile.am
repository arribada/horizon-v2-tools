basedir=$(top_srcdir)/..
core_srcdir=$(basedir)/core
devices_srcdir=$(basedir)/devices
syshal_srcdir=$(basedir)/syshal
gtest_srcdir=$(top_srcdir)/gtest
ports_srcdir=$(basedir)/ports/stm32
boards_srcdir=$(ports_srcdir)/boards/arribada_v2
CMSIS_srcdir=$(ports_srcdir)/drivers/CMSIS
device_srcdir=$(CMSIS_srcdir)/Device/ST/STM32F0xx
hal_srcdir=$(ports_srcdir)/drivers/STM32F0xx_HAL_Driver
hw_platform=arribada_v1
cpu_platform=stm32

AM_CPPFLAGS = -I$(syshal_srcdir)/inc \
              -I$(top_srcdir)/gtest/mocks \
              -g -Wall -DUNITTESTING -O0 \
              -DBASEDIR=\"$(abspath $(gtest_srcdir))/\" \
              -DSTM32F072xB \
              -DGTEST \
              -DDEBUG_DISABLED

HW_DEFS =
AM_CXXFLAGS = -std=c++0x -g -pg -fprofile-arcs -ftest-coverage \
              -I$(GTEST_INCLUDE) -O0 $(HW_DEFS) -DGTEST -DDEBUG_DISABLED

AM_CFLAGS = -std=gnu99 -fprofile-arcs -ftest-coverage -O0 $(HW_DEFS) -DGTEST -DCMOCK_MEM_SIZE=0x400000 -DUNITY_USE_FLUSH_STDOUT

COMMON_LDADD = -L$(GTEST_LIBS) -lgtest -lgtest_main -lpthread -lgcov -lrt

if BUILD_UNITTEST
EXT_TEST_SRCS =
TESTS =
TESTS_ENVIRONMENT = SRCDIR=$(top_srcdir)
check_PROGRAMS =

check_PROGRAMS += Buffer
nodist_Buffer_SOURCES = $(gtest_srcdir)/Buffer.cpp \
                        $(core_srcdir)/buffer/buffer.c

Buffer_LDADD = $(COMMON_LDADD)
Buffer_CPPFLAGS = $(AM_CPPFLAGS) -I$(core_srcdir)/buffer

TESTS += Buffer.test

check_PROGRAMS += Fs
nodist_Fs_SOURCES = $(gtest_srcdir)/Fs.cpp \
                    $(gtest_srcdir)/mocks/unity.c \
                    $(gtest_srcdir)/mocks/cmock.c \
                    $(gtest_srcdir)/mocks/Mocksyshal_flash.c \
                    $(core_srcdir)/fs/fs.c

Fs_LDADD = $(COMMON_LDADD)
Fs_CPPFLAGS = $(AM_CPPFLAGS) -I$(core_srcdir)/fs

TESTS += Fs.test

check_PROGRAMS += Flash
nodist_Flash_SOURCES = $(gtest_srcdir)/Flash.cpp \
                       $(gtest_srcdir)/mocks/cmock.c \
                       $(gtest_srcdir)/mocks/unity.c \
                       $(gtest_srcdir)/mocks/Mocksyshal_gpio.c \
                       $(gtest_srcdir)/mocks/Mocksyshal_spi.c \
                       $(devices_srcdir)/flash/S25FL128/S25FL128.c

Flash_LDADD = $(COMMON_LDADD)
Flash_CPPFLAGS = $(AM_CPPFLAGS) \
                 -I$(CMSIS_srcdir)/Device/ST/STM32F0xx/Include \
                 -I$(CMSIS_srcdir)/Include \
                 -I$(ports_srcdir) \
                 -I$(hal_srcdir)/Inc \
                 -I$(boards_srcdir) \
                 -I$(core_srcdir)/debug \
                 -I$(devices_srcdir)/flash/S25FL128

TESTS += Flash.test

check_PROGRAMS += Ring_Buffer
nodist_Ring_Buffer_SOURCES = $(gtest_srcdir)/Ring_Buffer.cpp

Ring_Buffer_LDADD = $(COMMON_LDADD)
Ring_Buffer_CPPFLAGS = $(AM_CPPFLAGS) -I$(core_srcdir)/ring_buffer
TESTS += Ring_Buffer.test

#check_PROGRAMS += Ble_nRF52
#nodist_Ble_nRF52_SOURCES = $(gtest_srcdir)/Ble_nRF52.cpp \
#                    $(gtest_srcdir)/mocks/cmock.c \
#                    $(gtest_srcdir)/mocks/Mocksyshal_spi.c \
#                    $(devices_srcdir)/ble/nRF52x/syshal_ble.c
#
#Ble_nRF52_LDADD = $(COMMON_LDADD)
#Ble_nRF52_CPPFLAGS = $(AM_CPPFLAGS) -I$(devices_srcdir)/ble/nRF52x
#TESTS += Ble_nRF52.test

check_PROGRAMS += Syshal_Timer
nodist_Syshal_Timer_SOURCES = $(gtest_srcdir)/Syshal_Timer.cpp \
                              $(gtest_srcdir)/mocks/Mocksyshal_rtc.c \
                              $(gtest_srcdir)/mocks/cmock.c \
                              $(gtest_srcdir)/mocks/unity.c \
                              $(syshal_srcdir)/src/syshal_timer.c

Syshal_Timer_LDADD = $(COMMON_LDADD)
Syshal_Timer_CPPFLAGS = $(AM_CPPFLAGS) -I$(core_srcdir)/debug
TESTS += Syshal_Timer.test

check_PROGRAMS += Sm_Main
nodist_Sm_Main_SOURCES = $(gtest_srcdir)/Sm_Main.cpp \
                         $(gtest_srcdir)/mocks/cmock.c \
                         $(gtest_srcdir)/mocks/unity.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_axl.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_batt.c \
                         $(gtest_srcdir)/mocks/Mockconfig_if.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_gpio.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_pressure.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_pmu.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_ble.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_switch.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_gps.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_flash.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_time.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_uart.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_spi.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_i2c.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_rtc.c \
                         $(gtest_srcdir)/mocks/Mocksyshal_firmware.c \
                         $(syshal_srcdir)/src/syshal_timer.c \
                         $(core_srcdir)/cexception/cexception.c \
                         $(core_srcdir)/sys_config/sys_config.c \
                         $(core_srcdir)/crc32/crc32.c \
                         $(core_srcdir)/fs/fs.c \
                         $(core_srcdir)/sm/sm.c \
                         $(core_srcdir)/buffer/buffer.c \
                         $(core_srcdir)/sm_main/sm_main.c

Sm_Main_LDADD = $(COMMON_LDADD)
Sm_Main_CPPFLAGS = $(AM_CPPFLAGS) \
                         -I$(CMSIS_srcdir)/Device/ST/STM32F0xx/Include \
                         -I$(CMSIS_srcdir)/Include \
                         -I$(boards_srcdir) \
                         -I$(hal_srcdir)/Inc \
                         -I$(ports_srcdir) \
                         -I$(core_srcdir)/crc32 \
                         -I$(core_srcdir)/config \
                         -I$(core_srcdir)/config_if \
                         -I$(core_srcdir)/buffer \
                         -I$(core_srcdir)/fs \
                         -I$(core_srcdir)/logging \
                         -I$(core_srcdir)/sys_config \
                         -I$(core_srcdir)/cexception \
                         -I$(core_srcdir)/debug \
                         -I$(core_srcdir)/sm_main \
                         -I$(core_srcdir)/sm
TESTS += Sm_Main.test

LINT_FLAGS  = -"A(C++2010)"
LINT_FLAGS += -e537 # Ignore repeated includes
LINT_FLAGS += -e160 # Allow GNU statement expression extension
LINT_FLAGS += -"efile(451,/usr/include/*)"
LINT_FLAGS += -"efile(451,stdarg.h)"
LINT_FLAGS += -"efile(451,stdarg.h)"
LINT_FLAGS += -"e747" # Significant prototype coercion - temporary?
LINT_FLAGS += +ffn -"width(0)" -hf1 -"format=%f(%l): %t %n: %m" # PC-Lint compatible output
LINT_FLAGS += -i$(builddir)/..
LINT_FLAGS += -i$(top_srcdir)
LINT_FLAGS += -i$(top_srcdir)/lib
LINT_FLAGS += -i$(top_srcdir)/tests/mocks
LINT_FLAGS += co-gcc.lnt
LINT_FLAGS += -w3
LINT_FLAGS += -DGTEST_USE_OWN_TR1_TUPLE=1

LOBS =

flint: $(LOBS)

clean-local:
	rm -rf gtest
	rm -f *.lob mocks/Mock*.{c,h}
	rm -f *.log *.xml *.gcda *.gcno *.out
endif
