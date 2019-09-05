#!/bin/bash

# 
# Run this script to regenerate mock code found in the /mocks directory
# with the exception of cmock.c, cmock.h and unity.h which are
# invariant and taken from ThrowTheSwitch.org.
#
# NOTE: Do not modify the code that is auto-generated since any modifications
# will get overwritten!
#
# NOTE: Assumes you have ruby already installed
#
mocks="../../syshal/inc/syshal_axl.h \
       ../../syshal/inc/syshal_batt.h \
       ../../syshal/inc/syshal_ble.h \
       ../../syshal/inc/syshal_firmware.h \
       ../../syshal/inc/syshal_flash.h \
       ../../syshal/inc/syshal_gpio.h \
       ../../syshal/inc/syshal_gps.h \
       ../../syshal/inc/syshal_i2c.h \
       ../../syshal/inc/syshal_pmu.h \
       ../../syshal/inc/syshal_pressure.h \
       ../../syshal/inc/syshal_rtc.h \
       ../../syshal/inc/syshal_spi.h \
       ../../syshal/inc/syshal_switch.h \
       ../../syshal/inc/syshal_time.h \
       ../../syshal/inc/syshal_timer.h \
       ../../syshal/inc/syshal_uart.h \
       ../../syshal/inc/syshal_usb.h \
       ../../core/config_if/config_if.h \
       ../../core/sys_config/sys_config.h \
       ../../core/fs/fs.h \
      "
tar -zxvf CMock-master.tar.gz
ruby CMock-master/lib/cmock.rb -ocmock_options.yml $mocks
rm -rf CMock-master
