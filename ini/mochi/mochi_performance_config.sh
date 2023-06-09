#!/bin/sh
export chip=mercury6p

#给出对应的命令
IrqCtlOn()
{
    # echo irqcrl irqsave_us_min_time irqsave_us_max_time irqrestore_us_time 1 >/sys/devices/virtual/mstar/msys/irq_control
    echo irqcrl 15000 50000 30000 1 > /sys/devices/virtual/mstar/msys/irq_control
}

IrqCtlOff()
{
    # echo irqcrl irqsave_us_min_time irqsave_us_max_time irqrestore_us_time 0 >/sys/devices/virtual/mstar/msys/irq_control
    echo irqcrl 15000 50000 30000 0 > /sys/devices/virtual/mstar/msys/irq_control
}

CpuOn()
{
    # echo name time percent 2 > /sys/devices/virtual/mstar/msys/cpu_loading
    echo test 10000000 95 2 > /sys/devices/virtual/mstar/msys/cpu_loading
}

CpuOff()
{
    echo test 10000000 95 0 > /sys/devices/virtual/mstar/msys/cpu_loading
}

BandwidthOn()
{
	echo MIU_PA_SET_MAX_SERVICE 262144 0xF0 0x10 0x00 > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_BIST256_OCCUPY 262144 > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0x0 0xff12 > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0x1 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0x2 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0x3 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0x4 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0x5 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0x6 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0x7 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0x8 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0x9 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0xA 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0xB 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0xC 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0xD 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0xE 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_RCMD_WIN 262144 0xF 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0x0 0xff12 > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0x1 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0x2 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0x3 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0x4 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0x5 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0x6 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0x7 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0x8 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0x9 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0xA 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0xB 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0xC 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0xD 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0xE 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
	echo MIU_PREARB_SET_WCMD_WIN 262144 0xF 0xffff > /sys/devices/virtual/mstar/msys/dmem_alloc;
}

BandwidthOff()
{
	echo MIU_BIST256_STOP 262144 > /sys/devices/virtual/mstar/msys/dmem_alloc;
}
