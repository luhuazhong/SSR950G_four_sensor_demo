#!/bin/sh

if [ ! -n "$1" ]
then
	echo "CRC:"
	cat /sys/module/mi_vif/parameters/g_bCrcEnable
	echo "Compress:"
	cat /sys/module/mi_vif/parameters/g_u16CompressMode
elif [ $1 = 0 ]
then
	echo "CRC: off"
	echo N,N,N,N,N,N,N,N,N,N,N,N,N,N,N,N > /sys/module/mi_vif/parameters/g_bCrcEnable
elif [ $1 = 1 ]
then
	echo "CRC: on"
	echo Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y > /sys/module/mi_vif/parameters/g_bCrcEnable
elif [ $1 = 2 ]
then
	echo "Compress: off"
	echo 65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535 > /sys/module/mi_vif/parameters/g_u16CompressMode
elif [ $1 = 3 ]
then
	echo "Compress: 5"
	echo 5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5 > /sys/module/mi_vif/parameters/g_u16CompressMode
elif [ $1 = 4 ]
then
	echo "Compress: 7"
	echo 7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7 > /sys/module/mi_vif/parameters/g_u16CompressMode
else
	echo "CRC:"
	cat /sys/module/mi_vif/parameters/g_bCrcEnable
	echo "Compress:"
	cat /sys/module/mi_vif/parameters/g_u16CompressMode
fi

