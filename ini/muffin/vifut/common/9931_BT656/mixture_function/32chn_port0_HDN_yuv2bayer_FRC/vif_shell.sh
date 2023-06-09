#!/bin/bash

if ls -l prog_vif_isp_scl;then
    echo "ls prog_vpe return true"
	chmod 755 prog_vif_isp_scl
else
	echo "ls prog_vpe return fail"
fi

echo "param_1=$1"
if [ "$1" = "16" ]; then
	echo "s1=$1"
    ./prog_vif_isp_scl \
	param_snr0_4multi_dev0_720p30.ini param_snr0_4multi_dev1_720p30.ini param_snr0_4multi_dev2_720p30.ini param_snr0_4multi_dev3_720p30.ini \
    param_snr2_4multi_dev4_720p30.ini param_snr2_4multi_dev5_720p30.ini param_snr2_4multi_dev6_720p30.ini param_snr2_4multi_dev7_720p30.ini \
	param_snr1_4multi_dev8_720p30.ini param_snr1_4multi_dev9_720p30.ini param_snr1_4multi_dev10_720p30.ini param_snr1_4multi_dev11_720p30.ini \
	param_snr3_4multi_dev12_720p30.ini param_snr3_4multi_dev13_720p30.ini param_snr3_4multi_dev14_720p30.ini param_snr3_4multi_dev15_720p30.ini
elif [ "$1" = "32" ]; then
	echo "s1=$1"
	./prog_vif_isp_scl \
	param_snr0_4multi_dev0_720p30.ini param_snr0_4multi_dev1_720p30.ini param_snr0_4multi_dev2_720p30.ini param_snr0_4multi_dev3_720p30.ini \
    param_snr2_4multi_dev4_720p30.ini param_snr2_4multi_dev5_720p30.ini param_snr2_4multi_dev6_720p30.ini param_snr2_4multi_dev7_720p30.ini \
	param_snr1_4multi_dev8_720p30.ini param_snr1_4multi_dev9_720p30.ini param_snr1_4multi_dev10_720p30.ini param_snr1_4multi_dev11_720p30.ini \
	param_snr3_4multi_dev12_720p30.ini param_snr3_4multi_dev13_720p30.ini param_snr3_4multi_dev14_720p30.ini param_snr3_4multi_dev15_720p30.ini \
	param_snr4_4multi_dev16_720p30.ini param_snr4_4multi_dev17_720p30.ini param_snr4_4multi_dev18_720p30.ini param_snr4_4multi_dev19_720p30.ini \
	param_snr6_4multi_dev20_720p30.ini param_snr6_4multi_dev21_720p30.ini param_snr6_4multi_dev22_720p30.ini param_snr6_4multi_dev23_720p30.ini \
	param_snr5_4multi_dev24_720p30.ini param_snr5_4multi_dev25_720p30.ini param_snr5_4multi_dev26_720p30.ini param_snr5_4multi_dev27_720p30.ini \
	param_snr7_4multi_dev28_720p30.ini param_snr7_4multi_dev29_720p30.ini param_snr7_4multi_dev30_720p30.ini param_snr7_4multi_dev28_720p31.ini 
else
	echo "s1=$1"
	./prog_vif_isp_scl \
	param_snr0_4multi_dev0_720p30.ini
fi
