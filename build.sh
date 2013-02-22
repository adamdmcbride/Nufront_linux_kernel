#!/bin/bash

KERNEL_ROOT=$(cd $(dirname $0);pwd)      #kernel code的根目录
CONFIG_FILE=""                           #kernel配置文件
MAKE_TARGET=""                           #编译的平台
KERNEL_OUT=${KERNEL_ROOT}/../../out/

F_CLEAN=0
F_TARGET=0

function show_help() {
	echo "
usage :
	build.sh [-c] [-i] [-t <target>] [-h]
	-h:		show help.
	-c:		clear project.
	-i:		initialize code use repo/git tools.
	-t <target>:	target type:
			full_nusmart3_pad          --->nufront_config_pad_ref
			full_nusmart3_pad_nw51     --->nufront_config_pad_ref(nw51 is y)
			full_nusmart3_prototype    --->nufront_config_pad_prototype
			full_nusmart3_stick        --->nufront_config_hdmi_stick
"
}

function LOGE() {
	TARGET_LOG=${KERNEL_ROOT}/../../out/$MAKE_TARGET.log
	echo "[ERROR]: $1" |tee -a $TARGET_LOG
	exit 0;
}

function LOGI() {
	TARGET_LOG=${KERNEL_ROOT}/../../out/$MAKE_TARGET.log
	echo "[INFO]: $1" |tee -a $TARGET_LOG
}

function select_platform() {
	ARG=$1
	if [[ $1 = "full_nusmart3_pad" ]]; then
		MAKE_TARGET=$ARG
	CONFIG_FILE=nufront_config_pad_ref
	elif [[ $1 = "full_nusmart3_stick" ]]; then
		MAKE_TARGET=$ARG
		CONFIG_FILE=nufront_config_hdmi_stick
	elif [[ $1 = "full_nusmart3_pad_nw51" ]]; then
		MAKE_TARGET=$ARG
		CONFIG_FILE=nufront_config_pad_ref
	elif [[ $1 = "full_nusmart3_prototype" ]]; then
		MAKE_TARGET=$ARG
		CONFIG_FILE=nufront_config_pad_prototype
	else
		LOGE "compile kernel fail, not found '$1' configure."
	fi
}

function select_kernel_version() {
	echo
}

function INIT_CODE_CMD() {
	#初始化code.
	repo init -u ssh://zhengxing@192.168.2.17:29418/platform/manifest.git -b  linux_kernel.ns115_pm
	repo sync
	repo start --all linux_kernel.ns115_pm
}

function check_kernel_out_dir() {
	if [[ ! -d $1 ]]; then
		mkdir -p $1
	fi
}

function compile_kernel() {
	LOGI "======================================================="
	LOGI "compile kernel......"
	LOGI "======================================================="
	LOGI "target is $MAKE_TARGET ......"
	LOGI "config is $CONFIG_FILE ......"
	copy_kernel_config
	make_uImage
	KERNEL_OUT=${KERNEL_ROOT}/../../out/$MAKE_TARGET
	check_kernel_out_dir ${KERNEL_OUT}
	LOGI "cp ${KERNEL_ROOT}/arch/arm/boot/uImage ${KERNEL_OUT}"
	cp ${KERNEL_ROOT}/arch/arm/boot/uImage ${KERNEL_OUT}
	if [[ $? != 0 ]]; then
		LOGE "copy uImage fail..."
	fi

	check_kernel_out_dir ${KERNEL_OUT}/modules

	for ko in $(find ${KERNEL_ROOT} -name "*.ko")
	do
		LOGI "cp -f $ko ${KERNEL_OUT}/modules/"
		cp -f $ko ${KERNEL_OUT}/modules/
	done

	LOGI "compile kernel[uImage_recovery]....."
	copy_recovery_config
	make_uImage_recovery
	check_kernel_out_dir ${KERNEL_OUT}
	LOGI "cp ${KERNEL_ROOT}/arch/arm/boot/uImage_recovery ${KERNEL_OUT}"
	cp ${KERNEL_ROOT}/arch/arm/boot/uImage ${KERNEL_OUT}/uImage_recovery
	if [[ $? != 0 ]]; then
		LOGE "copy uImage_recovery fail..."
	fi
}

function make_main() {
	check_kernel_out_dir ${KERNEL_OUT}
	if [ $F_CLEAN -eq 1 -a $F_TARGET -ne 1 ]; then
		echo "make clean kernel......"
		P1=$(pwd)
		cd $KERNEL_ROOT
		make clean
		cd $P1
		rm -fr ${KERNEL_ROOT}/.config
	elif [ $F_TARGET -eq 1 -a $F_CLEAN -ne 1 ]; then
		compile_kernel
	elif [ $F_TARGET -eq 1 -a $F_CLEAN -eq 1 ]; then
		make_clean
	fi
}

function copy_kernel_config() {
	LOGI "cp ${KERNEL_ROOT}/arch/arm/configs/${CONFIG_FILE} ${KERNEL_ROOT}/.config"
	cp ${KERNEL_ROOT}/arch/arm/configs/${CONFIG_FILE} ${KERNEL_ROOT}/.config
	if [ $MAKE_TARGET = "full_nusmart3_pad_nw51" ];then
	sed -i 's/# CONFIG_WIFI_NW51 is not set/CONFIG_WIFI_NW51=y/g' ${KERNEL_ROOT}/.config
	fi
}

function copy_recovery_config() {
	tmp_name=${KERNEL_ROOT}/arch/arm/configs/${CONFIG_FILE}
	recovery_config=${tmp_name}"_recovery"
	cp ${recovery_config} ${KERNEL_ROOT}/.config
	LOGI "cp ${recovery_config} ${KERNEL_ROOT}/.config"
}

function make_uImage() {
	P1=`pwd`
	cd ${KERNEL_ROOT}
	make -j4 uImage
	make -j8 modules
	cd $P1
}

function make_uImage_recovery() {
	p1=`pwd`
	cd ${KERNEL_ROOT}
	make -j4 uImage
	cd $P1
}

function make_clean() {
	KERNEL_OUT=${KERNEL_ROOT}/../../out/$MAKE_TARGET
	if [[ ! -d $KERNEL_OUT ]]; then
		echo "$KERNEL_OUT is empty......"
	else
		echo "clean ${KERNEL_OUT}/uImage......"
		rm -fr ${KERNEL_OUT}/uImage

		echo "clean ${KERNEL_OUT}/uImage_recovery"
		rm -fr ${KERNEL_OUT}/uImage_recovery

		echo "clean ${KERNEL_OUT}/modules/*.ko......"
		rm -fr ${KERNEL_OUT}/modules/*
	fi
	P1=$(pwd)
	cd $KERNEL_ROOT
	echo "make clean kernel......"
	make clean
	cd $P1
	rm -fr ${KERNEL_ROOT}/.config
}

###################################################

if [[ $# == 0 ]]; then
	show_help;
	exit 0;
fi

while getopts kt:chi OPT;
do
	case $OPT in
	t)
		select_platform $OPTARG
		F_TARGET=1
		;;
	k)
		select_kernel_version  $OPTARG
		;;
	c)
		F_CLEAN=1
		;;
	h)
		show_help
		exit 0
		;;
	i)
		#INIT_CODE_CMD
		exit 0
		;;
	*)
		show_help
		exit 1
		;;
	esac
done

make_main
