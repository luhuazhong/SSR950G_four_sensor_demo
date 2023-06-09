INC  += $(BUILD_TOP)/common/live555/UsageEnvironment/include
INC  += $(BUILD_TOP)/common/live555/groupsock/include
INC  += $(BUILD_TOP)/common/live555/liveMedia/include
INC  += $(BUILD_TOP)/common/live555/BasicUsageEnvironment/include
INC  += $(BUILD_TOP)/common/live555/mediaServer/include
INC  += $(BUILD_TOP)/common/iniparser
INC  += $(BUILD_TOP)/internal/md5
INC  += $(BUILD_TOP)/internal/vdisp
INTERFACE_EXIST = $(shell if [ -d $(BUILD_TOP)/../../interface ]; then echo "exist"; else echo "noexist"; fi)

ifneq ($(DUAL_OS), on)
ifeq ($(INTERFACE_EXIST), exist)
include $(MODULE_PATH)/kernel/Makefile
endif
endif

LIBS += -lmi_sys

PROG_USER_SENSOR   := 1
PROG_VIF_ENABLE    := 1
PROG_ISP_ENABLE    := 1
PROG_SCL_ENABLE    := 1
PROG_JPD_ENABLE    := 1
PROG_VENC_ENABLE   := 1
PROG_DISP_ENABLE   := 1
PROG_IPU_ENABLE   := 0

ifeq ($(INTERFACE_CUS3A), y)
PROG_ISPIQ_ENABLE  := 1
else
PROG_ISPIQ_ENABLE  := 0
endif

CODEDEFINE+= -DPROG_USER_SENSOR=$(PROG_USER_SENSOR)
CODEDEFINE+= -DPROG_VIF_ENABLE=$(PROG_VIF_ENABLE) -D_FILE_OFFSET_BITS=64
CODEDEFINE+= -DPROG_ISP_ENABLE=$(PROG_ISP_ENABLE)
CODEDEFINE+= -DPROG_ISPIQ_ENABLE=$(PROG_ISPIQ_ENABLE)
CODEDEFINE+= -DPROG_SCL_ENABLE=$(PROG_SCL_ENABLE)
CODEDEFINE+= -DPROG_JPD_ENABLE=$(PROG_JPD_ENABLE)
CODEDEFINE+= -DPROG_VENC_ENABLE=$(PROG_VENC_ENABLE)
CODEDEFINE+= -DPROG_DISP_ENABLE=$(PROG_DISP_ENABLE)
CODEDEFINE+= -DPROG_IPU_ENABLE=$(PROG_IPU_ENABLE)

ifeq ($(PROG_USER_SENSOR), 1)
INC += $(BUILD_TOP)/sensor/dh9931
ifeq ($(CHIP), i7)
LIBS += -L$(BUILD_TOP)/sensor/dh9931/i7
else ifeq ($(CHIP), m6p)
LIBS += -L$(BUILD_TOP)/sensor/dh9931/m6p
endif
LIBS += -ldh9931_sdk
endif

ifeq ($(PROG_VIF_ENABLE), 1)
LIBS += -lmi_sensor -lmi_vif
endif

#ifeq ($(PROG_ISP_ENABLE), 1)
LIBS += -lmi_isp
LIBS += -lfbc_decode
#endif

#ifeq ($(PROG_ISPIQ_ENABLE), 1)
INC  += ./internal/cus3a
LIBS += -lmi_iqserver -lcus3a -lispalgo -lmi_nir
#endif

ifeq ($(PROG_IPU_ENABLE), 1)
INC  += ./internal/ipu
LIBS += -lmi_ipu
endif

ifeq ($(PROG_SCL_ENABLE), 1)
LIBS += -lmi_scl
endif

ifeq ($(PROG_JPD_ENABLE), 1)
LIBS += -lmi_jpd
endif

ifeq ($(PROG_VENC_ENABLE), 1)
LIBS   += -lmi_venc
endif

ifeq ($(PROG_DISP_ENABLE), 1)
LIBS += -lmi_disp -lmi_panel -lmi_hdmi -lmi_vdisp
endif
