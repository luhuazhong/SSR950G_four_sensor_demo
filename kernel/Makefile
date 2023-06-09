KDIR?=$(PROJ_ROOT)/kbuild/$(KERNEL_VERSION)

SRC_FILE:= st_main_vif_isp_kernel.c
MODULE_DIR:= sys common vif isp scl
DEP_HAL_INC:= common
SIGMASTAR_INC_DIR:= $(foreach n,$(MODULE_DIR), $(PROJ_ROOT)/../sdk/interface/include/$(n)) \
        $(PROJ_ROOT)/../sdk/impl/$(MODULE_DIR) \
        $(PROJ_ROOT)/../sdk/interface/include \
        $(PROJ_ROOT)/../sdk/interface/include/internal/kernel \
        $(foreach n,$(DEP_HAL_INC),$(PROJ_ROOT)/../sdk/mhal/include/$(n))
APP_DIR:= $(OUT_PATH)/$(ARCH)/app
KONAME:= prog_vif_isp_kernel

MI_DBG       ?= 1
MAP_CONVER   := 1
VIF_REGISTER := 1
ISP_REGISTER := 1

EXTRA_CFLAGS+= -DMI_DBG=$(MI_DBG)
EXTRA_CFLAGS+= -DMAP_CONVER=$(MAP_CONVER)
EXTRA_CFLAGS+= -DVIF_REGISTER=$(VIF_REGISTER)
EXTRA_CFLAGS+= -DISP_REGISTER=$(ISP_REGISTER)

EXTRA_CFLAGS+= $(foreach n,$(SIGMASTAR_INC_DIR),-I$(n))
EXTRA_CFLAGS+= -I$(KDIR)/drivers/sstar/include/
ifeq ($(CHIP), i6)
EXTRA_CFLAGS+= -I$(KDIR)/drivers/sstar/include/infinity6
else ifeq ($(CHIP), i2m)
EXTRA_CFLAGS+= -I$(KDIR)/drivers/sstar/include/infinity2m
else ifeq ($(CHIP), i6e)
EXTRA_CFLAGS+= -I$(KDIR)/drivers/sstar/include/infinity6e
else ifeq ($(CHIP), i6b0)
EXTRA_CFLAGS+= -I$(KDIR)/drivers/sstar/include/infinity6b0
else ifeq ($(CHIP), m6)
EXTRA_CFLAGS+= -I$(KDIR)/drivers/sstar/include/mercury6
else ifeq ($(CHIP), i7)
EXTRA_CFLAGS+= -I$(KDIR)/drivers/sstar/include/infinity7
else ifeq ($(CHIP), m6p)
EXTRA_CFLAGS+= -I$(KDIR)/drivers/sstar/include/mercury6p
endif


obj-m       := $(KONAME).o
prog_vif_isp_kernel-objs+= $(patsubst %.c, %.o, $(SRC_FILE))

$(KONAME).ko:
	$(MAKE) -f Makefile -C $(KDIR) M=$(BUILD_TOP)/$(MODULE_PATH)/kernel modules

all: $(KONAME).ko
	@mkdir -p $(APP_DIR)
	@cp -vrf $(shell find $(MODULE_PATH)/kernel -name *.ko) $(APP_DIR)/

clean:
	$(MAKE) -f Makefile -C $(KDIR) M=$(BUILD_TOP)/$(MODULE_PATH)/kernel clean
	@rm -rf $(APP_DIR)/$(KONAME).ko
