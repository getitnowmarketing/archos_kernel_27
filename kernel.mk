KERNEL_BUILD_DIR=$(BASE_DIR)/
KERNEL_CONFIG=$(KERNEL_BUILD_DIR)/linux/.config
KERNEL_CONFIG_SRC=$(KERNEL_SRC_REAL_DIR)/linux.config

KERNEL=$(KERNEL_BUILD_DIR)/linux/arch/arm/boot/zImage

KERNEL_SRC_REAL_DIR:=$(BASE_DIR)/../linux
KERNEL_SRC_DIR:=$(KERNEL_SRC_REAL_DIR)

ifeq ($(KERNEL_FLAVOUR),init)
USE_TINY=n
else
ifeq ($(KERNEL_FLAVOUR),recovery)
USE_TINY=n
else
USE_TINY=n
endif
endif

## copy kernel source only if for patching it!!!

$(KERNEL_SRC_DIR):
ifneq ($(KERNEL_SRC_DIR),$(KERNEL_SRC_REAL_DIR))
	mkdir -p $(KERNEL_SRC_DIR)
	tar -C $(KERNEL_SRC_REAL_DIR) --exclude=.svn -cpf - . | tar -C $(KERNEL_SRC_DIR) -xf -
else
	@echo "Use unpatched kernel"
endif

$(KERNEL_BUILD_DIR)/linux: $(KERNEL_SRC_DIR)
	@mkdir -p $(KERNEL_BUILD_DIR)/linux

$(KERNEL_CONFIG): $(KERNEL_CONFIG_SRC)
	cp $(KERNEL_CONFIG_SRC) $(KERNEL_CONFIG)
	@($(MAKE) -C $(KERNEL_SRC_DIR) oldconfig O=$(KERNEL_BUILD_DIR)/linux ARCH=arm CROSS_COMPILE=arm-linux-)

$(KERNEL): $(KERNEL_BUILD_DIR)/linux $(KERNEL_CONFIG) FORCE
	($(MAKE) $(MAKE_J) -C $(KERNEL_SRC_DIR) O=$(KERNEL_BUILD_DIR)/linux ARCH=arm CROSS_COMPILER=arm-linux-) 
	
kernel-clean:
	@($(MAKE) -C $(KERNEL_SRC_DIR) O=$(KERNEL_BUILD_DIR)/linux ARCH=arm CROSS_COMPILE=arm-linux- clean)

kernel-cleaner: kernel-clean
	rm -rf $(KERNEL_BUILD_DIR)/linux

kernel: $(KERNEL)

.PHONY: kernel-clean kernel-cleaner FORCE

TARGETS	+= kernel

