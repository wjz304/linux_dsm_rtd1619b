COMMIT_REV ?= $(shell git describe  --always --abbrev=12)
KERNEL_SOURCE_VERSION ?= $(shell uname -r)
KERNEL_TREE ?= /lib/modules/$(KERNEL_SOURCE_VERSION)/build

EXTRA_CFLAGS += -I$(KERNEL_TREE)/drivers/md -I./ -DCOMMIT_REV="\"$(COMMIT_REV)\""
EXTRA_CFLAGS += -DTIME="\"$(TIME)\""
EXTRA_CFLAGS += -I$(KERNEL_TREE)/include/ -I$(KERNEL_TREE)/include/linux
# Stop at the first error
# FIXME: remove color cflags?
#EXTRA_CFLAGS += -Wfatal-errors -Wall -Werror
EXTRA_CFLAGS += -Wall -Werror -fdiagnostics-color -Wno-address-of-packed-member
KBUILD_EXTRA_SYMBOLS=$(PWD)/../syno-flashcache-control-driver/Module.symvers

ifeq ($(SYNO_FLASHCACHE), 1)
obj-m += $(SYNO_NEW_MODE_DRIVER_NAME).o
$(SYNO_NEW_MODE_DRIVER_NAME)-objs := flashcache_conf.o flashcache_main.o flashcache_subr.o flashcache_ioctl.o flashcache_procfs.o syno_functions.o syno_kernel_functions.o syno_quickflush.o syno_md_update.o
else
obj-m += flashcache.o
flashcache-objs := flashcache_conf.o flashcache_main.o flashcache_subr.o flashcache_ioctl.o flashcache_procfs.o
endif

.PHONY: all
ifeq ($(SYNO_FLASHCACHE), 1)
all: modules
else
all: modules utils

endif

.PHONY:    modules
modules: $(RHEL5_SETUP)
	$(MAKE) -C $(KERNEL_TREE) M=$(PWD) modules V=0

.PHONY:    utils
utils:
	$(MAKE) -C utils all

.PHONY: modules_install
modules_install: modules
	install -o root -g root -m 0755 -d $(DESTDIR)/lib/modules/$(KERNEL_SOURCE_VERSION)/extra/flashcache/
	install -o root -g root -m 0755 flashcache.ko $(DESTDIR)/lib/modules/$(KERNEL_SOURCE_VERSION)/extra/flashcache/
	depmod -a

.PHONY: utils_install
utils_install: utils
	$(MAKE) -C utils install

.PHONY: ocf_install
ocf_install:
	$(MAKE) -C ocf install

.PHONY: install
install: modules_install utils_install ocf_install

.PHONY: clean
clean:
	$(MAKE) -C $(KERNEL_TREE) M=$(PWD) clean

.PHONY: rhel5-setup
rhel5-setup: $(RHEL5_TREE)
	$(MAKE) -C $(RHEL5_TREE) oldconfig ; \
	$(MAKE) -C $(RHEL5_TREE) prepare modules_prepare
	ln -s -f $(RHEL5_SRC)/Module.symvers $(RHEL5_TREE)/Module.symvers

$(RHEL5_TREE):
	rpmbuild -bp --target=`uname -m` $(RHEL5_SPEC) 2>&1 | tee `dirname $(RHEL5_SPEC)`/prep.log
