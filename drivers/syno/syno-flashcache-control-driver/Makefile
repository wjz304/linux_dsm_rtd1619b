COMMIT_REV ?= $(shell git describe  --always --abbrev=12)
KERNEL_SOURCE_VERSION ?= $(shell uname -r)
KERNEL_TREE ?= /lib/modules/$(KERNEL_SOURCE_VERSION)/build

EXTRA_CFLAGS += -I$(KERNEL_TREE)/drivers/md -I./ -DCOMMIT_REV="\"$(COMMIT_REV)\""
EXTRA_CFLAGS += -I$(KERNEL_TREE)/include/ -I$(KERNEL_TREE)/include/linux

obj-m += $(SYNO_CONTROL_DRIVER_NAME).o

.PHONY: all
all: modules

.PHONY:    modules
modules:
	$(MAKE) -C $(KERNEL_TREE) M=$(PWD) modules

.PHONY: modules_install
modules_install: modules
	install -o root -g root -m 0755 -d $(DESTDIR)/lib/modules/$(KERNEL_SOURCE_VERSION)/extra/flashcache/
	install -o root -g root -m 0755 flashcache.ko $(DESTDIR)/lib/modules/$(KERNEL_SOURCE_VERSION)/extra/flashcache/
	depmod -a

.PHONY: install
install: modules_install

.PHONY: clean
clean:
	$(MAKE) -C $(KERNEL_TREE) M=$(PWD) clean
