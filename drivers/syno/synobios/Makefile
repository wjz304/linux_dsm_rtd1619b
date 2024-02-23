include Makefile.inc

all:
	$(MAKE) ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} -C $(KSRC) M=$(shell pwd)/$(SUBDIR) EXTRA_CFLAGS="-DSYNO_PLATFORM=\\\"${SYNO_PLATFORM}\\\" -I$(shell pwd)/include" modules
	find ./ -name *synobios.ko -exec /usr/syno/bin/sign {} \;

distclean: 
	find . -type f \( -name "*.o" -o -name ".*.o.cmd" -o -name ".*.o.flags" \) -delete
