KDIR := /lib/modules/$(shell uname -r)/build
DRIVER := vni

obj-m += $(DRIVER).o
$(DRIVER)-y := vni_main.o vni_common.o vni_netlink.o vni_netdev.o vni_ethtool.o vni_log.o

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules -I$(PWD)

clean:
	rm -rf *.o *.ko *.mod.* *.cmd .module* modules* Module* .*.cmd .tmp*
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
