KERNEL_DIR=/home/nanming/work/kernel_itop
PWD=$(shell pwd)

all:
	make -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	make -C $(KERNEL_DIR) M=$(PWD) modules clean
	rm -rf modules.order

obj-m	+=	mylcd.o
