# 定义模块的目标文件名
MODULE_NAME := adc_dma

# 定义模块的源文件
SRCS := $(MODULE_NAME).c

# 定义模块的目标文件
OBJS := $(SRCS:.c=.o)

obj-m += $(OBJS)

# 定义编译目标
all: modules | kbuild_def_check

kbuild_def_check:
ifndef KBUILD_DIR
	@printf "Error: The variable 'KBUILD_DIR' must be specified when "
	@printf "cross-compiling the driver.\n"
	@exit 1
endif

# 编译模块
modules:
	$(MAKE) -C $(KBUILD_DIR) M=$(PWD) modules

# 清理编译生成的文件
clean:
	$(MAKE) -C $(KBUILD_DIR) M=$(PWD) clean
