# Project
PROJECT := serial-device-plugin
NAME := virt-serial
VERSION := 1.0.0
BUILD_DATE := $(shell date +%Y%m%d)
GIT_VERSION := $(shell git describe --long --all 2>/dev/null)
SHA := $(shell git rev-parse --short=8 HEAD 2>/dev/null)

KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
WORK_DIR   := $(shell pwd)
SRC_DIR    := $(shell pwd)/src
BUILD_DIR  := $(WORK_DIR)/target

obj-m += virt_serial.o

##@ General
.PHONY: all clean install

help: ## Print help info
	@ awk 'BEGIN {FS = ":.*##"; printf "\nUsage:\n  make \033[36m<target>\033[0m\n"} /^[a-zA-Z_0-9-]+:.*?##/ { printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST)

clean: ## Clean build artifacts
	make -C $(KERNEL_DIR) M=$(BUILD_DIR) clean
	rm $(BUILD_DIR)/Makefile
	rm $(BUILD_DIR)/*.[ch]
	rmdir $(BUILD_DIR)

build: ## Build target artifacts
	mkdir -p $(BUILD_DIR)
	ln -sf $(WORK_DIR)/Makefile $(BUILD_DIR)/
	ln -sf $(SRC_DIR)/*.[ch]    $(BUILD_DIR)/
	make -C $(KERNEL_DIR) M=$(BUILD_DIR) modules

all: build ## build
