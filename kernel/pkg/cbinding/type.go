//go:build linux

package cbinding

/*
#include <stdlib.h>

struct virt_serial_config {
    const char* devname;
    unsigned int baud;
} virt_serial_config;
*/
import "C"
import (
	"unsafe"
)

type VirtSerialConfig struct {
	//devname [16]byte
	//baud    uint32
}

func (cfg VirtSerialConfig) Raw() unsafe.Pointer {
	return unsafe.Pointer(&cfg)
}

func (cfg VirtSerialConfig) Drop() {
	//C.free(cfg.devname)
}
