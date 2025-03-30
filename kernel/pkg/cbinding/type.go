//go:build linux

package cbinding

/*
#include <stdlib.h>

struct virt_serial_config {
    char devname[16];
    unsigned int baud;
} virt_serial_config;
*/
import "C"
import (
	"unsafe"
)

type VirtSerialConfig struct {
	// TODO

	//devname [16]byte
	//baud    uint32
}

func (cfg *VirtSerialConfig) Raw() unsafe.Pointer {
	return unsafe.Pointer(&cfg)
}

func (cfg *VirtSerialConfig) Drop() {
	//C.free(cfg.devname)
}

type DeviceName struct {
	// TODO
}

func NewDeviceName(name string) DeviceName {
	return DeviceName{}
}

func (d *DeviceName) Raw() unsafe.Pointer {
	return unsafe.Pointer(&d)
}

func (d *DeviceName) Drop() {
}
