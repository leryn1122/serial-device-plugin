package cbinding

import "github.com/leryn1122/serial-device-plugin/v2/lib/ioctl"

type VirtSerialIoctlOps = uint64

const (
	virtSerialIoctlMagic = 0xB8
)

var (
	VirtSerialIoctlPreserve     VirtSerialIoctlOps = ioctl.IO(virtSerialIoctlMagic, 0)
	VirtSerialIoctlCreateDevice VirtSerialIoctlOps = ioctl.IOW(virtSerialIoctlMagic, 1, 20)
	VirtSerialIoctlRemoveDevice VirtSerialIoctlOps = ioctl.IOW(virtSerialIoctlMagic, 2, 16)
)
