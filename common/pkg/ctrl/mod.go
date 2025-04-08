package ctrl

import (
	"github.com/leryn1122/serial-device-plugin/v2/lib/ioctl"
	"github.com/pkg/errors"
)

const (
	ControllerDeviceFile = "/dev/vrtsctl"
)

type ControlDevice struct {
	fd int
}

func NewControlDevice() (*ControlDevice, error) {
	fd, err := ioctl.Open(ControllerDeviceFile, ioctl.O_RDWR, 0)
	if err != nil {
		return nil, errors.Wrapf(err, "failed to open controller device")
	}
	return &ControlDevice{
		fd: fd,
	}, nil
}

func (d *ControlDevice) Close() error {
	err := ioctl.Close(d.fd)
	if err != nil {
		return errors.Wrapf(err, "failed to close controller device")
	}
	return nil
}

func (d *ControlDevice) Name() string {
	return ControllerDeviceFile
}

func (d *ControlDevice) CreateSerialPort(request CreateSerialPortRequest) error {
	config := request.raw()
	defer config.Drop()
	err := ioctl.Ioctl(d.fd, VirtSerialIoctlCreateDevice, uintptr(config.Raw()))
	if err != nil {
		return errors.Wrapf(err, "failed to create the virtual serial port")
	}
	return nil
}

func (d *ControlDevice) RemoveSerialPort(request RemoveSerialPortRequest) error {
	deviceName := request.raw()
	defer deviceName.Drop()
	err := ioctl.Ioctl(d.fd, VirtSerialIoctlRemoveDevice, uintptr(deviceName.Raw()))
	if err != nil {
		return errors.Wrapf(err, "failed to remove the virtual serial port")
	}
	return nil
}

func (d *ControlDevice) CreateSampleSerialPort() error {
	err := ioctl.Ioctl(d.fd, VirtSerialIoctlPreserve)
	if err != nil {
		return errors.Wrapf(err, "failed to remove the virtual serial port")
	}
	return nil
}
