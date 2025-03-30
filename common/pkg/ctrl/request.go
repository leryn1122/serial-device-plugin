package ctrl

import (
	"github.com/leryn1122/serial-device-plugin/v2/kernel/pkg/cbinding"
)

type CreateSerialPortRequest struct {
	DeviceName string
	BaudRate   uint
}

func (req *CreateSerialPortRequest) raw() cbinding.VirtSerialConfig {
	return cbinding.NewVirtSerialConfigBuilder().
		SetDeviceName(req.DeviceName).
		SetBaudRate(req.BaudRate).
		Build()
}

type RemoveSerialPortRequest struct {
	DeviceName string
}

func (req *RemoveSerialPortRequest) raw() cbinding.DeviceName {
	return cbinding.NewDeviceName(req.DeviceName)
}
