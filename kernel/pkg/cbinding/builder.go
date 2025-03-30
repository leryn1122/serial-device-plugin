package cbinding

// VirtSerialConfigBuilder Builder for VirtSerialConfig
type VirtSerialConfigBuilder struct {
	DeviceName string
	BaudRate   uint
}

func NewVirtSerialConfigBuilder() *VirtSerialConfigBuilder {
	return &VirtSerialConfigBuilder{}
}

func (builder *VirtSerialConfigBuilder) Build() *VirtSerialConfig {
	virtSerialConfig := &VirtSerialConfig{}
	return virtSerialConfig
}

func (builder *VirtSerialConfigBuilder) SetDeviceName(deviceName string) *VirtSerialConfigBuilder {
	builder.DeviceName = deviceName
	return builder
}

func (builder *VirtSerialConfigBuilder) SetBaudRate(baudRate uint) *VirtSerialConfigBuilder {
	builder.BaudRate = baudRate
	return builder
}
