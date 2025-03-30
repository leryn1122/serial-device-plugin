package cbinding

// VirtSerialConfigBuilder Builder for VirtSerialConfig
type VirtSerialConfigBuilder struct {
	deviceName string
	baudRate   uint
}

func NewVirtSerialConfigBuilder() *VirtSerialConfigBuilder {
	return &VirtSerialConfigBuilder{}
}

func (builder *VirtSerialConfigBuilder) Build() VirtSerialConfig {
	virtSerialConfig := VirtSerialConfig{}
	return virtSerialConfig
}

func (builder *VirtSerialConfigBuilder) SetDeviceName(deviceName string) *VirtSerialConfigBuilder {
	builder.deviceName = deviceName
	return builder
}

func (builder *VirtSerialConfigBuilder) SetBaudRate(baudRate uint) *VirtSerialConfigBuilder {
	builder.baudRate = baudRate
	return builder
}
