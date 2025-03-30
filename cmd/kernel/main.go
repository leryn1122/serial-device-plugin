package main

import (
	"fmt"
	kernel "github.com/leryn1122/serial-device-plugin/v2/kernel/pkg/cbinding"
	"github.com/leryn1122/serial-device-plugin/v2/lib/ioctl"
	"github.com/urfave/cli"
	"os"
)

const (
	VirtSerialControlDevice = "/dev/vrtsctl"
)

func main() {
	app := &cli.App{
		Name:  "kernel-ioctl",
		Usage: "Ioctl tools for kobject",
		Commands: []cli.Command{
			{
				Name:   "add",
				Usage:  "Add virtual serial",
				Action: AddVirtualSerial,
				Flags: []cli.Flag{
					cli.StringFlag{
						Name:  "devname",
						Usage: "Device name",
					},
					cli.UintFlag{
						Name:  "baud",
						Usage: "Baud rate",
						Value: 115200,
					},
				},
			},
			{
				Name:   "remove",
				Usage:  "Remove virtual serial",
				Action: RemoveVirtualSerial,
			},
			{
				Name:   "example",
				Usage:  "Create example virtual serial",
				Action: CreateExampleVirtualSerial,
			},
		},
	}

	if err := app.Run(os.Args); err != nil {
		fmt.Println(err)
	}
}

func AddVirtualSerial(ctx *cli.Context) error {
	fd, err := ioctl.Open(VirtSerialControlDevice, ioctl.O_RDWR, 0)
	defer ioctl.Close(fd)
	if err != nil {
		return err
	}

	config := kernel.NewVirtSerialConfigBuilder().
		SetDeviceName(ctx.String("devname")).
		SetBaudRate(ctx.Uint("baud")).
		Build()
	defer config.Drop()
	err = ioctl.Ioctl(fd, kernel.VirtSerialIoctlCreateDevice, uintptr(config.Raw()))
	if err != nil {
		return err
	}
	return nil
}

func RemoveVirtualSerial(ctx *cli.Context) error {
	return nil
}

func CreateExampleVirtualSerial(ctx *cli.Context) error {
	fd, err := ioctl.Open(VirtSerialControlDevice, ioctl.O_RDWR, 0)
	defer ioctl.Close(fd)
	if err != nil {
		return err
	}

	config := kernel.NewVirtSerialConfigBuilder().
		SetDeviceName("/dev/ttyVCOM0").
		SetBaudRate(115200).
		Build()
	defer config.Drop()
	err = ioctl.Ioctl(fd, kernel.VirtSerialIoctlCreateDevice, uintptr(config.Raw()))
	if err != nil {
		return err
	}
	return nil
}
