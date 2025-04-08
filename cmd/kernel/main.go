package main

import (
	"fmt"
	"github.com/leryn1122/serial-device-plugin/v2/common/pkg/ctrl"
	"github.com/urfave/cli/v2"
	"os"
)

func main() {
	app := &cli.App{
		Name:  "ioctl-tool",
		Usage: "Ioctl tools for kobject",
		Commands: []*cli.Command{
			{
				Name:   "add",
				Usage:  "Add virtual serial",
				Action: AddVirtualSerial,
				Flags: []cli.Flag{
					&cli.StringFlag{
						Name:    "device",
						Aliases: []string{"devname"},
						Usage:   "Device name, e.g. ttyVCOM0",
					},
					&cli.UintFlag{
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
				Flags: []cli.Flag{
					&cli.StringFlag{
						Name:    "device",
						Aliases: []string{"devname"},
						Usage:   "Device name, e.g. ttyVCOM0",
					},
				},
			},
			{
				Name:   "sample",
				Usage:  "Create sample virtual serial",
				Action: CreateSampleSerial,
				Hidden: true,
			},
		},
	}

	if err := app.Run(os.Args); err != nil {
		fmt.Println(err)
	}
}

func AddVirtualSerial(ctx *cli.Context) error {
	controlDevice, err := ctrl.NewControlDevice()
	if err != nil {
		return err
	}
	defer controlDevice.Close()

	request := ctrl.CreateSerialPortRequest{
		DeviceName: ctx.String("device"),
		BaudRate:   ctx.Uint("baud"),
	}
	err = controlDevice.CreateSerialPort(request)
	if err != nil {
		return err
	}
	return nil
}

func RemoveVirtualSerial(ctx *cli.Context) error {
	controlDevice, err := ctrl.NewControlDevice()
	if err != nil {
		return err
	}
	defer controlDevice.Close()

	request := ctrl.RemoveSerialPortRequest{
		DeviceName: ctx.String("device"),
	}
	err = controlDevice.RemoveSerialPort(request)
	if err != nil {
		return err
	}
	return nil
}

func CreateSampleSerial(ctx *cli.Context) error {
	controlDevice, err := ctrl.NewControlDevice()
	if err != nil {
		return err
	}
	defer controlDevice.Close()
	err = controlDevice.CreateSampleSerialPort()
	if err != nil {
		return err
	}
	return nil
}
