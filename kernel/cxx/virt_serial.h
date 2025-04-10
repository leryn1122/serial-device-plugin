#pragma once

#ifndef _VIRT_SERIAL_H
#define _VIRT_SERIAL_H

#include <linux/kernel.h>

// module info

#define DRIVER_LICENSE     "GPL"
#define DRIVER_VERSION     "v0.0.1"
#define DRIVER_AUTHOR      "Leryn <leryn1122@github.com>"
#define DRIVER_DESCRIPTION "Virtual Serial Port Driver"

// UART driver

#define VIRT_SERIAL_DRIVER_NAME   "virt-serial"
#define VIRT_SERIAL_DEVICE_PREFIX "ttyVCOM"
#define VIRT_SERIAL_MAJOR 0
#define VIRT_SERIAL_MINOR 0
#define VIRT_SERIAL_NR    4
#define VIRT_SERIAL_MIN_SPEED 9600
#define VIRT_SERIAL_MAX_SPEED 115200
#define VIRT_SERIAL_CONTROL_DEVICE  "vrtsctl"

// ioctl

#define VIRT_SERIAL_IOCTL_MAGIC 0xB8

// ioctl - /dev/vrtsctl

#define VIRT_SERIAL_IOCTL_PRESERVE      _IO(VIRT_SERIAL_IOCTL_MAGIC, 0)
#define VIRT_SERIAL_IOCTL_CREATE_DEVICE _IOW(VIRT_SERIAL_IOCTL_MAGIC, 1, struct virt_serial_config)
#define VIRT_SERIAL_IOCTL_REMOVE_DEVICE _IOW(VIRT_SERIAL_IOCTL_MAGIC, 2, devname_t)

// ioctl - UART

#define VIRT_SERIAL_IOCTL_SET_BAUDRATE _IOW(VIRT_SERIAL_IOCTL_MAGIC, 3, unsigned int)

// misc.

#define DEVICE_NAME_SIZE 16

typedef unsigned int baud_t;
typedef char devname_t[DEVICE_NAME_SIZE];

struct virt_serial_config
{
    char devname[DEVICE_NAME_SIZE] ;
    baud_t baud;
} virt_serial_config;

/**
 * Struct for virtual serial port.
 */
struct virt_serial_port
{
    char devname[DEVICE_NAME_SIZE] ;
    struct uart_port port;
    bool tx_enable_flag;
    bool rx_enable_flag;
    struct kfifo rx_fifo;
    struct kfifo tx_fifo;
    spinlock_t write_lock;
} virt_serial_port;

#endif // _VIRT_SERIAL_H
