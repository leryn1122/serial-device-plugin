#pragma once

#ifndef _VIRT_SERIAL_H
#define _VIRT_SERIAL_H

#include <linux/kernel.h>

// module info
#define DRIVER_LICENSE     "GPL"
#define DRIVER_VERSION     "v0.0.1"
#define DRIVER_AUTHOR      "Leryn <leryn1122@github.com>"
#define DRIVER_DESCRIPTION "Virtual Serial Port Driver"

// uart driver
#define VIRT_SERIAL_DRIVER_NAME   "virt-serial"
#define VIRT_SERIAL_DEVICE_PREFIX "ttyVCOM"
#define VIRT_SERIAL_MAJOR 0
#define VIRT_SERIAL_MINOR 0
#define VIRT_SERIAL_NR    4
#define VIRT_SERIAL_MIN_SPEED 57600
#define VIRT_SERIAL_MAX_SPEED 115200

#define VIRT_SERIAL_CONTROL_DEVICE  "vrtsctl"

// ioctl
#define VIRT_SERIAL_IOCTL_MAGIC 0xB8
// ioctl - vrtsctl
#define VIRT_SERIAL_IOCTL_PRESERVE      _IO(VIRT_SERIAL_IOCTL_MAGIC, 0)
#define VIRT_SERIAL_IOCTL_CREATE_DEVICE _IOW(VIRT_SERIAL_IOCTL_MAGIC, 1, struct virt_serial_config)
#define VIRT_SERIAL_IOCTL_REMOVE_DEVICE _IOW(VIRT_SERIAL_IOCTL_MAGIC, 2, char[16])
// ioctl - uart
#define VIRT_SERIAL_IOCTL_SET_BAUDRATE _IOW(VIRT_SERIAL_IOCTL_MAGIC, 3, unsigned int)

struct virt_serial_port
{
    char devname[16] ;
    struct uart_port port;
    bool tx_enable_flag;
    bool rx_enable_flag;
    struct kfifo rx_fifo;
    struct kfifo tx_fifo;
    spinlock_t write_lock;
    struct list_head list;
} virt_serial_port;

struct virt_serial_config
{
    char devname[16] ;
    unsigned int baud;
} virt_serial_config;

static unsigned int virt_serial_tx_empty(struct uart_port *port);
static void virt_serial_set_mctrl(struct uart_port *uart_port, unsigned int mctrl);
static unsigned int virt_serial_get_mctrl(struct uart_port *uart_port);
static void virt_serial_stop_tx(struct uart_port *port);
static void virt_serial_start_tx(struct uart_port *port);
static void virt_serial_send_xchar(struct uart_port *port, char ch);
static void virt_serial_stop_rx(struct uart_port *port);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
static void virt_serial_start_rx(struct uart_port *port);
#endif
static void virt_serial_enable_ms(struct uart_port *port);
static void virt_serial_break_ctl(struct uart_port *port, int break_state);
static int virt_serial_startup(struct uart_port *port);
static void virt_serial_shutdown(struct uart_port *port);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
static void virt_serial_set_termios(struct uart_port *port, struct ktermios *termios, const struct ktermios *old);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
static void virt_serial_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old);
#endif
static void virt_serial_pm(struct uart_port *port, unsigned int state, unsigned int oldstate);
static const char* virt_serial_type(struct uart_port *port);
static void virt_serial_release_port(struct uart_port *port);
static int virt_serial_request_port(struct uart_port *port);
static void virt_serial_config_port(struct uart_port*, int type);
static int virt_serial_verify_port(struct uart_port *port, struct serial_struct *serial);
static int virt_serial_uart_ioctl(struct uart_port *port, unsigned int request, unsigned long args);
#ifdef CONFIG_CONSOLE_POLL
static int virt_serial_poll_init(struct uart_port *port);
static void virt_serial_poll_put_char(struct uart_port *port, unsigned char ch);
static int virt_serial_poll_get_char(struct uart_port *port);
#endif

static long virt_serial_ioctl(struct file *file, unsigned int request, unsigned long args);

#endif // _VIRT_SERIAL_H
