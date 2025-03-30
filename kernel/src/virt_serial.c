#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/version.h>

#ifdef _LINUX_SERIAL_REG_H
#include <linux/serial_reg.h>
#else
#include <uapi/linux/serial_reg.h>
#endif // _LINUX_SERIAL_REG_H

#include "virt_serial.h"

#define UART_NAME_SIZE 16
#define DEBUG_MODE 1s

// kfifo
#define FIFO_SIZE 4096
DEFINE_KFIFO(rx_fifo, char, FIFO_SIZE);
DEFINE_KFIFO(tx_fifo, char, FIFO_SIZE);

#define IOCTL_FETCH_ARGS(parameter, retval) \
    if (!access_ok((void __user*) parameter, sizeof(retval))) \
    { \
        return -EFAULT; \
    } \
    \
    if (copy_from_user(&retval, (void __user*) parameter, sizeof(retval))) \
    { \
        return -EFAULT; \
    }

static struct uart_driver virt_serial_driver;
static struct file_operations virt_serial_ctrl_fops;
static dev_t virt_serial_ctrl_dev;
static struct cdev virt_serial_ctrl_cdev;
static struct class *virt_serial_ctrl_class;

static LIST_HEAD(virt_serial_port_list);
static DEFINE_MUTEX(virt_serial_lock);

static struct uart_driver virt_serial_driver = {
    .owner          = THIS_MODULE,
    .driver_name    = VIRT_SERIAL_DRIVER_NAME,
    .dev_name       = VIRT_SERIAL_DEVICE_PREFIX,
    .major          = VIRT_SERIAL_MAJOR,
    .minor          = VIRT_SERIAL_MINOR,
    .nr             = VIRT_SERIAL_NR,
    .cons           = NULL,
};

/**
 * https://www.kernel.org/doc/html/latest/driver-api/serial/driver.html#c.uart_ops
 */
static const struct uart_ops virt_serial_ops = {
    .tx_empty       = virt_serial_tx_empty        ,
    .set_mctrl      = virt_serial_set_mctrl       ,
    .get_mctrl      = virt_serial_get_mctrl       ,
    .stop_tx        = virt_serial_stop_tx         ,
    .start_tx       = virt_serial_start_tx        ,
    .send_xchar     = virt_serial_send_xchar      ,
    .stop_rx        = virt_serial_stop_rx         ,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
    .start_rx       = virt_serial_start_rx        ,
#endif
    .enable_ms      = virt_serial_enable_ms       ,
    .break_ctl      = virt_serial_break_ctl       ,
    .startup        = virt_serial_startup         ,
    .shutdown       = virt_serial_shutdown        ,
    .set_termios    = virt_serial_set_termios     ,
    .pm             = virt_serial_pm              ,
    .type           = virt_serial_type            ,
    .release_port   = virt_serial_release_port    ,
    .request_port   = virt_serial_request_port    ,
    .config_port    = virt_serial_config_port     ,
    .verify_port    = virt_serial_verify_port     ,
    .ioctl          = virt_serial_uart_ioctl      ,
#ifdef CONFIG_CONSOLE_POLL
    .poll_init      = virt_serial_poll_init       ,
    .poll_put_char  = virt_serial_poll_put_char   ,
    .poll_get_char  = virt_serial_poll_get_char   ,
#endif
};

/*
static struct platform_driver virt_serial_platform_driver = {
    .driver = {
        .name = VIRT_SERIAL_DRIVER_NAME,
        .owner = THIS_MODULE,
        // .of_match_table = VIRT_SERIAL_OF_MATCH,
    },
};
*/

static unsigned int virt_serial_tx_empty(struct uart_port *port)
{
//    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
//    struct circ_buf *xmit = &serial_port->port.state->xmit;
//    return uart_circ_empty(xmit);
    return TIOCSER_TEMT;
}

static void virt_serial_set_mctrl(struct uart_port *uart_port, unsigned int mctrl)
{
    // Intentionally left blank with empty implementation.
}

static unsigned int virt_serial_get_mctrl(struct uart_port *uart_port)
{
    // Intentionally left blank with empty implementation.
    return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void virt_serial_stop_tx(struct uart_port *port)
{
    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
    unsigned long flags = 0;

    spin_lock_irqsave(&serial_port->write_lock, flags);

    serial_port->rx_enable_flag = false;

    spin_unlock_irqrestore(&serial_port->write_lock, flags);
}

static void virt_serial_start_tx(struct uart_port *port)
{
    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
    unsigned long flags = 0;

    spin_lock_irqsave(&serial_port->write_lock, flags);

    serial_port->tx_enable_flag = true;

    spin_unlock_irqrestore(&serial_port->write_lock, flags);
}

static void virt_serial_send_xchar(struct uart_port *port, char ch)
{
    // TODO
}

static void virt_serial_stop_rx(struct uart_port *port)
{
    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
    unsigned long flags = 0;

    spin_lock_irqsave(&serial_port->write_lock, flags);

    serial_port->rx_enable_flag = false;

    spin_unlock_irqrestore(&serial_port->write_lock, flags);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
static void virt_serial_start_rx(struct uart_port *port)
{
    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
    unsigned long flags = 0;

    spin_lock_irqsave(&serial_port->write_lock, flags);

    serial_port->rx_enable_flag = true;

    spin_unlock_irqrestore(&serial_port->write_lock, flags);
}
#endif

static void virt_serial_enable_ms(struct uart_port *port)
{
    // Intentionally left blank with empty implementation.
}

static void virt_serial_break_ctl(struct uart_port *port, int break_state)
{
    // Intentionally left blank with empty implementation.
}

static int virt_serial_startup(struct uart_port *port)
{
    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
    unsigned long flags = 0;

    int ret = 0;

    spin_lock_irqsave(&serial_port->write_lock, flags);

    serial_port->rx_enable_flag = true;
    serial_port->tx_enable_flag = true;
    if (kfifo_alloc(&serial_port->rx_fifo, FIFO_SIZE, GFP_KERNEL))
    {
        ret = -ENOMEM;
        goto err_exit;
    }
    if (kfifo_alloc(&serial_port->tx_fifo, FIFO_SIZE, GFP_KERNEL)) {
        kfifo_free(&serial_port->rx_fifo);
        ret = -ENOMEM;
        goto err_exit;
    }

    spin_unlock_irqrestore(&serial_port->write_lock, flags);

    return 0;

err_exit:
    spin_unlock_irqrestore(&serial_port->write_lock, flags);
    return -EIO;
}

static void virt_serial_shutdown(struct uart_port *port)
{
    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
    unsigned long flags = 0;

    spin_lock_irqsave(&serial_port->write_lock, flags);

    serial_port->rx_enable_flag = false;
    serial_port->tx_enable_flag = false;
    kfifo_free(&serial_port->rx_fifo);
    kfifo_free(&serial_port->tx_fifo);

    spin_unlock_irqrestore(&serial_port->write_lock, flags);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
static void virt_serial_set_termios(struct uart_port *port, struct ktermios *termios, const struct ktermios *old)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
static void virt_serial_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
#endif
{
    unsigned char cval = 0;
    unsigned int baud = 0;

    switch (termios->c_cflag & CSIZE)
    {
        case CS5:
            cval = UART_LCR_WLEN5;
            break;
        case CS6:
            cval = UART_LCR_WLEN6;
            break;
        case CS7:
            cval = UART_LCR_WLEN7;
            break;
        default:
        case CS8:
            cval = UART_LCR_WLEN8;
            break;
    }

    if (termios->c_cflag & CSTOPB)
    {
        cval |= UART_LCR_STOP;
    }
    if (termios->c_cflag & PARENB)
    {
        cval |= UART_LCR_PARITY;
    }
    if (!(termios->c_cflag & PARODD))
    {
        cval |= UART_LCR_EPAR;
    }

    baud = uart_get_baud_rate(port, termios, old, VIRT_SERIAL_MIN_SPEED, VIRT_SERIAL_MAX_SPEED);
    printk("baud = %d cval = %d\n", baud, cval);
}

static void virt_serial_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
#ifdef CONFIG_PM
    // Intentionally left blank with empty implementation.
#endif
}

static const char* virt_serial_type(struct uart_port *port)
{
    return NULL;
}

static void virt_serial_release_port(struct uart_port *port)
{
    // TODO
}

static int virt_serial_request_port(struct uart_port *port)
{
    if (0)
    {
        return -EBUSY;
    }
    return 0;
}

static void virt_serial_config_port(struct uart_port*, int type)
{
}

static int virt_serial_verify_port(struct uart_port *port, struct serial_struct *serial)
{
    // TODO
    return 0;
}

static int virt_serial_uart_ioctl(struct uart_port *port, unsigned int request, unsigned long args)
{
    switch (request)
    {
        case VIRT_SERIAL_IOCTL_SET_BAUDRATE:
            // uart_set_baud_rate(port, args);
            break;
        default:
            return -ENOTTY;
    }
    return 0;
}

#ifdef CONFIG_CONSOLE_POLL
static int virt_serial_poll_init(struct uart_port *port)
{
    // TODO
    return 0;
}

static void virt_serial_poll_put_char(struct uart_port *port, unsigned char ch)
{
    // TODO
}

static int virt_serial_poll_get_char(struct uart_port *port)
{
    // TODO
    return 0;
}
#endif

static int create_virt_serial_handle(char devname[16] , unsigned int baud)
{
    int ret = 0;
    struct virt_serial_port *port;
    port = kzalloc(sizeof(*port), GFP_KERNEL);
    if (!port)
    {
        ret = -ENOMEM;
        goto err_exit;
    }
    strscpy(port->devname, devname, UART_NAME_SIZE);
    port->tx_enable_flag = false;
    port->rx_enable_flag = false;

    struct uart_port *u_port;
    u_port = kzalloc(sizeof(*u_port), GFP_KERNEL);
    if (!u_port) {
        ret = -ENOMEM;
        goto err_malloc_uart_port;
    }
    u_port->line = 0;
    u_port->type = PORT_UNKNOWN;
    u_port->ops = &virt_serial_ops;
    u_port->flags = UPF_BOOT_AUTOCONF;
    u_port->dev = NULL;
    port->port = *u_port;

    printk(KERN_INFO "Start uart_add_one_port");
    if (uart_add_one_port(&virt_serial_driver, &port->port) < 0)
    {
        ret = -EIO;
        goto err_add_uart_port;
    }

    mutex_lock(&virt_serial_lock);
    printk(KERN_INFO "Start list_add_tail");
    list_add_tail(&port->list, &virt_serial_port_list);
    mutex_unlock(&virt_serial_lock);

    return 0;

err_add_uart_port:
    kfree(u_port);
err_malloc_uart_port:
    kfree(port);
err_exit:
    return ret;
}

static int remove_virt_serial_handle(char devname[16])
{
    return -ENODEV;
}

static long virt_serial_ioctl(struct file *file, unsigned int cmd, unsigned long args)
{
    if (_IOC_TYPE(cmd) != VIRT_SERIAL_IOCTL_MAGIC)
    {
        return -ENOTTY;
    }

    switch (cmd)
    {
        case VIRT_SERIAL_IOCTL_CREATE_DEVICE:
            struct virt_serial_config config;
            IOCTL_FETCH_ARGS(args, config);

            return create_virt_serial_handle(config.devname, config.baud);
        case VIRT_SERIAL_IOCTL_REMOVE_DEVICE:
            char devname[16];
            IOCTL_FETCH_ARGS(args, devname);

            return remove_virt_serial_handle(devname);
        case VIRT_SERIAL_IOCTL_PRESERVE:
#ifdef DEBUG_MODE
            return create_virt_serial_handle("VCOM0", 115200);
#endif // DEBUG_MODE
        default:
            return -ENOTTY;
    }
}

static struct file_operations virt_serial_ctrl_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = virt_serial_ioctl,
};

static int __init virt_serial_init(void)
{
    int ret;

    printk(KERN_INFO "VirtSerial driver initialized named virt_serial\n");
    ret = uart_register_driver(&virt_serial_driver);

    if (ret < 0)
    {
        goto err_free_driver;
    }

    ret = alloc_chrdev_region(&virt_serial_ctrl_dev, 0, 1, VIRT_SERIAL_CONTROL_DEVICE);
    if (ret < 0)
    {
        goto err_alloc_chrdev;
    }
    cdev_init(&virt_serial_ctrl_cdev, &virt_serial_ctrl_fops);
    virt_serial_ctrl_cdev.owner = THIS_MODULE;

    ret = cdev_add(&virt_serial_ctrl_cdev, virt_serial_ctrl_dev, 1);
    if (ret < 0)
    {
        goto err_release_cdev;
    }

    virt_serial_ctrl_class = class_create(THIS_MODULE, VIRT_SERIAL_CONTROL_DEVICE);
    device_create(virt_serial_ctrl_class, NULL, virt_serial_ctrl_dev, NULL, VIRT_SERIAL_CONTROL_DEVICE);
    return 0;

err_release_cdev:
    unregister_chrdev_region(virt_serial_ctrl_dev, 1);
err_alloc_chrdev:
    uart_unregister_driver(&virt_serial_driver);
err_free_driver:
    kfree(&virt_serial_driver);
    return ret;
}

static void __exit virt_serial_exit(void)
{
    mutex_lock(&virt_serial_lock);
    struct virt_serial_port *port, *tmp;
    list_for_each_entry_safe(port, tmp, &virt_serial_port_list, list) {
       uart_remove_one_port(&virt_serial_driver, &port->port);
       list_del(&port->list);
       kfree(port);
    }
    mutex_unlock(&virt_serial_lock);

    kfree(&virt_serial_lock);
    kfree(&virt_serial_port_list);
    device_destroy(virt_serial_ctrl_class, virt_serial_ctrl_dev);
    class_destroy(virt_serial_ctrl_class);
    cdev_del(&virt_serial_ctrl_cdev);
    unregister_chrdev_region(virt_serial_ctrl_dev, 1);
    uart_unregister_driver(&virt_serial_driver);
    kfree(&virt_serial_driver);
    printk(KERN_INFO "VirtSerial driver unloaded\n");
}

module_init(virt_serial_init);
module_exit(virt_serial_exit);

MODULE_LICENSE(DRIVER_LICENSE);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
