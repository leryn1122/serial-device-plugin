#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/version.h>

#ifdef _LINUX_SERIAL_REG_H
#  include <linux/serial_reg.h>
#else
#  include <uapi/linux/serial_reg.h>
#endif // _LINUX_SERIAL_REG_H

#include "virt_serial.h"

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

// kfifo
#define FIFO_SIZE 4096

//============================================================================//
// virt_serial_uart.c
//============================================================================//

/**
 * UART driver for virtual serial device.
 * Core struct for driver with implementations.
 */
// NOLINTNEXTLINE(*-interfaces-global-init)
static struct uart_driver virt_serial_drv;
/**
 * UART ioctl operations for virtual serial device.
 */
static const struct uart_ops virt_serial_uart_ops;

// UART driver implementations

static unsigned int virt_serial_tx_empty(struct uart_port*);
static void virt_serial_set_mctrl(struct uart_port*, unsigned int mctrl);
static unsigned int virt_serial_get_mctrl(struct uart_port*);
static void virt_serial_stop_tx(struct uart_port*);
static void virt_serial_start_tx(struct uart_port*);
static void virt_serial_send_xchar(struct uart_port*, char ch);
static void virt_serial_stop_rx(struct uart_port*);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void virt_serial_start_rx(struct uart_port*);
#endif
static void virt_serial_enable_ms(struct uart_port*);
static void virt_serial_break_ctl(struct uart_port*, int break_state);
static int virt_serial_startup(struct uart_port*);
static void virt_serial_shutdown(struct uart_port*);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void virt_serial_set_termios(struct uart_port*, struct ktermios *termios, const struct ktermios *old);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
static void virt_serial_set_termios(struct uart_port*, struct ktermios *termios, struct ktermios *old);
#endif
static void virt_serial_pm(struct uart_port *port, unsigned int state, unsigned int oldstate);
static const char* virt_serial_type(struct uart_port*);
static void virt_serial_release_port(struct uart_port*);
static int virt_serial_request_port(struct uart_port*);
static void virt_serial_config_port(struct uart_port*, int type);
static int virt_serial_verify_port(struct uart_port*, struct serial_struct *serial);
static int virt_serial_uart_ioctl(struct uart_port*, unsigned int cmd, unsigned long args);
#ifdef CONFIG_CONSOLE_POLL
static int virt_serial_poll_init(struct uart_port*);
static void virt_serial_poll_put_char(struct uart_port*, unsigned char ch);
static int virt_serial_poll_get_char(struct uart_port*);
#endif // CONFIG_CONSOLE_POLL

//============================================================================//
// virt_serial_plt_drv.c
//============================================================================//

static int virt_serial_plt_drv_probe(struct platform_device*);

static int virt_serial_plt_drv_remove(struct platform_device*);

#define virt_serial_plt_drv_suspend  NULL
#define virt_serial_plt_drv_shutdown NULL
#define virt_serial_plt_drv_resume   NULL

/**
 * Platform driver for virtual serial driver.
 */
// NOLINTNEXTLINE(*-interfaces-global-init)
static struct platform_driver virt_serial_plt_drv = {
    .driver = {
        .name = VIRT_SERIAL_DRIVER_NAME,
        .owner = THIS_MODULE,
    },
    .probe      = virt_serial_plt_drv_probe,
    .remove     = virt_serial_plt_drv_remove,
    .resume     = virt_serial_plt_drv_resume,
    .shutdown   = virt_serial_plt_drv_shutdown,
    .suspend    = virt_serial_plt_drv_suspend,
};

// module_platform_driver(virt_serial_drv);

static int virt_serial_plt_drv_probe(struct platform_device *pdev)
{
    struct virt_serial_config *config = platform_get_drvdata(pdev);
    // Allocate and initialize virtual serial port.
    struct virt_serial_port *port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
    if (!port)
    {
        goto fail_alloc_port;
    }
    strscpy(port->devname, config->devname, DEVICE_NAME_SIZE);
    port->tx_enable_flag = false;
    port->rx_enable_flag = false;
    // DEFINE_KFIFO(port->rx_fifo, char, FIFO_SIZE);
    // DEFINE_KFIFO(port->tx_fifo, char, FIFO_SIZE);

    // Allocate and initialize UART port.
    struct uart_port *uart_port = kzalloc(sizeof(*uart_port), GFP_KERNEL);
    if (!uart_port) {
        goto fail_alloc_uart_port;
    }
    uart_port->line = 0;
    uart_port->type = PORT_UNKNOWN;
    uart_port->dev = &pdev->dev;
    uart_port->ops = &virt_serial_uart_ops;
    uart_port->flags = UPF_BOOT_AUTOCONF;

    port->port = *uart_port;
    platform_set_drvdata(pdev, port);

    int ret = 0;
    printk(KERN_DEBUG "uart_add_one_port");
    ret = uart_add_one_port(&virt_serial_drv, &port->port);
    if (ret < 0)
    {
        printk(KERN_DEBUG "uart_add_one_port: %d", ret);
        goto fail_add_uart_port;
    }
    return 0;

fail_add_uart_port:
    kfree(&uart_port);
fail_alloc_uart_port:
    kfree(&port);
fail_alloc_port:
    return ret;
}

static int virt_serial_plt_drv_remove(struct platform_device *pdev)
{
    // TODO
    return 0;
}

/**
* Remove virtual serial port handle by given device name.
 */
static int remove_virt_serial_handle(devname_t devname)
{
    // TODO
    return -ENODEV;
}

/**
 * Create virtual serial port handle with given device name and baud rate.
 */
static int create_virt_serial_handle(struct virt_serial_config *config)
{
    int ret = 0;

    // Allocate and initialize platform device.
    struct platform_device *pdev = platform_device_alloc(VIRT_SERIAL_DRIVER_NAME, PLATFORM_DEVID_AUTO);
    if (!pdev)
    {
        ret = -ENOMEM;
        goto fail_alloc_pdev;
    }

    platform_set_drvdata(pdev, config);

    ret = platform_device_add(pdev);
    if (ret < 0) {
        ret = -EIO;
        goto fail_add_platform_device;
    }
    return 0;

fail_add_platform_device:
    platform_device_put(pdev);
fail_alloc_pdev:
    return ret;
}

//============================================================================//
// virt_serial_ctrl.c
//============================================================================//

/**
 * Ioctl operations for virtual serial driver control device.
 */
// NOLINTNEXTLINE(*-interfaces-global-init)
static const struct file_operations virt_serial_ctrl_fops;

/**
 * Virtual serial driver control device called `/dev/vrtscrl`
 */
static dev_t virt_serial_ctrl_dev;
static struct cdev virt_serial_ctrl_cdev;
static struct class *virt_serial_ctrl_class;

static int create_virt_serial_ctrl_dev(void)
{
    int ret = 0;
    ret = alloc_chrdev_region(&virt_serial_ctrl_dev, 0, 1, VIRT_SERIAL_CONTROL_DEVICE);
    if (ret < 0)
    {
        ret = -EINVAL;
        goto fail_alloc_dev;
    }
    printk(KERN_DEBUG "major = %d, minor = %d\n", MAJOR(virt_serial_ctrl_dev), MINOR(virt_serial_ctrl_dev));

    cdev_init(&virt_serial_ctrl_cdev, &virt_serial_ctrl_fops);
    virt_serial_ctrl_cdev.owner = THIS_MODULE;

    ret = cdev_add(&virt_serial_ctrl_cdev, virt_serial_ctrl_dev, 1);
    if (ret < 0)
    {
        goto fail_add_cdev;
    }

    virt_serial_ctrl_class = class_create(THIS_MODULE, VIRT_SERIAL_CONTROL_DEVICE);
    device_create(virt_serial_ctrl_class, NULL, virt_serial_ctrl_dev, NULL, VIRT_SERIAL_CONTROL_DEVICE);
    return 0;

fail_add_cdev:
    kfree(&virt_serial_ctrl_cdev);
fail_alloc_dev:
    unregister_chrdev_region(virt_serial_ctrl_dev, 1);
    kfree(&virt_serial_ctrl_dev);
    return ret;
}

static void remove_virt_serial_ctrl_dev(void)
{
    device_destroy(virt_serial_ctrl_class, virt_serial_ctrl_cdev.dev);
    class_destroy(virt_serial_ctrl_class);
    cdev_del(&virt_serial_ctrl_cdev);
    unregister_chrdev_region(virt_serial_ctrl_cdev.dev, 1);
    kfree(&virt_serial_ctrl_cdev);
}

static long virt_serial_ctrl_ioctl(struct file *file, unsigned int cmd, unsigned long args)
{
    if (_IOC_TYPE(cmd) != VIRT_SERIAL_IOCTL_MAGIC)
    {
        return -ENOTTY;
    }

    switch (cmd)
    {
        case VIRT_SERIAL_IOCTL_CREATE_DEVICE:
        {
            struct virt_serial_config config;
            IOCTL_FETCH_ARGS(args, config);

            return create_virt_serial_handle(&config);
        }
        case VIRT_SERIAL_IOCTL_REMOVE_DEVICE:
        {
            devname_t devname;
            IOCTL_FETCH_ARGS(args, devname);

            return remove_virt_serial_handle(devname);
        }
        case VIRT_SERIAL_IOCTL_PRESERVE:
            struct virt_serial_config config = {
                .devname = "ttyVCOM0",
                .baud = 115200,
            };
            return create_virt_serial_handle(&config);
        default:
            return -ENOTTY;
    }
}

// NOLINTNEXTLINE(*-interfaces-global-init)
static const struct file_operations virt_serial_ctrl_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = virt_serial_ctrl_ioctl,
};

//============================================================================//
// virt_serial_drv.c
//============================================================================//

// NOLINTNEXTLINE(*-interfaces-global-init)
static struct uart_driver virt_serial_drv = {
    .owner          = THIS_MODULE,
    .driver_name    = VIRT_SERIAL_DRIVER_NAME,
    .dev_name       = VIRT_SERIAL_DEVICE_PREFIX,
    .major          = VIRT_SERIAL_MAJOR,
    .minor          = VIRT_SERIAL_MINOR,
    .nr             = VIRT_SERIAL_NR,
    .cons           = NULL,
};

/**
 * See also:
 *   https://www.kernel.org/doc/html/latest/driver-api/serial/driver.html#c.uart_ops
 */
static const struct uart_ops virt_serial_uart_ops = {
    .tx_empty       = virt_serial_tx_empty        ,
    .set_mctrl      = virt_serial_set_mctrl       ,
    .get_mctrl      = virt_serial_get_mctrl       ,
    .stop_tx        = virt_serial_stop_tx         ,
    .start_tx       = virt_serial_start_tx        ,
    .send_xchar     = virt_serial_send_xchar      ,
    .stop_rx        = virt_serial_stop_rx         ,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
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

/**
 * This function tests whether the transmitter fifo and shifter for the port is empty.
 */
static unsigned int virt_serial_tx_empty(struct uart_port *port)
{
//    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
//    struct circ_buf *xmit = &serial_port->port.state->xmit;
//    return uart_circ_empty(xmit);
    return TIOCSER_TEMT;
}

/**
 * This function sets the modem control lines for port to the state described by mctrl.
 */
static void virt_serial_set_mctrl(struct uart_port *uart_port, unsigned int mctrl)
{
    // Intentionally left blank with empty implementation.
}

/**
 * Returns the current state of modem control inputs of port.
 */
static unsigned int virt_serial_get_mctrl(struct uart_port *uart_port)
{
    // Intentionally left blank with empty implementation.
    return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

/**
 * Stop transmitting characters. This might be due to the CTS line becoming inactive or the tty layer indicating
 * we want to stop transmission due to an XOFF character.
 */
static void virt_serial_stop_tx(struct uart_port *port)
{
    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
    unsigned long flags = 0;

    spin_lock_irqsave(&serial_port->write_lock, flags);

    serial_port->rx_enable_flag = false;

    spin_unlock_irqrestore(&serial_port->write_lock, flags);
}

/**
 * Start transmitting characters.
 */
static void virt_serial_start_tx(struct uart_port *port)
{
    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
    unsigned long flags = 0;

    spin_lock_irqsave(&serial_port->write_lock, flags);

    serial_port->tx_enable_flag = true;

    spin_unlock_irqrestore(&serial_port->write_lock, flags);
}

/**
 * Transmit a high priority character, even if the port is stopped. This is used to implement XON/XOFF flow
 * control and tcflow(). If the serial driver does not implement this function, the tty core will append
 * the character to the circular buffer and then call start_tx() / stop_tx() to flush the data out.
 */
static void virt_serial_send_xchar(struct uart_port *port, char ch)
{
    // TODO
}

/**
 * Stop receiving characters; the port is in the process of being closed.
 */
static void virt_serial_stop_rx(struct uart_port *port)
{
    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
    unsigned long flags = 0;

    spin_lock_irqsave(&serial_port->write_lock, flags);

    serial_port->rx_enable_flag = false;

    spin_unlock_irqrestore(&serial_port->write_lock, flags);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
/**
 * Start receiving characters.
 */
static void virt_serial_start_rx(struct uart_port *port)
{
    struct virt_serial_port *serial_port = container_of(port, struct virt_serial_port, port);
    unsigned long flags = 0;

    spin_lock_irqsave(&serial_port->write_lock, flags);

    serial_port->rx_enable_flag = true;

    spin_unlock_irqrestore(&serial_port->write_lock, flags);
}
#endif

/**
 * Enable the modem status interrupts.
 */
static void virt_serial_enable_ms(struct uart_port *port)
{
    // Intentionally left blank with empty implementation.
}

/**
 * Control the transmission of a break signal. If ctl is nonzero, the break signal should be transmitted.
 * The signal should be terminated when another call is made with a zero ctl.
 */
static void virt_serial_break_ctl(struct uart_port *port, int break_state)
{
    // Intentionally left blank with empty implementation.
}

/**
 * Grab any interrupt resources and initialise any low level driver state. Enable the port for reception. It
 * should not activate RTS nor DTR; this will be done via a separate call to set_mctrl().
 */
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
        goto fail_exit;
    }
    if (kfifo_alloc(&serial_port->tx_fifo, FIFO_SIZE, GFP_KERNEL))
    {
        kfifo_free(&serial_port->rx_fifo);
        ret = -ENOMEM;
        goto fail_exit;
    }

    spin_unlock_irqrestore(&serial_port->write_lock, flags);

    return 0;

fail_exit:
    spin_unlock_irqrestore(&serial_port->write_lock, flags);

    return ret;
}

/**
 * Disable the port, disable any break condition that may be in effect, and free any interrupt resources. It
 * should not disable RTS nor DTR; this will have already been done via a separate call to set_mctrl().
 */
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

/**
 * Change the port parameters, including word length, parity, stop bits.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void virt_serial_set_termios(struct uart_port *port, struct ktermios *termios, const struct ktermios *old)
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
static void virt_serial_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
#endif
{
    unsigned char cval = 0;
    baud_t baud = 0;

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

/**
 * Perform any power management related activities on the specified port. state indicates the new state
 * (defined by enum uart_pm_state), oldstate indicates the previous state.
 */
static void virt_serial_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
#ifdef CONFIG_PM
    // Intentionally left blank with empty implementation.
#endif
}

/**
 * Return a pointer to a string constant describing the specified port, or return NULL, in which case the
 * string ‘unknown’ is substituted.
 */
static const char* virt_serial_type(struct uart_port *port)
{
    return NULL;
}

/**
 * Release any memory and IO region resources currently in use by the port.
 */
static void virt_serial_release_port(struct uart_port *port)
{
    // TODO
}

/**
 * Request any memory and IO region resources required by the port. If any fail, no resources should be registered
 * when this function returns, and it should return -EBUSY on failure.
 */
static int virt_serial_request_port(struct uart_port *port)
{
    // Intentionally left blank with empty implementation.
    return 0;
}

/**
 * Perform any autoconfiguration steps required for the port. type contains a bit mask of the required
 * configuration. UART_CONFIG_TYPE indicates that the port requires detection and identification. port->type
 * should be set to the type found, or PORT_UNKNOWN if no port was detected.
 *
 * UART_CONFIG_IRQ indicates autoconfiguration of the interrupt signal, which should be probed using standard
 * kernel autoprobing techniques. This is not necessary on platforms where ports have interrupts internally
 * hard wired (eg, system on a chip implementations).
 */
static void virt_serial_config_port(struct uart_port *port, int type)
{
    if (type & UART_CONFIG_TYPE)
    {
        // TODO
        port->type = type;
        virt_serial_request_port(port);
    }
}

/**
 * Verify the new serial port information contained within serinfo is suitable for this port type.
 */
static int virt_serial_verify_port(struct uart_port *port, struct serial_struct *serial)
{
    return 0;
}

/**
 * Perform any port specific IOCTLs. IOCTL commands must be defined using the standard numbering system
 * found in <asm/ioctl.h>.
 */
static int virt_serial_uart_ioctl(struct uart_port *port, unsigned int cmd, unsigned long args)
{
    if (_IOC_TYPE(cmd) != VIRT_SERIAL_IOCTL_MAGIC)
    {
        return -ENOTTY;
    }

    switch (cmd)
    {
        case VIRT_SERIAL_IOCTL_SET_BAUDRATE:
        {
            // uart_set_baud_rate(port, args);
            break;
        }
        default:
        {
            return -ENOTTY;
        }
    }
    return 0;
}

#ifdef CONFIG_CONSOLE_POLL
/**
 * Called by kgdb to perform the minimal hardware initialization needed to support poll_put_char() and
 * poll_get_char(). Unlike startup(), this should not request interrupts.
 */
static int virt_serial_poll_init(struct uart_port *port)
{
    // TODO
    return 0;
}

/**
 * Called by kgdb to write a single character ch directly to the serial port. It can and should block until
 * there is space in the TX FIFO.
 */
static void virt_serial_poll_put_char(struct uart_port *port, unsigned char ch)
{
    // TODO
}

/**
 * Called by kgdb to read a single character directly from the serial port. If data is available, it should
 * be returned; otherwise the function should return NO_POLL_CHAR immediately.
 */
static int virt_serial_poll_get_char(struct uart_port *port)
{
    // TODO
    return 0;
}
#endif

//============================================================================//
// virt_serial.c
//============================================================================//

/**
 * Kernel module init entrypoint
 */
static int __init virt_serial_init(void)
{
    int ret = 0;
    printk(KERN_INFO "VirtSerial driver initialized named virt_serial\n");
    ret = platform_driver_register(&virt_serial_plt_drv);
    if (ret < 0)
    {
        goto fail_register_plt_driver;
    }

    ret = uart_register_driver(&virt_serial_drv);
    if (ret < 0)
    {
        goto fail_register_uart_driver;
    }

    ret = create_virt_serial_ctrl_dev();
    if (ret < 0)
    {
        goto fail_create_ctrl_dev;
    }
    return 0;

fail_create_ctrl_dev:
    uart_unregister_driver(&virt_serial_drv);
fail_register_uart_driver:
    platform_driver_unregister(&virt_serial_plt_drv);
fail_register_plt_driver:
    kfree(&virt_serial_plt_drv);
    kfree(&virt_serial_drv);
    return ret;
}

/**
 * Kernel module exit
 */
static void __exit virt_serial_exit(void)
{
    remove_virt_serial_ctrl_dev();
    uart_unregister_driver(&virt_serial_drv);
    platform_driver_unregister(&virt_serial_plt_drv);
    kfree(&virt_serial_plt_drv);
    kfree(&virt_serial_drv);
    printk(KERN_INFO "VirtSerial driver unloaded\n");
}

module_init(virt_serial_init);
module_exit(virt_serial_exit);

/* Module information */
MODULE_LICENSE(DRIVER_LICENSE);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
