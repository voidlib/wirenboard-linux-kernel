#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/wbec.h>
#include "linux/kthread.h"

#define DRIVER_NAME "wbec-uart"

#define printk(...)
#define dev_info(...)
#define snprintf(...)

#define WBEC_REGMAP_PAD_WORDS_COUNT   		5
#define WBEC_REGMAP_READ_BIT				BIT(15)

#define WBEC_UART_REGMAP_BUFFER_SIZE		64

struct wbec_uart {
	struct device *dev;
	struct spi_device *spi;
	struct regmap *regmap;
	struct uart_port port;

	atomic_t irq_handled;
	atomic_t tx_start;

	// thread
	struct task_struct *thread;

	bool tx_in_progress;
};

struct wbec_regmap_header {
	u16 address : 15;
	u16 read_bit : 1;
	u16 pad[WBEC_REGMAP_PAD_WORDS_COUNT];
} __packed;

struct uart_rx {
    u8 read_bytes_count;
    u8 ready_for_tx;
    u8 read_bytes[WBEC_UART_REGMAP_BUFFER_SIZE];
} __packed;

struct uart_tx {
    u8 bytes_to_send_count;
    u8 reserved;
    u8 bytes_to_send[WBEC_UART_REGMAP_BUFFER_SIZE];
} __packed;

union uart_tx_regs {
	struct {
		struct wbec_regmap_header header;
		struct uart_tx tx;
	};
	u16 buf[33 + 6];
};

union uart_rx_regs {
	struct {
		struct wbec_regmap_header header;
		struct uart_rx rx;
	};
	u16 buf[33 + 6];
};

union uart_ctrl {
	struct {
		u16 reset : 1;
		u16 want_to_tx : 1;
	};
	u16 buf[1];
};

static struct uart_driver wbec_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = "ttyWBE",
	.nr = 1,
};

static void swap_bytes(u16 *buf, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		buf[i] = __fswab16(buf[i]);
	}
}

static void wbec_spi_exchange_sync(struct wbec_uart *wbec_uart)
{
	union uart_rx_regs rx;
	union uart_tx_regs tx = {};
	struct uart_port *port = &wbec_uart->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct spi_message msg;
	struct spi_transfer transfer = {};
	int ret;
	char str[256];

	tx.header.address = 0x100;

	spi_message_init(&msg);

	transfer.tx_buf = tx.buf;
	transfer.rx_buf = rx.buf;
	transfer.len = sizeof(tx);

	spi_message_add_tail(&transfer, &msg);

	// prepare tx data

	uart_port_lock(port);

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		tx.tx.bytes_to_send_count = 0;
	} else {
		int i;
		unsigned int to_send = uart_circ_chars_pending(xmit);
		printk(KERN_INFO "to_send linux: %d; head=%d, tail=%d\n", to_send, xmit->head, xmit->tail);
		to_send = min(to_send, ARRAY_SIZE(tx.tx.bytes_to_send));
		printk(KERN_INFO "to_send spi: %d\n", to_send);

		tx.tx.bytes_to_send_count = to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			u8 c = xmit->buf[(xmit->tail + i) & (UART_XMIT_SIZE - 1)];
			tx.tx.bytes_to_send[i] = c;
		}
	}

	uart_port_unlock(port);

	swap_bytes(tx.buf, ARRAY_SIZE(tx.buf));

	// transfer
	ret = spi_sync(wbec_uart->spi, &msg);
	if (ret) {
		pr_err("spi_sync failed with error %d\n", ret);
		return;
	}

	swap_bytes(rx.buf, ARRAY_SIZE(rx.buf));

	uart_port_lock(port);

	snprintf(str, ARRAY_SIZE(str), "received_bytes: %d: ", rx.rx.read_bytes_count);
	if (rx.rx.read_bytes_count > 0) {
		int i;
		for (i = 0; i < rx.rx.read_bytes_count; i++) {
			u8 c = rx.rx.read_bytes[i];
			snprintf(str, ARRAY_SIZE(str), "%s[%.2X]", str, c);
			wbec_uart->port.icount.rx++;
			uart_insert_char(&wbec_uart->port, 0, 0, c, TTY_NORMAL);
		}
		tty_flip_buffer_push(&wbec_uart->port.state->port);
	}

	if (rx.rx.ready_for_tx) {
		u8 bytes_sent = tx.tx.bytes_to_send_count;

		printk(KERN_INFO "bytes_sent=%d; tail_was=%d\n", bytes_sent, xmit->tail);


		if (bytes_sent > 0) {
			uart_xmit_advance(&wbec_uart->port, bytes_sent);
		} else {
			if (wbec_uart->tx_in_progress) {
				wbec_uart->tx_in_progress = false;
			}
		}

		printk(KERN_INFO "new_tail=%d\n", xmit->tail);

		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(port);
	}

	uart_port_unlock(port);
}

static void wbec_srart_tx_sync(struct wbec_uart *wbec_uart)
{
	if (!wbec_uart->tx_in_progress) {
		union uart_tx_regs tx_start = {};
		struct uart_port *port = &wbec_uart->port;
		struct circ_buf *xmit = &port->state->xmit;
		int i;
		unsigned int to_send;
		struct spi_message msg;
		struct spi_transfer transfer = {};

		tx_start.header.address = 0x190;

		spi_message_init(&msg);

		transfer.tx_buf = tx_start.buf;

		spi_message_add_tail(&transfer, &msg);

		uart_port_lock(port);

		to_send = uart_circ_chars_pending(xmit);

		wbec_uart->tx_in_progress = true;

		printk(KERN_INFO "to_send linux: %d; head=%d, tail=%d\n", to_send, xmit->head, xmit->tail);
		to_send = min(to_send, ARRAY_SIZE(tx_start.tx.bytes_to_send));
		printk(KERN_INFO "to_send spi: %d\n", to_send);

		tx_start.tx.bytes_to_send_count = to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			u8 c = xmit->buf[(xmit->tail + i) & (UART_XMIT_SIZE - 1)];
			tx_start.tx.bytes_to_send[i] = c;
		}
		uart_xmit_advance(port, to_send);

		uart_port_unlock(port);

		transfer.len = (1 + 5 + 1 + (to_send + 1) / 2) * 2;

		swap_bytes(tx_start.buf, ARRAY_SIZE(tx_start.buf));

		spi_sync(wbec_uart->spi, &msg);
	}
}



static int wbec_uart_thread(void *data)
{
	struct wbec_uart *wbec_uart = data;
	int ret;

	printk(KERN_INFO "%s called\n", __func__);

	while (!kthread_should_stop()) {
		if (atomic_read(&wbec_uart->tx_start)) {
			atomic_set(&wbec_uart->tx_start, 0);
			printk(KERN_INFO "tx_start in thread\n");

			wbec_srart_tx_sync(wbec_uart);

		} else if (atomic_read(&wbec_uart->irq_handled)) {
			atomic_set(&wbec_uart->irq_handled, 0);
			printk(KERN_INFO "IRQ handled in thread\n");

			wbec_spi_exchange_sync(wbec_uart);
		} else {
			schedule();
		}
	}

	return 0;
}



static unsigned int wbec_uart_tx_empty(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	printk(KERN_INFO "%s called\n", __func__);

	return TIOCSER_TEMT;
}

static void wbec_uart_start_tx(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);

	printk(KERN_INFO "%s called\n", __func__);

	atomic_set(&wbec_uart->tx_start, 1);
}

static void wbec_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	printk(KERN_INFO "%s called\n", __func__);
}

static unsigned int wbec_uart_get_mctrl(struct uart_port *port)
{
	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static void wbec_uart_stop_tx(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	printk(KERN_INFO "%s called\n", __func__);
}

static void wbec_uart_stop_rx(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	printk(KERN_INFO "%s called\n", __func__);
}

static void wbec_uart_break_ctl(struct uart_port *port, int break_state)
{
	printk(KERN_INFO "%s called\n", __func__);
}

static int wbec_uart_startup(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	union uart_ctrl uart_ctrl = {};

	printk(KERN_INFO "%s called\n", __func__);

	uart_ctrl.reset = 1;

	wbec_uart->tx_in_progress = false;

	// wbec_write_regs_sync(wbec_uart->spi, 0x1E0, uart_ctrl.buf, sizeof(uart_ctrl) / 2);
	// mdelay(1);
	return 0;
}

static void wbec_uart_shutdown(struct uart_port *port)
{
	struct wbec_uart *wbec_uart = container_of(port,
					      struct wbec_uart,
					      port);
	printk(KERN_INFO "%s called\n", __func__);

}

static void wbec_uart_set_termios(struct uart_port *, struct ktermios *new,
				       const struct ktermios *old)
{
	printk(KERN_INFO "%s called\n", __func__);

	// uart_update_timeout(port, termios->c_cflag, 115200);
}

static void wbec_uart_config_port(struct uart_port *port, int flags)
{
	printk(KERN_INFO "%s called\n", __func__);
	if (flags & UART_CONFIG_TYPE)
		port->type = 123;
}

static int wbec_uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static void wbec_uart_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
	printk(KERN_INFO "%s called\n", __func__);
}

static int wbec_uart_request_port(struct uart_port *port)
{
	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static void wbec_uart_null_void(struct uart_port *port)
{
	printk(KERN_INFO "%s called\n", __func__);
}

static const char * wbec_uart_type(struct uart_port *port)
{
	printk(KERN_INFO "%s called\n", __func__);
	return 0;
}

static irqreturn_t wbec_uart_irq(int irq, void *dev_id)
{
	struct wbec_uart *wbec_uart = dev_id;

	printk(KERN_INFO "%s called\n", __func__);

	atomic_set(&wbec_uart->irq_handled, 1);

	// Wake up the thread if necessary
	wake_up_process(wbec_uart->thread);

	return IRQ_HANDLED;
}

static const struct uart_ops wbec_uart_ops = {
	.tx_empty	= wbec_uart_tx_empty,
	.set_mctrl	= wbec_uart_set_mctrl,
	.get_mctrl	= wbec_uart_get_mctrl,
	.stop_tx	= wbec_uart_stop_tx,
	.start_tx	= wbec_uart_start_tx,
	.stop_rx	= wbec_uart_stop_rx,
	.break_ctl	= wbec_uart_break_ctl,
	.startup	= wbec_uart_startup,
	.shutdown	= wbec_uart_shutdown,
	.set_termios	= wbec_uart_set_termios,
	.type		= wbec_uart_type,
	.request_port	= wbec_uart_request_port,
	.release_port	= wbec_uart_null_void,
	.config_port	= wbec_uart_config_port,
	.verify_port	= wbec_uart_verify_port,
	.pm		= wbec_uart_pm,
};

static int wbec_uart_config_rs485(struct uart_port *,
				  struct ktermios *termios,
				  struct serial_rs485 *rs485)
{
	printk(KERN_INFO "%s called\n", __func__);
	// port->rs485 = *rs485;

	return 0;
}

static int wbec_uart_probe(struct platform_device *pdev)
{
	struct wbec *wbec = dev_get_drvdata(pdev->dev.parent);
	struct wbec_uart *wbec_uart;
	int ret, irq;
	u16 wbec_id;

	dev_info(&pdev->dev, "%s called\n", __func__);

	wbec_uart = devm_kzalloc(&pdev->dev, sizeof(struct wbec_uart),
				GFP_KERNEL);
	if (!wbec_uart)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ: %d\n", irq);
		return irq;
	}

	wbec_uart->dev = &pdev->dev;
	wbec_uart->spi = wbec->spi;
	wbec_uart->regmap = wbec->regmap;

	platform_set_drvdata(pdev, wbec_uart);

	// ret = wbec_read_regs_sync(wbec->spi, 0x00, &wbec_id, 1);
	// dev_info(&pdev->dev, "wbec_id 0xB0: %.2X\n", wbec_id);

	// Register the UART driver
	ret = uart_register_driver(&wbec_uart_driver);
	if (ret) {
		pr_err("Failed to register UART driver\n");
		return ret;
	}

	// Register the UART port
	// Initialize the UART port
	wbec_uart->port.ops = &wbec_uart_ops;
	wbec_uart->port.dev = &pdev->dev;
	wbec_uart->port.type = PORT_GENERIC;
	wbec_uart->port.irq = irq;
	wbec_uart->port.iotype = UPIO_PORT;
	wbec_uart->port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
	// wbec_uart->port.flags = UPF_BOOT_AUTOCONF;
	wbec_uart->port.rs485_config = wbec_uart_config_rs485;

	wbec_uart->port.uartclk = 115200;
	wbec_uart->port.fifosize = 64;
	wbec_uart->port.line = 0;

	atomic_set(&wbec_uart->irq_handled, 0);
	atomic_set(&wbec_uart->tx_start, 0);

	wbec_uart->thread = kthread_run(wbec_uart_thread, wbec_uart, "wbec_uart_thread");
	if (IS_ERR(wbec_uart->thread)) {
		pr_err("Failed to create thread\n");
		return PTR_ERR(wbec_uart->thread);
	}

	ret = uart_add_one_port(&wbec_uart_driver, &wbec_uart->port);
	if (ret) {
		pr_err("Failed to register UART port\n");
		return ret;
	}

	dev_info(&pdev->dev, "IRQ: %d\n", irq);

	ret = devm_request_irq(wbec_uart->dev, irq, wbec_uart_irq,
					IRQF_TRIGGER_RISING,
					dev_name(wbec_uart->dev), wbec_uart);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "WBE UART driver loaded\n");

	return 0;
}

static int wbec_uart_remove(struct platform_device *pdev)
{
	struct wbec_uart *wbec_uart = platform_get_drvdata(pdev);

	printk(KERN_INFO "%s called\n", __func__);

	// Unregister the UART port
	uart_remove_one_port(&wbec_uart_driver, &wbec_uart->port);

	// Unregister the UART driver
	uart_unregister_driver(&wbec_uart_driver);

	kthread_stop(wbec_uart->thread);

	return 0;
}

static const struct of_device_id wbec_uart_of_match[] = {
	{ .compatible = "wirenboard,wbec-uart" },
	{}
};
MODULE_DEVICE_TABLE(of, wbec_uart_of_match);

static struct platform_driver wbec_uart_platform_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = wbec_uart_of_match,
	},
	.probe = wbec_uart_probe,
	.remove = wbec_uart_remove,
};

module_platform_driver(wbec_uart_platform_driver);

MODULE_ALIAS("platform:wbec-uart");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("WBE UART Driver");
