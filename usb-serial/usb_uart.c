#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define CONNECT_STRING	"Hello ! Welcome to Neal's usb-uart connection.\r\n"

int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);	/* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;			/* 8-bit characters */
	tty.c_cflag &= ~PARENB;			/* no parity bit */
	tty.c_cflag &= ~CSTOPB;			/* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;		/* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}

	return 0;
}

void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		printf("Error tcsetattr: %s\n", strerror(errno));
}

/*
 * The main purpose in this program is to communication between two serial ports.
 * Example:
 * 	PC1 - usb interface - SoC (ttyGS0 - ttyS2) - rs232 interface - PC2
 *
 * Input parameters:
 * 	input: argv[1]: ttyGS0 (UDC gadgets for USB cdc-acm)
 * 	input: argv[2]: ttyS2 (uart port)
 *
 * Cross compile:
 * 	$ arm-aspeed-linux-gnueabihf-gcc usb_uart.c -o usb-uart
 *
 * Run on AST2600:
 * 	$ ./usb-uart /dev/ttyGS0 /dev/ttyS2
 *
 * Then you can key anything either on PC1 or PC2.
 *
 */
int main(int argc, char **argv)
{
	char *portname_a = argv[1];
	char *portname_b = argv[2];
	char *xstr = CONNECT_STRING;
	int xlen = strlen(xstr);
	int wlen, fda, fdb;

	fda = open(portname_a, O_RDWR | O_NOCTTY | O_SYNC);
	if (fda < 0) {
		printf("Error opening %s: %s\n", portname_a, strerror(errno));
		return -1;
	}

	fdb = open(portname_b, O_RDWR | O_NOCTTY | O_SYNC);
	if (fdb < 0) {
		printf("Error opening %s: %s\n", portname_a, strerror(errno));
		return -1;
	}

	/*baudrate 115200, 8 bits, no parity, 1 stop bit */
	set_interface_attribs(fda, B115200);
	set_interface_attribs(fdb, B115200);

	//set_mincount(fd, 0);                /* set to pure timed read */

	/* simple output */
	wlen = write(fda, xstr, xlen);
	if (wlen != xlen) {
		printf("Error from write: %d, %d\n", wlen, errno);
	}

	wlen = write(fdb, xstr, xlen);
	if (wlen != xlen) {
		printf("Error from write: %d, %d\n", wlen, errno);
	}

	/* delay for output */
	tcdrain(fda);
	tcdrain(fdb);

	unsigned char buf[80];
	int len, rdlena, rdlenb;

	/* simple noncanonical input */
	do {
		/* clean buf */
		memset(buf, 0, sizeof(buf));

		rdlena = read(fda, buf, sizeof(buf) - 1);
		rdlenb = read(fdb, buf, sizeof(buf) - 1);
		if (rdlena > 0 || rdlenb > 0) {
			if (rdlena)
				len = rdlena;
			else
				len = rdlenb;

			if (buf[0] == '\r') {
				buf[len++] = '\n';
				buf[len] = 0;

			} else {
				buf[len] = 0;
			}

			printf("%s", buf);

			wlen = write(fda, buf, len + 1);
			wlen = write(fdb, buf, len + 1);
#ifdef HEXDUMP
			unsigned char   *p;
			printf("Read %d:", len);
			for (p = buf; len-- > 0; p++)
				printf(" 0x%x", *p);
			printf("\n");
#endif
		} else if (rdlena < 0 || rdlenb < 0) {
			printf("Error from read: %d %d: %s\n", rdlena, rdlenb, strerror(errno));
		} else {  /* rdlen == 0 */
//			printf("Timeout from read\n");
			continue;
		}

		/* repeat read to get full message */
	} while (1);

	return 0;
}
