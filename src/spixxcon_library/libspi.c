#include <Python.h>

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

static PyObject* spi_open(PyObject* self, PyObject *args)
{
	int fd;
	char *s;

	if (!PyArg_ParseTuple(args, "s", &s)) {
		return NULL;
	}
	fd = open(s, O_RDWR);
	return Py_BuildValue("i", fd);
}

static PyObject* spi_close(PyObject* self, PyObject *args)
{
	int fd;

	if (!PyArg_ParseTuple(args, "i", &fd)) {
		return NULL;
	}
	return Py_BuildValue("i", close(fd));
}

static PyObject* spi_transfer(PyObject* self, PyObject *args)
{
	int fd;
	char *wr;
	char *rd;
	int len;
	PyObject *retp;

	if (!PyArg_ParseTuple(args, "is#", &fd, &wr, &len)) {
		return NULL;
	}
	rd = malloc(len);
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)wr,
		.rx_buf = (unsigned long)rd,
		.len = len,
		.delay_usecs = 0,
		.speed_hz = 0,
		.bits_per_word = 0,
	};
	ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	retp = Py_BuildValue("s#", rd, len);
	free(rd);
	rd = NULL;
	return retp;
}

static PyObject* spi_mode(PyObject* self, PyObject *args)
{
	int fd;
	int freq;
	int mode;
	char aux;

	if (!PyArg_ParseTuple(args, "iii", &fd, &freq, &mode)) {
		return NULL;
	}

	aux = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	aux |= ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq);

	return Py_BuildValue("i", aux);
}

static PyMethodDef spicon_funcs[] = {
    {"open", (PyCFunction)spi_open, 
     METH_VARARGS, NULL},
    {"close", (PyCFunction)spi_close, 
     METH_VARARGS, NULL},
    {"set_speed_mode", (PyCFunction)spi_mode, 
     METH_VARARGS, NULL},
    {"transfer", (PyCFunction)spi_transfer, 
     METH_VARARGS, NULL},
    {NULL}
};

void initspicon(void)
{
    Py_InitModule3("spicon", spicon_funcs,
                   "C code for SPI in Linux");
}
