#ifndef _SERIAL_COMM_HPP_
#define _SERIAL_COMM_HPP_

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// const char device_serial[20] = "/dev/pts/3";

int serial_init(const char* dev);
void send_speed(int spd_l, int spd_r);
int read_serial();
void close_serial();

#endif