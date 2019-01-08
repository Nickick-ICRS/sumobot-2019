#ifndef __MBED_SERIAL_H
#define __MBED_SERIAL_H

#include <iostream>
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/file.h> // flock()

class MbedSerial {
public:
    MbedSerial(std::string port);
    ~MbedSerial();

    void send_msg(std::string msg);
private:
    void setup();
    std::string m_port;
};

#endif // __MBED_SERIAL_H
