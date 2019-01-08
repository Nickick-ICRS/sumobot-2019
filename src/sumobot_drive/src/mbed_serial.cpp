#include "mbed_serial.h"

MbedSerial::MbedSerial(std::string port):m_port(port) {
    setup();
}

void MbedSerial::send_msg(std::string msg) {
    int serial_port = open(m_port.c_str(), O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        std::cout << "Error " << errno << " from open: " 
                  << strerror(errno) << std::endl;
        return;
    }

    // Acquire non-blocking exclusive lock
    if(flock(serial_port, LOCK_EX | LOCK_NB) == -1) {
        throw std::runtime_error("Serial port with file descriptor " +
            std::to_string(serial_port) + " is already locked by another process.");
    }
 
    write(serial_port, msg.c_str(), sizeof(msg.c_str()));
    close(serial_port);
}

void MbedSerial::setup() {
    int serial_port = open(m_port.c_str(), O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        std::cout << "Error " << errno << " from open: " 
                  << strerror(errno) << std::endl;
    }

    termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    // Disable any special handling of received bytes
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); 
    
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to cr/lf

    // We're not actually reading, but if we decide to in the future
    // Then wait for 10 deciseconds max before returning nothing
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
    close(serial_port);
}
