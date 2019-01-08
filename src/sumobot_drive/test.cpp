#include "mbed_serial.h"

#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>

#define START char(0x02)
#define END char(0x03)
#define LEFT_PWM char(0x04)
#define RIGHT_PWM char(0x05)
#define LEFT_DIRECTION char(0x06)
#define RIGHT_DIRECTION char(0x07)

MbedSerial serial("/dev/ttyACM0");

int main() {
    std::stringstream ss;
    std::string msg;
    char l, r;
    bool ldir, rdir;
    l = 0;
    r = 100;
    ldir = true;
    rdir = false;
    while(1) {	
       	ss.clear();
	msg.clear();
        ss << START << LEFT_PWM << l << RIGHT_PWM << r << END;
        ss >> msg;

        serial.send_msg(msg);
	
	std::cout << msg << std::endl;

        if(ldir) {
            l++;
            if(l == 100) {
                ldir = false;
            }
        }
        else {
            l--;
            if(l == 0) {
                ldir = true;
            }
        }
        if(rdir) {
            r++;
            if(r == 100) {
                rdir = false;
            }
        }
        else {
            r--;
            if(r == 0) {
                rdir = true;
            }
        }
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
