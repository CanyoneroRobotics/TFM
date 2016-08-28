#ifndef ARDUINO_H
#define ARDUINO_H

#include <SerialStream.h>
#include <SerialStreamBuf.h>
#include <SerialPort.h>
#include <string>
#include <stdint.h>

class Arduino{
public:
    Arduino();
    int open();
    std::vector<int8_t> read();
    void write(std::vector<int8_t> buffer);
    void close();
private:
    std::string serialDevice;
    LibSerial::SerialStream serial;
};

#endif // ARDUINO_H
