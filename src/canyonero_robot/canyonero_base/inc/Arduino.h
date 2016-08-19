#ifndef ARDUINO_H
#define ARDUINO_H

#include <SerialStream.h>
#include <SerialStreamBuf.h>
#include <SerialPort.h>
#include <string>

class Arduino{
public:
    Arduino();
    int open();
    SerialPort::DataBuffer read();
    void write(SerialPort::DataBuffer buffer);
    void close();
private:
    std::string serialDevice;
    LibSerial::SerialStream serial;
};

#endif // ARDUINO_H
