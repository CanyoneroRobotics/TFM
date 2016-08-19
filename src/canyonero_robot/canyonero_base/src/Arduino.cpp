#include "Arduino.h"

#include <SerialStream.h>

Arduino::Arduino()
{
    serialDevice = "/dev/ACM0";
}

int Arduino::open()
{
    serial.Open(serialDevice);
    serial.SetBaudRate( LibSerial::SerialStreamBuf::BAUD_9600 );
    serial.SetCharSize( LibSerial::SerialStreamBuf::CHAR_SIZE_8 );
    serial.SetNumOfStopBits( 1 );
    serial.SetParity( LibSerial::SerialStreamBuf::PARITY_NONE );
    serial.SetFlowControl( LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE );

    if (!serial.IsOpen())
        return -1;

    return 0;
}

void Arduino::close()
{
    serial.Close();
}

SerialPort::DataBuffer Arduino::read()
{
    SerialPort::DataBuffer buffer;
    unsigned int i = 0;
    char c;

    serial >> c;
    while (c != '\0')
    {
        buffer.push_back(c);
        serial >> c;
    }

    return buffer;
}

void Arduino::write(SerialPort::DataBuffer buffer)
{
    serial << buffer[0] << buffer[1] << buffer[2] << buffer[3] << std::endl;
}
