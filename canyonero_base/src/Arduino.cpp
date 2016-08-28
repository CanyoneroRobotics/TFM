#include "Arduino.h"

Arduino::Arduino()
{
    serialDevice = "/dev/ttyACM0";
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

std::vector<int8_t> Arduino::read()
{
    std::vector<int8_t> buffer;
    unsigned int i = 0;
    char c = '\0';

    // Detect Start of Packet
    while (c != '{')
    {
        serial >> c;
    }

    serial >> c;
    buffer.push_back(c);
    serial >> c;
    buffer.push_back(c);
    serial >> c;
    buffer.push_back(c);
    serial >> c;
    buffer.push_back(c);

    // Detect End of Packet
    while (c != '}')
    {
        serial >> c;
    }

    return buffer;
}

void Arduino::write(std::vector<int8_t> buffer)
{
    serial << '{' << buffer[0] << buffer[1] << buffer[2] << buffer[3] << '}' << std::endl;
}
