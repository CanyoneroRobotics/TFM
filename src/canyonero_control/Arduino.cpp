#include <Arduino.h>

Arduino::Arduino()
{
    serial(dev);
}

int Arduino::open()
{
    serial.Open(SerialPort::BAUD_9600,
                SerialPort::CHAR_SIZE_8,
                SerialPort::PARITY_NONE,
                SerialPort::STOP_BITS_1,
                SerialPort::FLOW_CONTROL_NONE);

    if (!serial.IsOpen())
        return -1;

    return 0;
}

void Arduino::close()
{
    serial.Close();
}

DataBuffer Arduino::read()
{
    SerialPort::DataBuffer buffer;
    serial.Read(buffer, 5, 250);

    return buffer;
}

void Arduino::write(DataBuffer buffer)
{
    serial.Write(buffer);
}
