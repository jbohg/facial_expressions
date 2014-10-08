// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino, Carlos Beltran-Gonzalez
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <facial_expression/SerialDeviceDriver.h>

#include <stdio.h>
#include <stdlib.h>

SerialDeviceDriver::SerialDeviceDriver() {
	verbose=false;
}

SerialDeviceDriver::~SerialDeviceDriver() {
    close();
}

bool SerialDeviceDriver::open(SerialDeviceDriverSettings& config) 
{
    if(config.verbose)
        this->verbose=true;

    ACE_TRACE("SerialHandler::initialize");
    ACE_OS::printf("Starting Serial Port in %s \n", config.CommChannel);
    
    // Initialize serial port
    if(_serialConnector.connect(_serial_dev, ACE_DEV_Addr(config.CommChannel)) == -1)
    { 
        ACE_OS::printf("Invalid communications port in %s \n", config.CommChannel);
        return false;
    } 


    // Set TTY_IO parameter into the ACE_TTY_IO device(_serial_dev)
    if (_serial_dev.control (ACE_TTY_IO::SETPARAMS, &config.SerialParams) == -1)
    {
         ACE_OS::printf("Can not control communications port %s \n", config.CommChannel);
        return false;
    }

    return true;
}

bool SerialDeviceDriver::close(void) {
    return true;
}

bool SerialDeviceDriver::send(const std::string& msg)
{
    int message_size = msg.length();
    if (verbose) ACE_OS::printf("Received string: %s of length %d\n", msg.c_str(), message_size);

    // Write message in the serial device
    ssize_t bytes_written = _serial_dev.send_n((void *)msg.c_str(), message_size);

    if (bytes_written == -1)
        ACE_ERROR((LM_ERROR, ACE_TEXT ("%p\n"), ACE_TEXT ("send")));
    return true;
} 

bool SerialDeviceDriver::send(char *msg, size_t size)
{
    if (verbose) ACE_OS::printf("Received string: %s\n", msg);
    
	// Write message in the serial device
    ssize_t bytes_written = _serial_dev.send_n((void *)msg, size);

    if (bytes_written == -1)
        ACE_ERROR((LM_ERROR, ACE_TEXT ("%p\n"), ACE_TEXT ("send")));
    return true;
}

int SerialDeviceDriver::receiveChar(char& c)
{
	char chr;
	
	//this function call blocks
    ssize_t bytes_read = _serial_dev.recv ((void *) &chr, 1);

    if (bytes_read == -1)
	{
		ACE_ERROR((LM_ERROR, ACE_TEXT ("Error on SerialDeviceDriver : receive \n")));
		return 0;
	}

    if (bytes_read == 0)
	{
        return 0;
	}

	c=chr;
	return 1;
}

int  SerialDeviceDriver::flush()
{
	char chr;
	int count=0;
	ssize_t bytes_read=0;
	do
	{
		bytes_read = _serial_dev.recv ((void *) &chr, 1);
		count+=bytes_read;
	}
	while (bytes_read>0);
	return count;
}

int SerialDeviceDriver::receiveLine(char* buffer, const int MaxLineLength)
{
	int i;
	for (i = 0; i < MaxLineLength -1; ++i)
	{
		char recv_ch;
		int n = receiveChar(recv_ch);
		if (n <= 0)
		{
			return 0;
		}
		if ((recv_ch == '\r') || (recv_ch == '\n'))
		{
			buffer[i] = recv_ch;
			i++;
		    break;
		}
		buffer[i] = recv_ch;
	 }
	 buffer[i] = '\0';
	 return i;
}

bool SerialDeviceDriver::receive(std::string& msg)
{
    char message[1001];

    //this function call blocks
    ssize_t bytes_read = _serial_dev.recv ((void *) message, 1000);

    if (bytes_read == -1)
        ACE_ERROR((LM_ERROR, ACE_TEXT ("Error on SerialDeviceDriver : receive \n")));

    if (bytes_read == 0)  //nothing there
        return true;
        
    message[bytes_read] = 0;

    if (verbose) ACE_OS::printf("Datareceived in Serial DeviceDriver receive:#%s#\n",message);

    // Put message in the bottle
    msg.append(message);

    return true;
}
