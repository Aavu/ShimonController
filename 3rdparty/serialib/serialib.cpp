/*!
 \file    serialib.cpp
 \brief   Source file of the class serialib. This class is used for communication over a serial device.
 \author  Philippe Lucidarme (University of Angers)
 \modifiedBy Raghavasimhan Sankaranarayanan
 \version 2.0
 \date    december the 27th of 2019
 \Modified May 05 2022

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


This is a licence-free software, it can be used by anyone who try to build a better world.
 */

#include "serialib.h"



//_____________________________________
// ::: Constructors and destructors :::


/*!
    \brief      Constructor of the class serialib.
*/
serialib::serialib()
{
#if defined (_WIN32) || defined( _WIN64)
    // Set default value for RTS and DTR (Windows only)
    currentStateRTS=true;
    currentStateDTR=true;
    hSerial = INVALID_HANDLE_VALUE;
#endif
#if defined (__linux__) || defined(__APPLE__)
    m_fd = -1;
#endif
}


/*!
    \brief      Destructor of the class serialib. It close the connection
*/
// Class desctructor
serialib::~serialib()
{
    closeDevice();
}



//_________________________________________
// ::: Configuration and initialization :::



/*!
     \brief Open the serial port
     \param Device : Port name (/dev/ttyS0, /dev/ttyACM0, /dev/ttyUSB0 ... for linux)
     \param Bauds : Baud rate of the serial port.
               \n Supported baud rate for Linux :\n
                        - 110
                        - 300
                        - 600
                        - 1200
                        - 2400
                        - 4800
                        - 9600
                        - 19200
                        - 38400
                        - 57600
                        - 115200
                        - 230400
     \param Databits : Number of data bits in one UART transmission.

            \n Supported values: \n
                - SERIAL_DATABITS_5 (5)
                - SERIAL_DATABITS_6 (6)
                - SERIAL_DATABITS_7 (7)
                - SERIAL_DATABITS_8 (8)
                - SERIAL_DATABITS_16 (16) (not supported on Unix)

     \param Parity: Parity type

            \n Supported values: \n
                - SERIAL_PARITY_NONE (N)
                - SERIAL_PARITY_EVEN (E)
                - SERIAL_PARITY_ODD (O)
                - SERIAL_PARITY_MARK (MARK) (not supported on Unix)
                - SERIAL_PARITY_SPACE (SPACE) (not supported on Unix)
    \param Stopbit: Number of stop bits

            \n Supported values:
                - SERIAL_STOPBITS_1 (1)
                - SERIAL_STOPBITS_1_5 (1.5) (not supported on Unix)
                - SERIAL_STOPBITS_2 (2)

     \return 1 success
     \return -1 device not found
     \return -2 error while opening the device
     \return -3 error while getting port parameters
     \return -4 Speed (Bauds) not recognized
     \return -5 error while writing port parameters
     \return -6 error while writing timeout parameters
     \return -7 Databits not recognized
     \return -8 Stopbits not recognized
     \return -9 Parity not recognized
  */
char serialib::openDevice(const char *Device) {
    // Open device
    m_fd = open(Device, O_RDWR | O_NOCTTY | O_NDELAY);
    // If the device is not open, return -1
    if (m_fd == -1) return -2;
    // Open the device in nonblocking mode
    fcntl(m_fd, F_SETFL, FNDELAY);

    // Get the current options of the port
    tcgetattr(m_fd, &m_options);
    return 0;
}

bool serialib::isDeviceOpen() const
{
    return m_fd >= 0;
}

/*!
     \brief Close the connection with the current device
*/
void serialib::closeDevice()
{
    close (m_fd);
    m_fd = -1;
}

char serialib::init(unsigned int Bauds, SerialDataBits dataBits, SerialParity Parity, SerialStopBits stopBits) {
    // Clear all the options
    bzero(&m_options, sizeof(m_options));

    int databits_flag = 0;
    switch(dataBits) {
        case SERIAL_DATABITS_5: databits_flag = CS5; break;
        case SERIAL_DATABITS_6: databits_flag = CS6; break;
        case SERIAL_DATABITS_7: databits_flag = CS7; break;
        case SERIAL_DATABITS_8: databits_flag = CS8; break;
            //16 bits and everything else not supported
        default: return -7;
    }
    int stopbits_flag = 0;
    switch(stopBits) {
        case SERIAL_STOPBITS_1: stopbits_flag = 0; break;
        case SERIAL_STOPBITS_2: stopbits_flag = CSTOPB; break;
            //1.5 stopbits and everything else not supported
        default: return -8;
    }
    int parity_flag = 0;
    switch(Parity) {
        case SERIAL_PARITY_NONE: parity_flag = 0; break;
        case SERIAL_PARITY_EVEN: parity_flag = PARENB; break;
        case SERIAL_PARITY_ODD: parity_flag = (PARENB | PARODD); break;
            //mark and space parity not supported
        default: return -9;
    }

    // Configure the device : data bits, stop bits, parity, no control flow
    // Ignore modem control lines (CLOCAL) and Enable receiver (CREAD)
    m_options.c_cflag |= ( CLOCAL | CREAD | databits_flag | parity_flag | stopbits_flag);
    m_options.c_iflag |= ( IGNPAR | IGNBRK );
//    m_options.c_lflag &= ~(ICANON);
    // Timer unused
    m_options.c_cc[VTIME]=0;
    // At least on character before satisfy reading
    m_options.c_cc[VMIN]=0;

    // Set Baudrate
    char err = setBaudrate(Bauds);
    if (err != 1) return err;

    // Activate the settings
    tcsetattr(m_fd, TCSANOW, &m_options);
    // Success
    return (1);
}


char serialib::setBaudrate(const unsigned int Bauds) {
// Prepare speed (Bauds)
    speed_t         Speed;
    switch (Bauds)
    {
        case 110  :     Speed=B110; break;
        case 300  :     Speed=B300; break;
        case 600  :     Speed=B600; break;
        case 1200 :     Speed=B1200; break;
        case 2400 :     Speed=B2400; break;
        case 4800 :     Speed=B4800; break;
        case 9600 :     Speed=B9600; break;
        case 19200 :    Speed=B19200; break;
        case 38400 :    Speed=B38400; break;
        case 57600 :    Speed=B57600; break;
        case 115200 :   Speed=B115200; break;
        case 230400 :   Speed=B230400; break;
        default : return -4;
    }

    // Set the baud rate
    cfsetispeed(&m_options, Speed);
    cfsetospeed(&m_options, Speed);

    return 1;
}


//___________________________________________
// ::: Read/Write operation on characters :::



/*!
     \brief Write a char on the current serial port
     \param Byte : char to send on the port (must be terminated by '\0')
     \return 1 success
     \return -1 error while writting data
  */
char serialib::writeChar(const char Byte)
{
    // Write the char
    if (write(m_fd, &Byte, 1) != 1) return -1;

    // Write operation successfull
    return 1;
}



//________________________________________
// ::: Read/Write operation on strings :::


/*!
     \brief     Write a string on the current serial port
     \param     receivedString : string to send on the port (must be terminated by '\0')
     \return     1 success
     \return    -1 error while writing data
  */
char serialib::writeString(const char *receivedString)
{
    // Length of the string
    size_t Length=strlen(receivedString);
    // Write the string
    if (write(m_fd, receivedString, Length) != Length) return -1;
    // Write operation successful
    return 1;
}

// _____________________________________
// ::: Read/Write operation on bytes :::



/*!
     \brief Write an array of data on the current serial port
     \param Buffer : array of bytes to send on the port
     \param NbBytes : number of byte to send
     \return 1 success
     \return -1 error while writing data
  */
char serialib::writeBytes(const void *Buffer, const unsigned int NbBytes)
{
    // Write data
    if (write (m_fd, Buffer, NbBytes) != (ssize_t)NbBytes) return -1;
    // Write operation successful
    return 1;
}

/*!
     \brief Write an array of data on the current serial port
     \param Buffer : array of bytes to send on the port
     \param NbBytes : number of byte to send
     \return 1 success
     \return -1 error while writing data
  */
char serialib::writeBytes(const uint8_t *Buffer, const unsigned int NbBytes)
{
    return writeBytes((const void *)Buffer, NbBytes);
}


/*!
     \brief Wait for a byte from the serial device and return the data read
     \param pByte : data read on the serial device
     \param timeOut_ms : delay of timeout before giving up the reading
            If set to zero, timeout is disable (Optional)
     \return 1 success
     \return 0 Timeout reached
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
  */
char serialib::readChar(char *pByte,unsigned int timeOut_ms)
{
    // Timer used for timeout
    timeOut         timer;
    // Initialise the timer
    timer.initTimer();
    // While Timeout is not reached
    while (timer.elapsedTime_ms()<timeOut_ms || timeOut_ms==0)
    {
        // Try to read a byte on the device
        switch (read(m_fd, pByte, 1)) {
        case 1  : return 1; // Read successful
        case -1 : return -2; // Error while reading
        }
    }
    return 0;
}



/*!
     \brief Read a string from the serial device (without TimeOut)
     \param receivedString : string read on the serial device
     \param FinalChar : final char of the string
     \param MaxNbBytes : maximum allowed number of bytes read
     \return >0 success, return the number of bytes read
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
     \return -3 MaxNbBytes is reached
  */
int serialib::readStringNoTimeOut(char *receivedString,char finalChar,unsigned int maxNbBytes)
{
    // Number of characters read
    unsigned int    NbBytes=0;
    // Returned value from Read
    char            charRead;

    // While the buffer is not full
    while (NbBytes<maxNbBytes)
    {
        // Read a character with the restant time
        charRead=readChar(&receivedString[NbBytes]);

        // Check a character has been read
        if (charRead==1)
        {
            // Check if this is the final char
            if (receivedString[NbBytes]==finalChar)
            {
                // This is the final char, add zero (end of string)
                receivedString  [++NbBytes]=0;
                // Return the number of bytes read
                return (int)NbBytes;
            }

            // The character is not the final char, increase the number of bytes read
            NbBytes++;
        }

        // An error occurred while reading, return the error number
        if (charRead<0) return charRead;
    }
    // Buffer is full : return -3
    return -3;
}


/*!
     \brief Read a string from the serial device (with timeout)
     \param receivedString : string read on the serial device
     \param finalChar : final char of the string
     \param maxNbBytes : maximum allowed number of bytes read
     \param timeOut_ms : delay of timeout before giving up the reading (optional)
     \return  >0 success, return the number of bytes read
     \return  0 timeout is reached
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
     \return -3 MaxNbBytes is reached
  */
int serialib::readString(char *receivedString,char finalChar,unsigned int maxNbBytes,unsigned int timeOut_ms)
{
    // Check if timeout is requested
    if (timeOut_ms==0) return readStringNoTimeOut(receivedString,finalChar,maxNbBytes);

    // Number of bytes read
    unsigned int    nbBytes=0;
    // Character read on serial device
    char            charRead;
    // Timer used for timeout
    timeOut         timer;
    long int        timeOutParam;

    // Initialize the timer (for timeout)
    timer.initTimer();

    // While the buffer is not full
    while (nbBytes<maxNbBytes)
    {
        // Compute the TimeOut for the next call of ReadChar
        timeOutParam = timeOut_ms-timer.elapsedTime_ms();

        // If there is time remaining
        if (timeOutParam>0)
        {
            // Wait for a byte on the serial link with the remaining time as timeout
            charRead=readChar(&receivedString[nbBytes],timeOutParam);

            // If a byte has been received
            if (charRead==1)
            {
                // Check if the character received is the final one
                if (receivedString[nbBytes]==finalChar)
                {
                    // Final character: add the end character 0
                    receivedString  [++nbBytes]=0;
                    // Return the number of bytes read
                    return (int)nbBytes;
                }
                // This is not the final character, just increase the number of bytes read
                nbBytes++;
            }
            // Check if an error occurred during reading char
            // If an error occurred, return the error number
            if (charRead<0) return charRead;
        }
        // Check if timeout is reached
        if (timer.elapsedTime_ms()>timeOut_ms)
        {
            // Add the end character
            receivedString[nbBytes]=0;
            // Return 0 (timeout reached)
            return 0;
        }
    }

    // Buffer is full : return -3
    return -3;
}


/*!
     \brief Read an array of bytes from the serial device (with timeout)
     \param buffer : array of bytes read from the serial device
     \param maxNbBytes : maximum allowed number of bytes read
     \param timeOut_ms : delay of timeout before giving up the reading
     \param sleepDuration_us : delay of CPU relaxing in microseconds (Linux only)
            In the reading loop, a sleep can be performed after each reading
            This allows CPU to perform other tasks
     \return >=0 return the number of bytes read before timeout or
                requested data is completed
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
  */
int serialib::readBytes (void *buffer,unsigned int maxNbBytes,unsigned int timeOut_ms, unsigned int sleepDuration_us) const
{
    // Timer used for timeout
    timeOut          timer;
    // Initialise the timer
    timer.initTimer();
    unsigned int     NbByteRead=0;
    // While Timeout is not reached
    while (timer.elapsedTime_ms()<timeOut_ms || timeOut_ms==0)
    {
//        auto l = available();
//        if (l < maxNbBytes) {
//            usleep(sleepDuration_us);
//            continue;
//        }
        // Compute the position of the current byte
        unsigned char* Ptr=(unsigned char*)buffer+NbByteRead;
        // Try to read a byte on the device
        ssize_t Ret=read(m_fd, (void*)Ptr, maxNbBytes - NbByteRead);
        // Error while reading
        if (Ret==-1) return -2;

        // One or several byte(s) has been read on the device
        if (Ret>0)
        {
            // Increase the number of read bytes
            NbByteRead+=Ret;
            // Success : bytes has been read
            if (NbByteRead>=maxNbBytes)
                return (int)NbByteRead;
        }
        // Suspend the loop to avoid charging the CPU
        usleep (sleepDuration_us);
    }
    // Timeout reached, return the number of bytes read
//    if (NbByteRead == 0) {
//        int a = 0;
//    }
    return (int)NbByteRead;
}

int serialib::readBytes (uint8_t *buffer,unsigned int maxNbBytes,unsigned int timeOut_ms, unsigned int sleepDuration_us) const {
    return readBytes((void*)buffer, maxNbBytes, timeOut_ms, sleepDuration_us);
}


// _________________________
// ::: Special operation :::



/*!
    \brief Empty receiver buffer
    \return If the function succeeds, the return value is nonzero.
            If the function fails, the return value is zero.
*/
char serialib::flushReceiver() const
{
    // Purge receiver
    tcflush(m_fd, TCIFLUSH);
    return true;
}

int serialib::sendBreak(int time) const {
    return tcsendbreak(m_fd, time);
}

/*!
    \brief  Return the number of bytes in the received buffer (UNIX only)
    \return The number of bytes received by the serial provider but not yet read.
*/
int serialib::available() const
{
    int nBytes=0;
    // Return number of pending bytes in the receiver
    ioctl(m_fd, FIONREAD, &nBytes);
    return nBytes;
}



// __________________
// ::: I/O Access :::

/*!
    \brief      Set or unset the bit DTR (pin 4)
                DTR stands for Data Terminal Ready
                Convenience method :This method calls setDTR and clearDTR
    \param      status = true set DTR
                status = false unset DTR
    \return     If the function fails, the return value is false
                If the function succeeds, the return value is true.
*/
bool serialib::DTR(bool status)
{
    if (status)
        // Set DTR
        return this->setDTR();
    else
        // Unset DTR
        return this->clearDTR();
}


/*!
    \brief      Set the bit DTR (pin 4)
                DTR stands for Data Terminal Ready
    \return     If the function fails, the return value is false
                If the function succeeds, the return value is true.
*/
bool serialib::setDTR()
{
    // Set DTR
    int status_DTR=0;
    ioctl(m_fd, TIOCMGET, &status_DTR);
    status_DTR |= TIOCM_DTR;
    ioctl(m_fd, TIOCMSET, &status_DTR);
    return true;
}

/*!
    \brief      Clear the bit DTR (pin 4)
                DTR stands for Data Terminal Ready
    \return     If the function fails, the return value is false
                If the function succeeds, the return value is true.
*/
bool serialib::clearDTR()
{
    // Clear DTR
    int status_DTR=0;
    ioctl(m_fd, TIOCMGET, &status_DTR);
    status_DTR &= ~TIOCM_DTR;
    ioctl(m_fd, TIOCMSET, &status_DTR);
    return true;
}



/*!
    \brief      Set or unset the bit RTS (pin 7)
                RTS stands for Request to Send
                Convenience method :This method calls setDTR and clearDTR
    \param      status = true set DTR
                status = false unset DTR
    \return     false if the function fails
    \return     true if the function succeeds
*/
bool serialib::RTS(bool status)
{
    if (status)
        // Set RTS
        return this->setRTS();
    else
        // Unset RTS
        return this->clearRTS();
}


/*!
    \brief      Set the bit RTS (pin 7)
                RTS stands for Request to Send
    \return     If the function fails, the return value is false
                If the function succeeds, the return value is true.
*/
bool serialib::setRTS() const
{
    // Set RTS
    int status_RTS=0;
    ioctl(m_fd, TIOCMGET, &status_RTS);
    status_RTS |= TIOCM_RTS;
    ioctl(m_fd, TIOCMSET, &status_RTS);
    return true;
}



/*!
    \brief      Clear the bit RTS (pin 7)
                RTS stands for Request to Send
    \return     If the function fails, the return value is false
                If the function succeeds, the return value is true.
*/
bool serialib::clearRTS() const
{
    // Clear RTS
    int status_RTS=0;
    ioctl(m_fd, TIOCMGET, &status_RTS);
    status_RTS &= ~TIOCM_RTS;
    ioctl(m_fd, TIOCMSET, &status_RTS);
    return true;
}




/*!
    \brief      Get the CTS's status (pin 8)
                CTS stands for Clear To Send
    \return     Return true if CTS is set otherwise false
  */
bool serialib::isCTS() const
{
    int status=0;
    //Get the current status of the CTS bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_CTS;
}



/*!
    \brief      Get the DSR's status (pin 6)
                DSR stands for Data Set Ready
    \return     Return true if DTR is set otherwise false
  */
bool serialib::isDSR() const
{
    int status=0;
    //Get the current status of the DSR bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_DSR;
}






/*!
    \brief      Get the DCD's status (pin 1)
                CDC stands for Data Carrier Detect
    \return     true if DCD is set
    \return     false otherwise
  */
bool serialib::isDCD() const
{
    int status=0;
    //Get the current status of the DCD bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_CAR;
}


/*!
    \brief      Get the RING's status (pin 9)
                Ring Indicator
    \return     Return true if RING is set otherwise false
  */
bool serialib::isRI() const
{
    int status=0;
    //Get the current status of the RING bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_RNG;
}


/*!
    \brief      Get the DTR's status (pin 4)
                DTR stands for Data Terminal Ready
                May behave abnormally on Windows
    \return     Return true if CTS is set otherwise false
  */
bool serialib::isDTR() const
{
    int status=0;
    //Get the current status of the DTR bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_DTR  ;
}



/*!
    \brief      Get the RTS's status (pin 7)
                RTS stands for Request To Send
                May behave abnormally on Windows
    \return     Return true if RTS is set otherwise false
  */
bool serialib::isRTS() const
{
    int status=0;
    //Get the current status of the CTS bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_RTS;
}


// ******************************************
//  Class timeOut
// ******************************************


/*!
    \brief      Constructor of the class timeOut.
*/
// Constructor
timeOut::timeOut()
{}


/*!
    \brief      Initialise the timer. It writes the current time of the day in the structure PreviousTime.
*/
//Initialize the timer
void timeOut::initTimer()
{
#if defined (NO_POSIX_TIME)
    LARGE_INTEGER tmp;
    QueryPerformanceFrequency(&tmp);
    counterFrequency = tmp.QuadPart;
    // Used to store the previous time (for computing timeout)
    QueryPerformanceCounter(&tmp);
    previousTime = tmp.QuadPart;
#else
    gettimeofday(&previousTime, NULL);
#endif
}

/*!
    \brief      Returns the time elapsed since initialization.  It setPosition the current time of the day in the structure CurrentTime.
                Then it returns the difference between CurrentTime and PreviousTime.
    \return     The number of microseconds elapsed since the functions InitTimer was called.
  */
//Return the elapsed time since initialization
unsigned long int timeOut::elapsedTime_ms()
{
#if defined (NO_POSIX_TIME)
    // Current time
    LARGE_INTEGER CurrentTime;
    // Number of ticks since last call
    int sec;

    // Get current time
    QueryPerformanceCounter(&CurrentTime);

    // Compute the number of ticks elapsed since last call
    sec=CurrentTime.QuadPart-previousTime;

    // Return the elapsed time in milliseconds
    return sec/(counterFrequency/1000);
#else
    // Current time
    struct timeval CurrentTime;
    // Number of seconds and microseconds since last call
    int sec,usec;

    // Get current time
    gettimeofday(&CurrentTime, NULL);

    // Compute the number of seconds and microseconds elapsed since last call
    sec=CurrentTime.tv_sec-previousTime.tv_sec;
    usec=CurrentTime.tv_usec-previousTime.tv_usec;

    // If the previous usec is higher than the current one
    if (usec<0)
    {
        // Recompute the microseonds and substract one second
        usec=1000000-previousTime.tv_usec+CurrentTime.tv_usec;
        sec--;
    }

    // Return the elapsed time in milliseconds
    return sec*1000+usec/1000;
#endif
}
