/*!
 \file    serialib.cpp
 \brief   Class to manage the serial port
 \author  Philippe Lucidarme (University of Angers) <serialib@googlegroups.com>
 \version 1.2
 \date    28 avril 2011

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


This is a licence-free software, it can be used by anyone who try to build a better world.
 */

#include "serialib.h"


/*!
    \brief      Constructor of the class serialib.
*/
// Class constructor
serialib::serialib()
{}


/*!
    \brief      Destructor of the class serialib. It close the connection
*/
// Class desctructor
serialib::~serialib()
{
    Close();
}



//_________________________________________
// ::: Configuration and initialization :::



/*!
     \brief Open the serial port
     \param Device : Port name (/dev/ttyS0, /dev/ttyACM0, /dev/ttyUSB0 ...)
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

     \return 1 success
     \return -1 device not found
     \return -2 error while opening the device
     \return -3 error while getting port parameters
     \return -4 Speed (Bauds) not recognized
     \return -5 error while writing port parameters
     \return -6 error while writing timeout parameters
  */
char serialib::Open(const char *Device,const unsigned int Bauds)
{
  
    struct termios options;                                             // Structure with the device's options


    // Open device
    fd = open(Device, O_RDWR | O_NOCTTY | O_NDELAY);                    // Open port
    if (fd == -1) return -2;                                            // If the device is not open, return -1
    fcntl(fd, F_SETFL, FNDELAY);                                        // Open the device in nonblocking mode

    // Set parameters
    tcgetattr(fd, &options);                                            // Get the current options of the port
    bzero(&options, sizeof(options));                                   // Clear all the options
    speed_t         Speed;
    switch (Bauds)                                                      // Set the speed (Bauds)
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
    default : return -4;
}
    cfsetispeed(&options, Speed);                                       // Set the baud rate at 115200 bauds
    cfsetospeed(&options, Speed);
    options.c_cflag |= ( CLOCAL | CREAD |  CS8);                        // Configure the device : 8 bits, no parity, no control
    options.c_iflag |= ( IGNPAR | IGNBRK );
    options.c_cc[VTIME]=0;                                              // Timer unused
    options.c_cc[VMIN]=0;                                               // At least on character before satisfy reading
    tcsetattr(fd, TCSANOW, &options);                                   // Activate the settings
    return (1);                                                         // Success

}


/*!
     \brief Close the connection with the current device
*/
void serialib::Close()
{
    close (fd);

}




//___________________________________________
// ::: Read/Write operation on characters :::



/*!
     \brief Write a char on the current serial port
     \param Byte : char to send on the port (must be terminated by '\0')
     \return 1 success
     \return -1 error while writting data
  */
char serialib::WriteChar(const char Byte)
{

    if (write(fd,&Byte,1)!=1)                                           // Write the char
        return -1;                                                      // Error while writting
    return 1;                                                           // Write operation successfull

}



//________________________________________
// ::: Read/Write operation on strings :::


/*!
     \brief Write a string on the current serial port
     \param String : string to send on the port (must be terminated by '\0')
     \return 1 success
     \return -1 error while writting data
  */
char serialib::WriteString(const char *String)
{
    int Lenght=strlen(String);                                          // Lenght of the string
    if (write(fd,String,Lenght)!=Lenght)                                // Write the string
        return -1;                                                      // error while writing
    return 1;                                                           // Write operation successfull

}

// _____________________________________
// ::: Read/Write operation on bytes :::



/*!
     \brief Write an array of data on the current serial port
     \param Buffer : array of bytes to send on the port
     \param NbBytes : number of byte to send
     \return 1 success
     \return -1 error while writting data
  */
char serialib::Write(const void *Buffer, const unsigned int NbBytes)
{
    if (write (fd,Buffer,NbBytes)!=(ssize_t)NbBytes)                              // Write data
        return -1;                                                      // Error while writing
    return 1;                                                           // Write operation successfull

}



/*!
     \brief Wait for a byte from the serial device and return the data read
     \param pByte : data read on the serial device
     \param TimeOut_ms : delay of timeout before giving up the reading
            If set to zero, timeout is disable (Optional)
     \return 1 success
     \return 0 Timeout reached
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
  */
char serialib::ReadChar(char *pByte,unsigned int TimeOut_ms)
{
    TimeOut         Timer;                                              // Timer used for timeout
    Timer.InitTimer();                                                  // Initialise the timer
    while (Timer.ElapsedTime_ms()<TimeOut_ms || TimeOut_ms==0)          // While Timeout is not reached
    {
        switch (read(fd,pByte,1)) {                                     // Try to read a byte on the device
        case 1  : return 1;                                             // Read successfull
        case -1 : return -2;                                            // Error while reading
        }
    }
    return 0;

}



/*!
     \brief Read a string from the serial device (without TimeOut)
     \param String : string read on the serial device
     \param FinalChar : final char of the string
     \param MaxNbBytes : maximum allowed number of bytes read
     \return >0 success, return the number of bytes read
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
     \return -3 MaxNbBytes is reached
  */
int serialib::ReadStringNoTimeOut(char *String,char FinalChar,unsigned int MaxNbBytes)
{
    unsigned int    NbBytes=0;                                          // Number of bytes read
    char            ret;                                                // Returned value from Read
    while (NbBytes<MaxNbBytes)                                          // While the buffer is not full
    {                                                                   // Read a byte with the restant time
        ret=ReadChar(&String[NbBytes]);
        if (ret==1)                                                     // If a byte has been read
        {
            if (String[NbBytes]==FinalChar)                             // Check if it is the final char
            {
                String  [++NbBytes]=0;                                  // Yes : add the end character 0
                return NbBytes;                                         // Return the number of bytes read
            }
            NbBytes++;                                                  // If not, just increase the number of bytes read
        }
        if (ret<0) return ret;                                          // Error while reading : return the error number
    }
    return -3;                                                          // Buffer is full : return -3
}

/*!
     \brief Read a string from the serial device (with timeout)
     \param String : string read on the serial device
     \param FinalChar : final char of the string
     \param MaxNbBytes : maximum allowed number of bytes read
     \param TimeOut_ms : delay of timeout before giving up the reading (optional)
     \return  >0 success, return the number of bytes read
     \return  0 timeout is reached
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
     \return -3 MaxNbBytes is reached
  */
int serialib::ReadString(char *String,char FinalChar,unsigned int MaxNbBytes,unsigned int TimeOut_ms)
{
    if (TimeOut_ms==0)
        return ReadStringNoTimeOut(String,FinalChar,MaxNbBytes);

    unsigned int    NbBytes=0;                                          // Number of bytes read
    char            ret;                                                // Returned value from Read
    TimeOut         Timer;                                              // Timer used for timeout
    long int        TimeOutParam;
    Timer.InitTimer();                                                  // Initialize the timer

    while (NbBytes<MaxNbBytes)                                          // While the buffer is not full
    {                                                                   // Read a byte with the restant time
        TimeOutParam=TimeOut_ms-Timer.ElapsedTime_ms();                 // Compute the TimeOut for the call of ReadChar
        if (TimeOutParam>0)                                             // If the parameter is higher than zero
        {
            ret=ReadChar(&String[NbBytes],TimeOutParam);                // Wait for a byte on the serial link            
            if (ret==1)                                                 // If a byte has been read
            {

                if (String[NbBytes]==FinalChar)                         // Check if it is the final char
                {
                    String  [++NbBytes]=0;                              // Yes : add the end character 0
                    return NbBytes;                                     // Return the number of bytes read
                }
                NbBytes++;                                              // If not, just increase the number of bytes read
            }
            if (ret<0) return ret;                                      // Error while reading : return the error number
        }
        if (Timer.ElapsedTime_ms()>TimeOut_ms) {                        // Timeout is reached
            String[NbBytes]=0;                                          // Add the end caracter
            return 0;                                                   // Return 0
        }
    }
    return -3;                                                          // Buffer is full : return -3
}


/*!
     \brief Read an array of bytes from the serial device (with timeout)
     \param Buffer : array of bytes read from the serial device
     \param MaxNbBytes : maximum allowed number of bytes read
     \param TimeOut_ms : delay of timeout before giving up the reading
     \return 1 success, return the number of bytes read
     \return 0 Timeout reached
     \return -1 error while setting the Timeout
     \return -2 error while reading the byte
  */
int serialib::Read (void *Buffer,unsigned int MaxNbBytes,unsigned int TimeOut_ms)
{
    TimeOut          Timer;                                             // Timer used for timeout
    Timer.InitTimer();                                                  // Initialise the timer
    unsigned int     NbByteRead=0;
    while (Timer.ElapsedTime_ms()<TimeOut_ms || TimeOut_ms==0)          // While Timeout is not reached
    {
        unsigned char* Ptr=(unsigned char*)Buffer+NbByteRead;           // Compute the position of the current byte
        int Ret=read(fd,(void*)Ptr,MaxNbBytes-NbByteRead);              // Try to read a byte on the device
        if (Ret==-1) return -2;                                         // Error while reading
        if (Ret>0) {                                                    // One or several byte(s) has been read on the device
            NbByteRead+=Ret;                                            // Increase the number of read bytes
            if (NbByteRead>=MaxNbBytes)                                 // Success : bytes has been read
                return 1;
        }
    }
    return 0;                                                           // Timeout reached, return 0

}




// _________________________
// ::: Special operation :::



/*!
    \brief Empty receiver buffer (UNIX only)
*/

void serialib::FlushReceiver()
{
    tcflush(fd,TCIFLUSH);
}



/*!
    \brief  Return the number of bytes in the received buffer (UNIX only)
    \return The number of bytes in the received buffer
*/
int serialib::Peek()
{
    int Nbytes=0;
    ioctl(fd, FIONREAD, &Nbytes);
    return Nbytes;
}

// ******************************************
//  Class TimeOut
// ******************************************


/*!
    \brief      Constructor of the class TimeOut.
*/
// Constructor
TimeOut::TimeOut()
{}

/*!
    \brief      Initialise the timer. It writes the current time of the day in the structure PreviousTime.
*/
//Initialize the timer
void TimeOut::InitTimer()
{
    gettimeofday(&PreviousTime, NULL);
}

/*!
    \brief      Returns the time elapsed since initialization.  It write the current time of the day in the structure CurrentTime.
                Then it returns the difference between CurrentTime and PreviousTime.
    \return     The number of microseconds elapsed since the functions InitTimer was called.
  */
//Return the elapsed time since initialization
unsigned long int TimeOut::ElapsedTime_ms()
{
    struct timeval CurrentTime;
    int sec,usec;
    gettimeofday(&CurrentTime, NULL);                                   // Get current time
    sec=CurrentTime.tv_sec-PreviousTime.tv_sec;                         // Compute the number of second elapsed since last call
    usec=CurrentTime.tv_usec-PreviousTime.tv_usec;                      // Compute
    if (usec<0) {                                                       // If the previous usec is higher than the current one
        usec=1000000-PreviousTime.tv_usec+CurrentTime.tv_usec;          // Recompute the microseonds
        sec--;                                                          // Substract one second
    }
    return sec*1000+usec/1000;
}

