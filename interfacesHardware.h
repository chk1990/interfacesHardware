/**
 * @author Christoph Kolhoff
 * @file interfacesHardware.h
 */

#ifndef _INTERFACES_H_
#define _INTERFACES_H_

#ifdef __unix
//#include <linux/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/spi/spidev.h>
#include <net/if.h>
#include <netinet/in.h> // for internet domain addresses
#include <sys/socket.h>
#include <unistd.h>
#endif // __unix

#include <stdint.h>
#include <string>
#include <vector>

// GPIO defines
#define GPIO_IN "in"
#define GPIO_OUT "out"
#define GPIO_HIGH "1"
#define GPIO_LOW "0"
#define GPIO_EDGE_RISING "rising"
#define GPIO_EDGE_FALLING "falling"
#define GPIO_EDGE_BOTH "both"
#define GPIO_EDGE_NONE "none"
#define GPIO_ACTIVE_LOW "1"
#define GPIO_ACTIVE_HIGH "0"

// UART defines
#define BAUD_0 B0
#define BAUD_50 B50
#define BAUD_110 B110
#define BAUD_134 B134
#define BAUD_150 B150
#define BAUD_200 B200
#define BAUD_300 B300
#define BAUD_600 B600
#define BAUD_1200 B1200
#define BAUD_1800 B1800
#define BAUD_2400 B2400
#define BAUD_4800 B4800
#define BAUD_9600 B9600

using namespace std;

/** @defgroup linuxInterfaces Linux interfaces
 *  This group contains several functions for interfacing other devices
 *  @{
 */

/**
 * @class IBase
 * @author Christoph Kolhoff
 * @brief Basic interface class to be inherited by other classes to interface other devices. This class is not designed to be used directly but to be inherited.
 */
class IBase {
protected:
    int _fileId = -1; ///< Returned file ID for the device
    int _nChars = -1; ///< Number of chars returned by reading and writing
#ifdef __unix
    //
#endif // __unix

public:
#ifdef __unix
    //
#endif // __unix
};

/**
 * @class IGpio
 * @author Christoph Kolhoff
 * @brief Class to read from and write to GPIO pins.
 *
 * @par User API example - Write to pin
 * @code
 * #include <iostream>
 * #include "interfacesHardware.h"
 *
 * using namespace std;
 *
 * int
 * main(void) {
 *     IGpio Gpio;
 *     Gpio.addPin(23, GPIO_OUT);
 *
 *     int8_t retWrite = Gpio.writePin(23, GPIO_HIGH);
 *
 *     return 0;
 * }
 * @endcode
 *
 * @par User API example - Read from pin
 * @code
 * #include <iostream>
 * #include "interfacesHardware.h"
 *
 * using namespace std;
 *
 * int
 * main(void) {
 *     IGpio Gpio;
 *     Gpio.addPin(24, GPIO_IN);
 *
 *     uint8_t retRead = Gpio.readPin(24);
 *
 *     return 0;
 * }
 * @endcode
 * @todo Complete API example
 */
class IGpio : public IBase {
    //   * @note https://www.ics.com/blog/how-control-gpio-hardware-c-or-c
    //   * @note https://www.ics.com/blog/gpio-programming-using-sysfs-interface
    //   * @note https://www.kernel.org/doc/Documentation/gpio/sysfs.txt
public:
    IGpio();
#ifdef __unix
    IGpio(char *filenameOpen, char *filenameClose, char *pathGpio);
#endif // __unix
    ~IGpio();

    int8_t addPin(int pin, string direction);
    int8_t setPinDir(int pin, string direction);
    int8_t setActiveLow(int pin);
    int8_t setActiveHigh(int pin);
    int8_t setEdgeRising(int pin);
    int8_t setEdgeFalling(int pin);
    int8_t setEdgeBoth(int pin);
    int8_t setEdgeNone(int pin);

    uint8_t readPin(uint8_t pin);
    int8_t writePin(uint8_t pin, string level);

private:
    vector<int> _gpioBcm; ///< Used to store the corresponding BCM pins.

#ifdef __unix
    //vector<int> _pinId; ///< Used to store the corresponding file descriptors in - given by the OS.
    int _currPin = -1; ///< Current pin to work with

    char *_filenamePortOpen; ///< Device name on Linux systems
    char *_filenamePortClose; ///< Device name on Linux systems
    char *_pathGpio; ///< Path where all GPIO executables are stored

    int _fdOpen = -1; ///< File descriptor for the executable to generate GPIOs
    int _fdClose = -1; ///< File descriptor for the executable to destroy GPIOs
#endif // __unix

    int init(void);
    int deinit(void);
};

/**
 * @class II2c_Master
 * @author Christoph Kolhoff
 * @brief Class to connect I2C devices. Currently this only works on Linux systems. The clock speed is usually 400 kHz on Linux systems.
 *
 * @par User API example 1 - Simply reading from register
 * The BUS device used for testing is ST Microelectronics LSM9DS1 IMU
 * @code
 * #include <iostream>
 * #include "interfacesHardware.h"
 *
 * using namespace std;
 *
 * int
 * main(void) {
 *
 *      // select the I2C device
 *      char* interfaceFile = (char*)"/dev/i2c-1";
 *
 *      // variable for checking for errors
 *      int ret = 0;
 *
 *      // buffers for data transfer
 *      uint8_t bufRx[10];
 *      uint8_t bufTx[10] = {0xF}; // contains the address for WHO_AM_I of the IMU
 *
 *      // setting up the BUS communication
 *      II2c_Master imu;
 *      imu.setDevice(interfaceFile);
 *      imu.setAddr(0x6a);
 *
 *      // test communication
 *      ret = imu.openDevice();
 *      cout << "Opened I2C device: " << ret << endl;
 *
 *      ret = imu.sendAddr();
 *      cout << "Sended address: " << ret << endl;
 *
 *      ret = imu.writeBytes(&buffer, 1);
 *      cout << "Write bytes: " << ret << endl;
 *
 *      imu.readBytes(&buffer, 1);
 *      cout << "Read address: " << ret << endl;
 *      printf("Data read: %u\n", buffer); // returns value 104 on success
 *
 *      ret = I2c.closeDevice();
 *      cout << "Closed device: " << ret << endl;
 *
 *      return 0;
 * }
 * @endcode
 *
 * @par User API example 2 - Write to register and read value again
 * The BUS device used for testing is ST Microelectronics LSM9DS1 IMU
 * @code
 * #include <iostream>
 * #include "interfacesHardware.h"
 *
 * using namespace std;
 *
 * int
 * main(void) {
 *
 *      uint8_t buffer[10] = {0xF};
 *      char* filename = (char*)"/dev/i2c-1";
 *      int ret = 0;
 *
 *      II2c_Master I2c;
 *      I2c.setDevice(filename);
 *      I2c.setAddr(0x6a);
 *
 *      ret = I2c.openDevice();
 *      cout << "Opened I2C device: " << ret << endl;
 *
 *      ret = I2c.sendAddr();
 *      //ret = I2c.sendAddr(0x6A);
 *      cout << "Sended address: " << ret << endl;
 *
 *      buffer[0] = {0x10}; // register to write to
 *      buffer[1] = {0b00000011}; // value to write to register
 *      ret = I2c.writeBytes(buffer, 2);
 *      cout << "Write bytes: " << ret << endl;
 *
 *      ret = I2c.sendAddr();
 *      ret = I2c.writeBytes(buffer, 1);
 *
 *      I2c.readBytes(buffer, 1);
 *      cout << "Read address: " << ret << endl;
 *      printf("Data read: %u\n", buffer[0]);
 *
 *      ret = I2c.closeDevice();
 *      cout << "Closed device: " << ret << endl;
 *
 *      return 0;
 * }
 * @endcode
 */
class II2c_Master : public IBase {
    //    * @note https://community.nxp.com/t5/i-MX-RT/Can-t-send-spi-message-in-imx6-with-qt/m-p/791366
    //    * @note https://stackoverflow.com/questions/56433271/ioctl-cant-send-spi-message-invalid-argument-beaglebone-black
    //    * @note https://raspberry-projects.com/pi/programming-in-c/i2c/using-the-i2c-interface
    //    * @note https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial/all
    //    * @note https://elinux.org/Interfacing_with_I2C_Devices
private:
    int _addr = 0; ///< Address of the device
#ifdef __unix
    char *_filenameDevice; ///< Device name on Linux systems
#endif // __unix

public:
    // constructors and destructors
    II2c_Master(void);
    ~II2c_Master(void);

    // configuration methods
#ifdef __unix
    void setDevice(char *filename);
#endif // __unix

    void setAddr(int add);

    // operating methods
    int8_t openDevice(void);
    int8_t closeDevice(void);
    int8_t sendAddr(void);
    int8_t sendAddr(uint8_t addr);
    int8_t readBytes(uint8_t *dataRx, uint8_t length);
    int8_t writeBytes(uint8_t *dataTx, uint8_t length);
};

/**
 * @class ISpi_Master
 * @author Christoph Kolhoff
 * @brief Class to connect SPI devices. Currently this only works on Linux systems. The class is limited to one device by now. The overall feature to be considered is that reading and writing is always performed simultaneously bytewise.
 * @par User API example
 * @code
 * #include "interfacesHardware.h"
 *
 * int
 * main(void) {
 *
 *      // define parameters for the SPI device
 *      char* interfaceFile = (char*)"/dev/spidev0.1";
 *      int mode = 0;
 *      int speed = 500000;
 *      int bits = 8;
 *      int bitorder = 0;
 *      int csRelease = 0;
 *
 *      // variable for checking for errors
 *      int ret = 0;
 *
 *      ISpi_Master ISpi;
 *      ISpi.openDevice(interfaceFile, mode, speed, bits, bitorder, csRelease);
 *
 *      uint8_t dataIn[10] = {0x8F, 0};
 *      uint8_t dataOut[10] = {0, 0};
 *      ret = ISpi.readWriteBytes(dataOut, dataIn, 2);
 *
 *      ret = ISpi.closeDevice();
 *
 *      return ret;
 * }
 * @endcode
 */
class ISpi_Master : public IBase {
    // https://raspberry-projects.com/pi/programming-in-c/spi/using-the-spi-interface
    // https://www.kernel.org/doc/html/latest/spi/spidev.html
    // https://stackoverflow.com/questions/56433271/ioctl-cant-send-spi-message-invalid-argument-beaglebone-black
    // https://community.nxp.com/t5/i-MX-RT/Can-t-send-spi-message-in-imx6-with-qt/m-p/791366
    // https://docs.huihoo.com/doxygen/linux/kernel/3.7/structspi__ioc__transfer.html
private:
#ifdef __unix
    char *_filenameDevice; ///< Device name on Linux systems
    unsigned char _mode = 0; ///< Specify CPOL and CPHA: CPOL indicates the initial clock polarity. CPOL=0 means the clock starts low, so the first (leading) edge is rising, and the second (trailing) edge is falling. CPOL=1 means the clock starts high, so the first (leading) edge is falling. CPHA indicates the clock phase used to sample data; CPHA=0 says sample on the leading edge, CPHA=1 means the trailing edge. Modes: 0 = (0, 0), 1 = (0, 1), 2 = (1, 0), 3 = (1, 1)
    int _bits = 0; ///< Bits per word
    int _bitorder = 0; /// 0: MSB first, 1: LSB first
    int _speed = 0; ///< Clock speed
    spi_ioc_transfer _spiObj; ///< Object collecting buffers etc. for comunication
    int _csRelease = 0; ///< 0 = Set CS high after a transfer, 1 = leave CS set low
#endif // __unix
public:
    // constructors and destructors
    ISpi_Master(void);
    ~ISpi_Master(void);

    // configuration methods
    int8_t openDevice(char *filename, int mode, int speed, int bits, int bitorder, int csRelease);
    int8_t closeDevice(void);

    // operating methods
    int8_t readWriteBytes(uint8_t *dataRx, uint8_t *dataTx, uint8_t lenRxTx);
};

/**
 * @class ITcpClient
 * @author Christoph Kolhoff
 * @brief Class to define a TCP Client. Currently this only works on Linux systems.
 *
 * @par User API example
 * @code
 * #include "interfacesHardware.h"
 *
 * int
 * main(void) {
 *
 *      char *serverIp = "192.168.43.112";
 *      int port = 5000;
 *
 *      ITcpClient TcpClient;
 *      TcpClient.init(serverIp, port);
 *
 *      ITcpClient.connect2server();
 *
 *      char buf[256];
 *      ITcpClient.receive(buf);
 *      ITcpClient.transmit(buf, 10);
 *
 *      ITcpClient.closeSocket();
 *
 *      return 0;
 * }
 * @endcode
 * @note Use ITcpServer for the server.
 */
class ITcpClient : public IBase {
    // https://simpledevcode.wordpress.com/2016/06/16/client-server-chat-in-c-using-sockets/comment-page-1/
private:
    char *_serverIp; ///< IP address of the server
    int _portNum = -1; ///< Port number on which communication is started
    // int _nChars = -1; ///< Number of chars returned by reading and writing

    char _bufRx[256]; ///< Buffer to read data in (input buffer)
    char _bufTx[256]; ///< Buffer to write data from (output buffer)

#ifdef __unix
    struct sockaddr_in _serverAddr; // maybe delete the keyword struct ???
    struct hostent *_server;
#endif // __unix

public:
    // constructors and destructors
    ITcpClient(void);
    ~ITcpClient(void);
    
    // configuration methods
#ifdef __unix
    void init(char* serverIp, int pNum);
#endif // __unix

    // connect and disconnect
    int8_t connect2server(void);
    int8_t closeSocket(void);
    int8_t isConnected(void);
    
    // operating methods
    int receive(char *Rx);
    int transmit(char *Tx, int nByte);
#ifdef __unix
    //
#endif // __unix
};

/**
 * @class ITcpServer
 * @author Christoph Kolhoff
 * @brief Class to define a TCP Server. Currently this only works on Linux systems.
 *
 * @par User API example
 * @code
 * #include "interfacesHardware.h"
 *
 * int
 * main(void) {
 *
 *      int port = 5000;
 *
 *      ITcpServer TcpServer;
 *      TcpServer.init(port);
 *
 *      TcpServer.wait4client();
 *
 *      TcpServer.acceptClient();
 *
 *      char buf[256];
 *      TcpServer.receive(buf);
 *
 *      TcpServer.transmit(buf, 10);
 *
 *      TcpServer.closeSocket();
 *
 *      return 0;
 * }
 * @endcode
 * @note Use ITcpClient for the client.
 */
class ITcpServer : public IBase {
    // https://www.linuxhowtos.org/C_C++/socket.htm
    // https://simpledevcode.wordpress.com/2016/06/16/client-server-chat-in-c-using-sockets/comment-page-1/
    // http://www.qnx.com/developers/docs/qnx_4.25_docs/tcpip50/prog_guide/sock_advanced_tut.html
private:
    int _socketAccept = -1; ///< Returned by the system accept call. This should be used for all further operations.
    int _portNum = -1; ///< Port number on which communication has been accepted
    //int _nChars = -1; ///< Number of chars returned by reading and writing

#ifdef __unix
    socklen_t _clilen; ///< Size of the address of the client for accept system call

    struct sockaddr_in _servAddr; ///< Containing the internet address of the server
    struct sockaddr_in _cliAddr; ///< Containing the internet address of the client
#endif // __unix

    char _bufRx[256]; ///< Buffer to read data in (input buffer)
    char _bufTx[256]; ///< Buffer to write data from (output buffer)

public:
    // constructors and destructors
    ITcpServer(void);
    ~ITcpServer(void);

    // configuration methods
    void init(int pNum);

    // connect and disconnect
    int8_t wait4client(void);
    int acceptClient(void);
    int8_t closeSocket(void);
    int8_t isConnected(void);

    // operating methods
    int receive(char *Rx);
    int transmit(char *Tx, int nByte);
#ifdef __unix
    //
#endif // __unix
};

/**
 * @class IUart
 * @author Christoph Kolhoff
 * @brief Class to define a UART interface. Currently this only works on Linux systems.
 *
 * @par User API example
 * @code
 * #include "interfacesHardware.h"
 *
 * int
 * main(void) {
 *      char *filename = "";
 *      int baud = 256000;
 *
 *      IUart Interface;
 *      Interface.openDevice(filename, baud);
 *
 *      Interface.receive();
 *      Interface.transmit();
 *
 *      Interface.closeDevice();
 * }
 * @endcode
 * @todo complete and test
 */
class IUart : public IBase {
    // https://man7.org/linux/man-pages/man2/ioctl_tty.2.html
    // https://embetronicx.com/tutorials/linux/device-drivers/ioctl-tutorial-in-linux/
    // https://www.element14.com/community/community/manufacturers/xsens/blog/2021/07/01/interfacing-mti-devices-with-the-raspberry-pi
    // https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
    // https://www.cmrr.umn.edu/~strupp/serial.html
    // https://github.com/Digilent/linux-userspace-examples/blob/master/uart_example_linux/src/uart.c
public:
    // constructors and destructors
    IUart(void);
    ~IUart(void);

    // configuration methods, open and close
#ifdef __unix
    void openDevice(char *filename, int baud);
#endif // __unix
    int8_t closeDevice(void);

    // operating methods
    int receive(int32_t *Rx);
    int transmit(int32_t *Tx, int nByte);
};

/** @} */ // end of group "linuxInterfaces"
#endif // _INTERFACES_H_
