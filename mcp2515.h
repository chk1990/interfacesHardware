/**
 * @author Christoph Kolhoff
 * @file mcp2515.h
 */

#ifndef _MCP2515_H_
#define _MCP2515_H_

#ifdef __unix
#include "interfacesHardware.h"
#endif // __unix

#include "group_names_doxy.h"
#define MCP2515_STD_EXT_IDENT 3 ///< Constant for standard and extended identifier to be used in Mcp2515
#define MCP2515_STD_IDENT 0 ///< Constant for standard identifier to be used in Mcp2515
#define MCP2515_EXT_IDENT 1 ///< Constant for extended identifier to be used in Mcp2515
#define MCP2515_ID_FILT_OFF 2 ///< Receiving any message on Mcp2515
#define MCP2515_RX0 0 ///< Macro for Receive buffer 0
#define MCP2515_RX1 1 ///< Macro for Receive buffer 1
#define MCP2515_TX0 0 ///< Macro for Transmit buffer 0
#define MCP2515_TX1 1 ///< Macro for Transmit buffer 1
#define MCP2515_TX2 2 ///< Macro for Transmit buffer 2
#define MCP2515_PRIO_0 0 ///< Lowest priority for transmission buffer
#define MCP2515_PRIO_1 1 ///< Mid-low priority for transmission buffer
#define MCP2515_PRIO_2 2 ///< Mid-high priority for transmission buffer
#define MCP2515_PRIO_3 3 ///< Highest priority for transmission buffer

/**
  * @addtogroup devices
  * @{
  */

/**
 * @class Mcp2515 devices
 * @brief This class is used to interface the Microchip MCP2515 CAN controller with SPI on Linux.
 * @author Christoph Kolhoff
 * @par User API example for receive
 * @code
 * #include <iostream>
 * #include "mcp2515.h"
 *
 * using namespace std;
 *
 * int
 * main(void) {
 *
 *      // Configure the SPI interface
 *      char *interfaceFile = (char*)"/dev/spidev0.0";
 *      int mode = 0; // CPOL, CPHA
 *      int speed = 1000000; // SCKL speed
 *
 *      // Configure the CAN controller
 *      uint8_t bufferRxInd = 0; // index of the receive buffer to read from
 *
 *      uint8_t idType = 0;
 *      uint32_t identifierRx = 0;
 *
 *      Mcp2515 Can;
 *      Can.init(interfaceFile, mode, speed);
 *      Can.configRxBuf(bufferRxInd, idType);
 *      Can.setModeNormal();
 *
 *      cout << "Waiting for received message" << endl;
 *      while(!Can.check4message(bufferRxInd)) {
 *          // wait until a new message is available (no interupt mode)
 *      }
 *
 *      bufferRxInd = Can.getInterruptFlags();
 *
 *      idType = Can.getIdentifierType(bufferRxInd & 0x03);
 *
 *      identifier = Can.getIdentifierRx(bufferRxInd, idType);
 *
 *      uint8_t data[255];
 *      uint8_t nBytes = Can.receiveData(bufferRxInd, data);
 *
 *      cout << "Received " << nBytes << " bytes:" << endl;
 *      for(uint8_t indByte = 0; indByte < nBytes; indByte++){
 *          cout << data[indByte] << " ";
 *      }
 *
 *      // Disconnect device
 *      Can.terminate();
 *
 *      return 0;
 * }
 * @endcode
 *
 * @par User API example for transmit
 * @code
 * #include <iostream>
 * #include "mcp2515.h"
 *
 * int
 * main(void) {
 *
 *      // Configure the SPI interface
 *      char *interfaceFile = (char*)"/dev/spidev0.0";
 *      int mode = 0; // CPOL, CPHA
 *      int speed = 1000000; // SCKL speed

 *      // Configure the CAN controller
 *      uint8_t bufferTxInd = 0; // index of the buffer to transmit
 *      uint32_t identifier = 100; // identifier of the message of interest
 *      uint8_t opMode = 0; // standard frame
 *      uint8_t maskBytes[2] = {0xF, 0xF}; // masking byte for the identifier
 *      uint8_t prio = 3; // priority
 *
 *      uint8_t data[4] = {1, 2, 3, 4};
 *      uint8_t nBytes = 4;
 *
 *      Mcp2515 Can;
 *      Can.init(interfaceFile, mode, speed);
 *      Can.setModeNormal();
 *
 *      Can.transmitData(identifier, opMode, data, nBytes, bufferTxInd, prio);
 *
 *      Can.terminate();
 *
 *      return 0;
 * }
 * @endcode
 * @todo Method for TXBnCTRL analysis
 * @todo extend error handling
 */
class Mcp2515 {
public:
    // constructors and destructors
    Mcp2515(void);
    ~Mcp2515(void);

    // initialization and termination
#ifdef __unix
    int8_t init(char* filename, int mode, int speed);
#endif // __unix
    void resetDevice(void);
    int8_t terminate(void);

    // configuration
    void configRxBuf(uint8_t indRxb, uint8_t modeRx);

    void setFilterRx(uint8_t indFiltRxb, uint32_t identVal, uint8_t identType, uint8_t indRxb);

    void setModeNormal(void);
    void setModeSleep(void);
    void setModeListen(void);
    void setModeLoopback(void);
    void setModeConfig(void);

    void setPropSeg(uint8_t ps);
    void setPs1(uint8_t ps1);
    void setPs2(uint8_t ps2);
    void setSjw(uint8_t sjw);
    void setBrp(uint8_t brp);

    // read and write
    void modBit(uint8_t addr, uint8_t mask, uint8_t data);
    void readReg(uint8_t reg, uint8_t* dataReg, uint8_t nBytes);
    int8_t resetReg(uint8_t reg);
    void terminateTx(void);
    void writeReg(uint8_t reg, uint8_t* val, uint8_t nBytes);

    // transmit
    int8_t transmitData(uint32_t identVal, uint8_t identType, uint8_t* dataIn, uint8_t nBytes, uint8_t indTxb, uint8_t prio);

    // receive
    uint8_t check4message(void);
    uint8_t check4message(uint8_t indRxb);
    void wait4message(uint8_t indRxb);

    uint8_t receiveData(uint8_t indRxb, uint8_t* data);
    uint8_t getIdentifierType(uint8_t indRxb);
    uint32_t getIdentifierRx(uint8_t indRxb, uint8_t identType);
    void disableRollover(void);
    void enableRollover(void);

    // error detection
    uint8_t getStatus(void);
    void clearErrors(void);
    uint8_t getErrorFlags(void);
    uint8_t getInterruptFlags(void);

private:
    // member types
#ifdef __unix
    ISpi_Master* _interface = new ISpi_Master; ///< SPI interface to be used
#endif // __unix

    // initialization and termination
    void parseIdentStd(uint8_t* data, uint32_t value);
    void parseIdentExt(uint8_t* data, uint32_t value);
    void setMode(uint8_t mode);

    // transmit
    int8_t setRts(uint8_t indTxb);

    // receive
    uint8_t getNumBytesReceived(uint8_t indRxb);
    uint8_t getDataLenCodeRx(uint8_t indRxb);
};

/**
  * @}
  */ // end of group devices

#endif // _MCP2515_H_
