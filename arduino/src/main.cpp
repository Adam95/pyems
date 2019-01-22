#include <Arduino.h>
#include <NefitSerial.h>
#include <avr/pgmspace.h>
#include <stdint.h>

// DEBUG: local debuging
#define xDEBUG

// DEBUGDATA: send debug informations to master
#define xDEBUGDATA

#define MAX_IRRIGATION_TIME 720 // time in seconds divisible by 4

#define MASTER_UART_RATE 9600 // Computer interface UART rate
#define EMS_BUS_UART_RATE 9700 // EMS Bus - UART Interface rate

#define OUT_EMS_MESSAGE_LENGTH 6
#define OUT_EMS_BUFFER_SIZE OUT_EMS_MESSAGE_LENGTH + 1
#define IN_EMS_BUFFER_SIZE 256
#define EMS_MAX_WAIT_TIME 1000
#define MASTER_MAX_WAIT_TIME 1000
#define RETRY_FACTOR 4
#define MAX_EMS_READ 32
#define EMS_DATAGRAM_OVERHEAD 6
#define MAX_MASTER_READ 32

#define IRRIGATION_TIMER_VALUE MAX_IRRIGATION_TIME/4

#define ON LOW
#define OFF HIGH

#define emsData 0xF4
#define gwResponse 0xA0

// RELAY1_COMMON disconnect power to all irrigation valves: HIGH(OFF)-power connected, LOW(ON)-power disconnected
#define RELAY1_COMMON 2

// RELAY2-RELAY6: irrigation circuits 1-5
#define RELAY2 3
#define RELAY3 4
#define RELAY4 5
#define RELAY5 6
#define RELAY6 7

#ifdef DEBUG
  #define DPRINT(item) nefitSerial.print(item)
  #define DPRINTLN(item) nefitSerial.println(item)
  #define DPRINTVALUE(item1, item2) nefitSerial.print(item1);nefitSerial.print(": ");nefitSerial.println(item2)
  #define DPRINTHEX(aBuffer, aLength) for (int i = 0; i < aLength; i++) { char hex[10]; sprintf(hex, "%02x", aBuffer[i]); nefitSerial.print(hex); }
#else
  #define DPRINT(item)
  #define DPRINTLN(item)
  #define DPRINTVALUE(item1, item2)
  #define DPRINTHEX(item, length)
#endif

#ifdef DEBUGDATA
  #define sendMasterDebug(aData, aLength, aID) sendMasterDebugData(aData, aLength, aID)
#else
  #define sendMasterDebug(aData, aLength, aID)
#endif

#define sendMasterBuffer(buffer, length) nefitSerial.write(buffer, length)

enum DeviceID {
  UBA = 0x08,
  BC_10 = 0x09,
  PC = 0x0B,
  RC_35 = 0x10,
  WM_10 = 0x11,
  RC_20 = 0x17,
  MM_10 = 0x21
};

byte emsBuffer[MAX_EMS_READ+4];
byte masterBuffer[MAX_MASTER_READ+4];

byte pollAddress = 0;
byte myAddress[1] = { DeviceID::PC };

#define MASTER_RESPONSE_BUFFER_SIZE 40
byte masterResponse[MASTER_RESPONSE_BUFFER_SIZE];

int irrigationTimer;

void sendEMSBuffer(byte *outEMSBuffer, int len, boolean checkEcho)
{
  for (byte j = 0; j < len; j++)
  {
    nefitSerial3.write(outEMSBuffer[j]);
    delay(3);
  }
  nefitSerial3.writeEOF();
  
  delay(2);
  nefitSerial3.flush();

  if (!checkEcho) return;

  // try to eat last char from echo (the best solution from all proven bad solutions)
  unsigned long timeout = millis() + EMS_MAX_WAIT_TIME;

  // wait for a data and eat zero bytes
  while (millis() < timeout)
  {
    if (!nefitSerial3.available()) continue;
    if ((byte)nefitSerial3.peek() != 0) break;
    nefitSerial3.read();
  }

  // read byte is echo of last sent byte
  if (nefitSerial3.available() &&  (byte)nefitSerial3.peek() == outEMSBuffer[len-1])
  {
    nefitSerial3.read();

    while (millis() < timeout)
    {
      if (!nefitSerial3.available()) continue;
      if ((byte)nefitSerial3.peek() != 0) break;
      nefitSerial3.read();
    }
  }
}

byte crcCalculator(byte *aBuffer, int len)
{
  byte i, crc = 0x0;
  byte d;
  for (i = 0; i < len; i++)
  {
    d = 0;
    if (crc & 0x80)
    {
      crc ^= 12;
      d = 1;
    }
    crc = (crc << 1) & 0xfe;
    crc |= d;
    crc ^= aBuffer[i];
  }
  return crc;
}

boolean crcCheckOK(byte *aBuffer, int len)
{
  return len < 1 ? false : crcCalculator(aBuffer, len) == aBuffer[len];
}

void sendMasterDebugData(byte *aData, byte aLength, byte aID )
{ 
  masterResponse[0] = aLength+2;
  masterResponse[1] = gwResponse+aID;
  masterResponse[2] = aLength;
  for (int i=0; i<aLength && (i+3)<MASTER_RESPONSE_BUFFER_SIZE; i++)
  {
    masterResponse[i+3] = aData[i];
  }
  delay(1);
  #ifndef DEBUG
    sendMasterBuffer(masterResponse, aLength+3);
  #else
    DPRINTHEX(masterResponse, aLength+3);
    DPRINTLN("");
  #endif
}

int readBytesMaster(byte *inMasterBuffer, byte len, unsigned long masterTimeout)
{
  // 0: length of data including CRC, 1-n: data, n+1: CRC 
  byte ptr = 0;

  // while there is available data and no timeout, skip the 0's in the buffer
  while (nefitSerial.available() && (millis() < masterTimeout))
  {
    if ((byte)nefitSerial.peek() == 0) nefitSerial.read(); else break;
  }

  // read data until data available, max bytes are read
  while (!nefitSerial.frameError() && (ptr < len) && (millis() < masterTimeout))
  {
    if (nefitSerial.available())
    {
      inMasterBuffer[ptr] = nefitSerial.read();
      ptr++;
    }
  }

  // flush the possible pending information left to be read (garbage)
  nefitSerial.flush();

  sendMasterDebug(inMasterBuffer, ptr, 1);

  // null message or length is wrong (length:data:CRC)
  if ((ptr < 3) || ((ptr-1)!=*inMasterBuffer)) return -1;

  return crcCheckOK(inMasterBuffer+1, ptr-2) ? ptr-2 : 0;
}

int readBytesEMS(byte *inEMSBuffer, byte len, unsigned long eMSTimeout)
{
  int ptr = 0;

  // while there is available data and no timeout, skip the 0's in the buffer
  while (nefitSerial3.available() && (millis() < eMSTimeout))
  {
    // if the first byte to be read is 0x00 (break)
    if ((byte)nefitSerial3.peek() == 0) nefitSerial3.read(); else break;
  }

  // read data until frame-error, max bytes are read
  while (!nefitSerial3.frameError() && (ptr < len) && (millis() < eMSTimeout))
  {
    if (nefitSerial3.available())
    {
      inEMSBuffer[ptr] = nefitSerial3.read();
      ptr++;
    }
  }

  // flush the possible pending information left to be read (garbage)
  nefitSerial3.flush();

  // return the number of bytes read
  return ptr;
}

void sendEMSAck()
{ 
  delay(1);
  sendEMSBuffer(myAddress, 1, false); // send own address and a break
}

// aValue: 0-success, 1-not success, 2-invalid request, 3-CRC error, 4-data error
// aValue: 5-request not confirmed, 6-request not verified, 7-Send timeout, 8-Send verification timeout
// aValue: 9-Unspecified error
void sendMasterResponse(byte aValue, byte aData = 0)
{ 
  masterResponse[0] = 5;
  masterResponse[1] = gwResponse;
  masterResponse[2] = aValue;
  masterResponse[3] = aData;
  masterResponse[4] = crcCalculator(masterResponse+1, 2);
  masterResponse[5] = 0;
  delay(1);
  sendMasterBuffer(masterResponse, masterResponse[0]+1);
}

void sendMasterFakeEMSResponse(byte aData)
{ 
  masterResponse[0] = 8;
  masterResponse[1] = emsData;
  masterResponse[2] = 0; // sender
  masterResponse[3] = DeviceID::PC; // receiver
  masterResponse[4] = 0xE0; // frame type - relay status - fake EMS response
  masterResponse[5] = 0; // offset
  masterResponse[6] = aData;
  masterResponse[7] = crcCalculator(masterResponse+1, 2);
  masterResponse[8] = 0;
  delay(1);
  sendMasterBuffer(masterResponse, masterResponse[0]+1);
}

#define sendMasterSuccess() sendMasterResponse(0)
#define sendMasterNotSuccess() sendMasterResponse(1)
#define sendMasterInvalidRequest() sendMasterResponse(2)
#define sendMasterCRCError() sendMasterResponse(3)
#define sendMasterDataError() sendMasterResponse(4)
#define sendMasterRequestNotConfirmed() sendMasterResponse(5)
#define sendMasterRequestNotVerified() sendMasterResponse(6)
#define sendMasterSendTimeout() sendMasterResponse(7)
#define sendMasterSendVerificationTimeout() sendMasterResponse(8)
#define sendMasterUnspecifiedError() sendMasterResponse(9)

boolean sendEMSRequest(byte *outEMSBuffer)
{
  // calculate the CRC value in the sIdxth position of the buffer
  outEMSBuffer[5] = crcCalculator(outEMSBuffer, OUT_EMS_MESSAGE_LENGTH-1);

  sendMasterDebug(outEMSBuffer, OUT_EMS_MESSAGE_LENGTH, 0x10);

  // last polled address (wait until this is DeviceID::PC)
  byte pollAddress = 0;

  // flush the EMS Serial Stream by clearing (if any) current buffer contents
  nefitSerial3.flush();

  // watchdog (maximum polling waiting time)
  unsigned long eMSTimeout = millis() + EMS_MAX_WAIT_TIME * RETRY_FACTOR;

  byte auxBuffer[MAX_EMS_READ];

  // wait until being polled by the Bus Master
  // Loop unitl PollAddress is the device ID that Calduino simulates (PC)
  while ((pollAddress & 0x7F) != DeviceID::PC)
  {
    // end the operation without sending the request if the timeout expires
    if (millis() > eMSTimeout) return false;
    
    // Read next datagram without limits of size (do not force 2 bytes read, in case UBA sends a monitor)
    // Assign the first  read byte to pollAddress only if two bytes are read (bus master polls:
    // pollAddress + Break) 
    if (readBytesEMS(auxBuffer, MAX_EMS_READ, eMSTimeout) == 2)
    {
      pollAddress = auxBuffer[0];
    }
  }

  // wait two milliseconds and send the 6 bytes buffer
  delay(2);
  sendEMSBuffer(outEMSBuffer, OUT_EMS_MESSAGE_LENGTH, true);

  return true;
}

#define emsSuccess 0
#define emsNotConfirmed 1
#define emsNotVerified 2
#define emsSendTimeout 3
#define emsSendVerificationTimeout 4
#define emsReadTimeout 5
#define emsUnspecifiedError 6

// send request for data to EMS a recieve standar EMS datagram (UBAMonitorSlow)
byte requestEMSCommand(byte aDestinationID, byte aMessageID, byte aOffset, byte aLength)
{
  DPRINTLN("requestEMSCommand");

  byte operationStatus = emsUnspecifiedError;

  byte outEMSBuffer[OUT_EMS_BUFFER_SIZE+2];
  byte inEMSBuffer[IN_EMS_BUFFER_SIZE+2];
  byte *inEMSBufferPtr = inEMSBuffer+2;
  byte readLength = 0;
  byte gOriginalOffset = aOffset;

  unsigned long timeout;

  // First load outEMSBuffer with corresponding values for a SET Command
  // first position is the transmitterID. Ox0b is the ComputerID (our address)
  outEMSBuffer[0] = DeviceID::PC;

  // second position is destinationID. 1st bit=1 in case of getEMS
  outEMSBuffer[1] = aDestinationID | 0x80;

  // third position is the messageID
  outEMSBuffer[2] = aMessageID;

  // fourth position is the offset in the buffer.
  outEMSBuffer[3] = -1;

  // while there are still bytes pending to be read and the last EMS Command was correctly executed (offset has been increased)
  while (aLength > 0 && (aOffset > (char)outEMSBuffer[3]))
  {
    // fourth position is the offset in the buffer. If the message is split in different commands,
    // the offset contains the byte required from the EMS Datagram
    outEMSBuffer[3] = aOffset;

    // fifth position is the data to send
    outEMSBuffer[4] = (aLength > (MAX_EMS_READ - EMS_DATAGRAM_OVERHEAD) ? (MAX_EMS_READ - EMS_DATAGRAM_OVERHEAD) : aLength);;

    // once the buffer is loaded, send the request.
    if (sendEMSRequest(outEMSBuffer))
    {
      // check if the requested query is answered in the next EMSMaxWaitTime milliseconds
      timeout = millis() + EMS_MAX_WAIT_TIME;

      // wait until timeout or some new data in the EMS-Bus
      while ((millis() < timeout) && (!nefitSerial3.available())) {}

      // if there is data to be read
      if (nefitSerial3.available())
      {
        byte auxBuffer[outEMSBuffer[4] + EMS_DATAGRAM_OVERHEAD];

        int ptr = readBytesEMS(auxBuffer, outEMSBuffer[4] + EMS_DATAGRAM_OVERHEAD, timeout);

        sendMasterDebug(auxBuffer, ptr, 2);

        // if more than 4 bytes are read (datagram received)
        // check if the CRC of the information received is correct and the operation type returned corresponds with the one requested
        if ((ptr > 4) && crcCheckOK(auxBuffer, ptr-2) && (auxBuffer[2] == aMessageID))
        {
          readLength += ptr;
          // copy 1st four bytes (TransmitterID, DestinationID, MessageType, Offset)
          memcpy(inEMSBufferPtr,auxBuffer,4);

          // copy the bytes read from auxiliarBuffer to inEMSBufferPtr taking into account the internal offset (reconstruct the EMS Datagram)
          memcpy(inEMSBufferPtr+4+auxBuffer[3],auxBuffer+4, outEMSBuffer[4]);

          // update the length and offset values to prepare the read of the next block
          aLength -= (aLength >(MAX_EMS_READ - EMS_DATAGRAM_OVERHEAD) ? (MAX_EMS_READ - EMS_DATAGRAM_OVERHEAD) : aLength);
          aOffset += (MAX_EMS_READ - EMS_DATAGRAM_OVERHEAD);
          operationStatus = emsSuccess;
        } else operationStatus = emsNotConfirmed;
      } else operationStatus = emsReadTimeout;
    } else operationStatus = emsSendTimeout;
  }

  if (operationStatus == emsSuccess)
  {
    inEMSBuffer[0] = readLength+1;
    inEMSBuffer[1] = emsData;
    inEMSBuffer[5] = gOriginalOffset; // restore original offset
    DPRINT("data: "); DPRINTHEX(inEMSBuffer, readLength); DPRINTLN("");
    #ifndef DEBUG
      sendMasterBuffer(inEMSBuffer, readLength+2);
    #endif
    DPRINTLN("requestEMSCommand: request confirmed");
  } else { DPRINT("requestEMSCommand: not success: "); DPRINTLN(operationStatus); }
  return operationStatus;
}

// read one byte data from EMS
byte getEMSCommand(byte aDestinationID, byte aMessageID, byte aOffset, byte *aResult)
{
  DPRINTLN("getEMSCommand");

  byte operationStatus = emsUnspecifiedError;
  byte outEMSBuffer[OUT_EMS_BUFFER_SIZE+2];
  byte inEMSBuffer[OUT_EMS_BUFFER_SIZE+2];
  byte *inEMSBufferPtr = inEMSBuffer+2;

  unsigned long timeout;

  // First load outEMSBuffer with corresponding values for a SET Command
  // first position is the transmitterID. Ox0b is the ComputerID (our address)
  outEMSBuffer[0] = DeviceID::PC;

  // second position is destinationID. 1st bit=1 in case of getEMS otherwise writeEMS
  outEMSBuffer[1] = aDestinationID | 0x80;

  // third position is the messageID
  outEMSBuffer[2] = aMessageID;

  // fourth position is the offset in the buffer.
  outEMSBuffer[3] = aOffset;

  // fifth position is the length of the data requested.
  outEMSBuffer[4] = 1;

  // once the buffer is loaded, send the request.
  if (sendEMSRequest(outEMSBuffer))
  {
    // check if the requested query is answered in the next EMSMaxWaitTime milliseconds
    timeout = millis() + EMS_MAX_WAIT_TIME;

    // wait until timeout or some new data in the EMS-Bus
    while ((millis() < timeout) && (!nefitSerial3.available())) {}

    // if there is data to be read
    if (nefitSerial3.available())
    {
      // read the information sent
      int ptr = readBytesEMS(inEMSBufferPtr, OUT_EMS_BUFFER_SIZE, timeout);

      sendMasterDebug(inEMSBufferPtr, ptr, 3);

      // if more than 4 bytes are read (datagram received)
      // the CRC of the information received is correct
      // and the operation type returned corresponds with the one requested
      if ((ptr > 4) && crcCheckOK(inEMSBufferPtr, ptr-2) && (inEMSBufferPtr[2] == aMessageID))
      {
        // send EMS response to SerialMaster
        inEMSBuffer[0] = (byte) ptr+1;
        inEMSBuffer[1] = emsData;
        DPRINT("data: "); DPRINTHEX(inEMSBufferPtr, ptr); DPRINTLN("");
        #ifndef DEBUG
          sendMasterBuffer(inEMSBuffer, ptr+2);
        #endif

        operationStatus = emsSuccess;
        *aResult = inEMSBufferPtr[4];
      } else { operationStatus = emsNotConfirmed; DPRINTLN("getEMSCommand: data not received"); }
    } else { operationStatus = emsNotConfirmed; DPRINTLN("getEMSCommand: data not received"); }
  } else { operationStatus = emsSendTimeout; DPRINTLN("getEMSCommand: send data request timeout"); }
  return operationStatus;
}

// write EMS data and verify result by redaig od data from EMS
byte setEMSCommand(byte aDestinationID, byte aMessageID, byte aOffset, byte aData, byte aVerify)
{
  DPRINTLN("setEMSCommand");

  byte operationStatus = emsUnspecifiedError;
  byte outEMSBuffer[OUT_EMS_BUFFER_SIZE+2];
  byte inEMSBuffer[OUT_EMS_BUFFER_SIZE+2];
  byte *inEMSBufferPtr = inEMSBuffer+2;

  unsigned long timeout;

  // First load outEMSBuffer with corresponding values for a SET Command
  // first position is the transmitterID. Ox0b is the ComputerID (our address)
  outEMSBuffer[0] = DeviceID::PC;

  // second position is destinationID. 1st bit=1 in case of getEMS otherwise writeEMS
  outEMSBuffer[1] = aDestinationID;

  // third position is the messageID
  outEMSBuffer[2] = aMessageID;

  // fourth position is the offset in the buffer.
  outEMSBuffer[3] = aOffset;

  // fifth position is the data to send
  outEMSBuffer[4] = aData;

  // once the buffer is loaded, send the request.
  if (sendEMSRequest(outEMSBuffer))
  {
    // check if the requested query is answered in the next EMSMaxWaitTime milliseconds
    timeout = millis() + EMS_MAX_WAIT_TIME;

    // wait until timeout or some new data in the EMS-Bus
    while ((millis() < timeout) && (!nefitSerial3.available())) {}

    // if there is data to be read
    if (nefitSerial3.available())
    {
      // search confirmation datagram
      int ptr = readBytesEMS(inEMSBufferPtr, 1, timeout);

      sendMasterDebug(inEMSBufferPtr, ptr, 2);
      
      // if the answer received is 0x01, the value has been correctly sent
      if (ptr != 1 || inEMSBufferPtr[0] != 0x01) return emsNotConfirmed;
    } else { DPRINTLN("setEMSCommand: not confirmed"); return emsNotConfirmed; }
  } else { DPRINTLN("setEMSCommand: send request timeout"); return emsSendTimeout; }

  DPRINTLN("setEMSCommand: confirmed");

  // return if verification is not required
  if (!aVerify) return emsSuccess;
  
  // Second Load outEMSBuffer with corresponding values for a GET Command and check if value received matches
  // second position is destinationID. Masked with 0x80 as a read command
  outEMSBuffer[1] = aDestinationID | 0x80;

  // fifth position is the length of the data requested.
  outEMSBuffer[4] = 1;

  // once the buffer is loaded, send the request.
  if (sendEMSRequest(outEMSBuffer))
  {
    // check if the requested query is answered in the next EMSMaxWaitTime milliseconds
    timeout = millis() + EMS_MAX_WAIT_TIME;

    // wait until timeout or some new data in the EMS-Bus
    while ((millis() < timeout) && (!nefitSerial3.available())) {}

    // if there is data to be read
    if (nefitSerial3.available())
    {
      // read the information sent
      int ptr = readBytesEMS(inEMSBufferPtr, OUT_EMS_BUFFER_SIZE, timeout);

      sendMasterDebug(inEMSBufferPtr, ptr, 3);

      // if more than 4 bytes are read (datagram received)
      // the CRC of the information received is correct
      // and the operation type returned corresponds with the one requested
      if ((ptr > 4) && crcCheckOK(inEMSBufferPtr, ptr-2) && (inEMSBufferPtr[2] == aMessageID))
      {
        // send EMS response to SerialMaster
        inEMSBuffer[0] = (byte) ptr+1;
        inEMSBuffer[1] = emsData;
        DPRINT("data: "); DPRINTHEX(inEMSBufferPtr, ptr); DPRINTLN("");
        #ifndef DEBUG
          sendMasterBuffer(inEMSBuffer, ptr+2);
        #endif
        
        // check if the data received corresponds with the change requested
        operationStatus = ((aData == (byte)inEMSBufferPtr[4])) ? emsSuccess : emsNotVerified;

        #ifdef DEBUG
          if (operationStatus) DPRINTLN("setEMSCommand: data verification failed"); else DPRINTLN("setEMSCommand: data verified");
        #endif
      } else { operationStatus = emsNotVerified; DPRINTLN("setEMSCommand: data not verified"); }
    } else { operationStatus = emsNotVerified; DPRINTLN("setEMSCommand: data not verified"); }
  } else { operationStatus = emsSendVerificationTimeout; DPRINTLN("setEMSCommand: send data verification request timeout"); }
  return operationStatus;
}

// aRelay=0 means all irrigation circuits
void setRelay(byte aRelay, boolean aState)
{
  if (aRelay == 0)
  {
    digitalWrite(RELAY2, aState ? ON : OFF);
    digitalWrite(RELAY3, aState ? ON : OFF);
    digitalWrite(RELAY4, aState ? ON : OFF);
    digitalWrite(RELAY5, aState ? ON : OFF);
    digitalWrite(RELAY6, aState ? ON : OFF);
  } else
  {
    byte gRelay;
    switch (aRelay)
    {
      case 1: gRelay = RELAY1_COMMON; break;
      case 2: gRelay = RELAY2; break;
      case 3: gRelay = RELAY3; break;
      case 4: gRelay = RELAY4; break;
      case 5: gRelay = RELAY5; break;
      case 6: gRelay = RELAY6; break;
      default: return;
    }
    digitalWrite(gRelay, aState ? ON : OFF);
  }

  // reset timer when switching on some irrigation circuit
  if (aState && aRelay != 1) irrigationTimer = IRRIGATION_TIMER_VALUE;
}

// return: 0-OFF, 1-ON
// aRelay=0 means all irrigation circuits
// return 0-all irrigation circuits are OFF, 1-some irrigation curcuit is ON
// aRelay= 255 means status of all irrigation circuits
// return bit: 0-all irrigation relays (or), 1-irrigation relay 1, 2-irrigation realy 2, etc.
byte getRelayState(byte aRelay)
{
  byte gRelay;
  byte gResult;

  if (aRelay==0)
  {
    return ((0!=(*portOutputRegister( digitalPinToPort(RELAY2) ) & digitalPinToBitMask(RELAY2))) &&
            (0!=(*portOutputRegister( digitalPinToPort(RELAY3) ) & digitalPinToBitMask(RELAY3))) &&
            (0!=(*portOutputRegister( digitalPinToPort(RELAY4) ) & digitalPinToBitMask(RELAY4))) &&
            (0!=(*portOutputRegister( digitalPinToPort(RELAY5) ) & digitalPinToBitMask(RELAY5))) &&
            (0!=(*portOutputRegister( digitalPinToPort(RELAY6) ) & digitalPinToBitMask(RELAY6)))) ? 0 : 1;
  }

  if (aRelay==255)
  {
    gResult = ((0!=(*portOutputRegister( digitalPinToPort(RELAY2) ) & digitalPinToBitMask(RELAY2)))? 0 : 1)<<1;
    gResult |= ((0!=(*portOutputRegister( digitalPinToPort(RELAY3) ) & digitalPinToBitMask(RELAY3)))? 0 : 1)<<2;
    gResult |= ((0!=(*portOutputRegister( digitalPinToPort(RELAY4) ) & digitalPinToBitMask(RELAY4)))? 0 : 1)<<3;
    gResult |= ((0!=(*portOutputRegister( digitalPinToPort(RELAY5) ) & digitalPinToBitMask(RELAY5)))? 0 : 1)<<4;
    gResult |= ((0!=(*portOutputRegister( digitalPinToPort(RELAY6) ) & digitalPinToBitMask(RELAY6)))? 0 : 1)<<5;
    if (gResult != 0) gResult |= 1;
    return gResult;
  }

  switch (aRelay)
  {
    case 1: gRelay = RELAY1_COMMON; break;
    case 2: gRelay = RELAY2; break;
    case 3: gRelay = RELAY3; break;
    case 4: gRelay = RELAY4; break;
    case 5: gRelay = RELAY5; break;
    case 6: gRelay = RELAY6; break;
    default: return 0x00;
  }
  return (0!=(*portOutputRegister( digitalPinToPort(gRelay) ) & digitalPinToBitMask(gRelay))) ? 0 : 1;
}

ISR(TIMER1_COMPA_vect) // timer compare interrupt service routine
{
  if (--irrigationTimer <= 0)
  {
    digitalWrite(RELAY2, OFF);
    digitalWrite(RELAY3, OFF);
    digitalWrite(RELAY4, OFF);
    digitalWrite(RELAY5, OFF);
    digitalWrite(RELAY6, OFF);
    irrigationTimer = IRRIGATION_TIMER_VALUE;
  }
}

// -----------------------------------------------------------------------------------------------------------------

void serveEMS()
{
  int ptr = 0;
  byte *emsPtr = emsBuffer+2;

  if (nefitSerial3.available())
  {
    // data from emsBuffer[2], emsBuffer[0] is reserved for data length, emsBuffer[1] is reserved data type
    ptr = readBytesEMS(emsPtr, MAX_EMS_READ, millis() + EMS_MAX_WAIT_TIME);
    if (ptr == 2)
    {
      pollAddress = emsPtr[0];
      //pollAddress = 0;
      if (pollAddress==(DeviceID::PC | 0x80)) // we are being polled,send acknowledge
      {                                       
        DPRINTLN("I am polled!!!");
        //byte result = setEMSCommand(DeviceID::RC_35, 0x3d, 7, 1);
        //DPRINTVALUE("setNighMode",result);
        sendEMSAck(); // this causes to be polled more often
      }
    } else if (ptr > 4)
    {
      if (crcCheckOK(emsPtr, ptr-2))
      {
        emsBuffer[0] = (byte) ptr+1;
        emsBuffer[1] = emsData;
        DPRINT("data: "); DPRINTHEX(emsPtr, ptr); DPRINTLN("");
        #ifndef DEBUG
          sendMasterBuffer(emsBuffer, ptr+2);
        #endif
      }
    }
  }
}

void serveMaster()
{
  int ptr = 0;
  byte result = 0;
  byte verify = 0;

  if (nefitSerial.available())
  {
    ptr = readBytesMaster(masterBuffer, MAX_MASTER_READ, millis() + MASTER_MAX_WAIT_TIME);
    if (ptr > 0)
    {
      switch (masterBuffer[1])
      {
        case 0x10: // set EMS with result verification
          verify = 1;
        case 0x11: // set EMS
          if (ptr != 5) { sendMasterInvalidRequest(); break; }
          switch (setEMSCommand(masterBuffer[2], masterBuffer[3], masterBuffer[4], masterBuffer[5], verify))
          {
            case emsSuccess: sendMasterSuccess(); break;
            case emsNotConfirmed: sendMasterRequestNotConfirmed(); break;
            case emsNotVerified: sendMasterRequestNotVerified(); break;
            case emsSendTimeout: sendMasterSendTimeout(); break;
            case emsSendVerificationTimeout: sendMasterSendVerificationTimeout(); break;
            case emsUnspecifiedError: sendMasterUnspecifiedError(); break;
            default: sendMasterNotSuccess(); break;
          }
          break; 
        case 0x20: // request EMS
          if (ptr != 5) { sendMasterInvalidRequest(); break; }
          switch (requestEMSCommand(masterBuffer[2], masterBuffer[3], masterBuffer[4], masterBuffer[5]))
          {
            case emsSuccess: sendMasterSuccess(); break;
            case emsNotConfirmed: sendMasterRequestNotConfirmed(); break;
            case emsNotVerified: sendMasterRequestNotVerified(); break;
            case emsSendTimeout: sendMasterSendTimeout(); break;
            case emsSendVerificationTimeout: sendMasterSendVerificationTimeout(); break;
            case emsUnspecifiedError: sendMasterUnspecifiedError(); break;
            default: sendMasterNotSuccess(); break;
          }
          break; 
        case 0x30: // get EMS
          if (ptr != 4) { sendMasterInvalidRequest(); break; }
          switch (getEMSCommand(masterBuffer[2], masterBuffer[3], masterBuffer[4], &result))
          {
            case emsSuccess: sendMasterResponse(0,result); break;
            case emsNotConfirmed: sendMasterRequestNotConfirmed(); break;
            case emsNotVerified: sendMasterRequestNotVerified(); break;
            case emsSendTimeout: sendMasterSendTimeout(); break;
            case emsSendVerificationTimeout: sendMasterSendVerificationTimeout(); break;
            case emsUnspecifiedError: sendMasterUnspecifiedError(); break;
            default: sendMasterNotSuccess(); break;
          }
          break; 
        case 0x40: // request relays status
          if (ptr !=1) { sendMasterInvalidRequest(); break; }
          sendMasterFakeEMSResponse(getRelayState(255));
          sendMasterSuccess();
          break;
        case 0x70: // get relay state: 0-all, n-relay number
          if (ptr !=2) { sendMasterInvalidRequest(); break; }
          sendMasterResponse(0,getRelayState(masterBuffer[2]));
          break;
        case 0x80: // set relay off: 0-all, n-relay number
          if (ptr !=2) { sendMasterInvalidRequest(); break; }
          setRelay(masterBuffer[2],false);
          sendMasterFakeEMSResponse(getRelayState(255));
          sendMasterSuccess();
          break;
        case 0x81: // set relay on: 0-all, n-relay number
          if (ptr != 2) { sendMasterInvalidRequest(); break; }
          setRelay(masterBuffer[2],true);
          sendMasterFakeEMSResponse(getRelayState(255));
          sendMasterSuccess();
          break;
        default: sendMasterInvalidRequest(); break;
      }
    } else if (ptr==0) // CRC error
    {
      sendMasterCRCError();
    } else // data corrupted
    {
      sendMasterDataError();
    }
  }
}

// -----------------------------------------------------------------------------------------------------------------

void setup()
{
  nefitSerial3.begin(EMS_BUS_UART_RATE);
  nefitSerial.begin(MASTER_UART_RATE);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(RELAY1_COMMON, OUTPUT);
  digitalWrite(RELAY1_COMMON, OFF);

  pinMode(RELAY2, OUTPUT);
  digitalWrite(RELAY2, OFF);

  pinMode(RELAY3, OUTPUT);
  digitalWrite(RELAY3, OFF);

  pinMode(RELAY4, OUTPUT);
  digitalWrite(RELAY4, OFF);

  pinMode(RELAY5, OUTPUT);
  digitalWrite(RELAY5, OFF);

  pinMode(RELAY6, OUTPUT);
  digitalWrite(RELAY6, OFF);

  // initialize Timer1
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 62500; // compare match register 16MHz/1024/0.25Hz tick/4s
  TCCR1B |= (1 << WGM12); // CTC mode interrupt
  TCCR1B |= (1 << CS12) | (1 << CS10); // 1024 prescaler
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  irrigationTimer = IRRIGATION_TIMER_VALUE;
  interrupts(); // enable all interrupts
}

void loop()
{
  serveEMS();
  serveMaster();
}
