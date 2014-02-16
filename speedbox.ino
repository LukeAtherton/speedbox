#include <SoftwareSerial.h>
#include <EEPROM.h>

//libraries for LCD
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
 
//UI Constants
#define BTN_DOWN 2
#define BTN_UP 3
#define BTN_MODE 5

#define UP_LED 10
#define DOWN_LED 11

#define UP_CHAR ((uint8_t)24)
#define DOWN_CHAR ((uint8_t)25)
#define UPDOWN_CHAR ((uint8_t)18)

#define OLED_RESET 4
//display is on SDA(blue)->A4 and SLC(green)->A5
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// From antmessage.h
#define UCHAR unsigned char
 
#define MESG_TX_SYNC                      ((UCHAR)0xA4)
#define MESG_ASSIGN_CHANNEL_ID            ((UCHAR)0x42)
#define MESG_CHANNEL_MESG_PERIOD_ID       ((UCHAR)0x43)
#define MESG_CHANNEL_SEARCH_TIMEOUT_ID    ((UCHAR)0x44)
#define MESG_CHANNEL_RADIO_FREQ_ID        ((UCHAR)0x45)
#define MESG_NETWORK_KEY_ID               ((UCHAR)0x46)
#define MESG_SYSTEM_RESET_ID              ((UCHAR)0x4A)
#define MESG_OPEN_CHANNEL_ID              ((UCHAR)0x4B)
#define MESG_CHANNEL_ID_ID                ((UCHAR)0x51)
 
#define MESG_RESPONSE_EVENT_ID            ((UCHAR)0x40)
#define MESG_BROADCAST_DATA_ID            ((UCHAR)0x4E)
#define MESG_CAPABILITIES_ID              ((UCHAR)0x54)
 
#define DEBUG 1
 
// Garmin cadence sensor uses deviceType = 121
// the tranmission period is supposedly 4.05 MHz or period = 8085
#define ANT_CHAN           0
#define ANT_NET            0    // public network
#define ANT_TIMEOUT       12    // 12 * 2.5 = 30 seconds
#define ANT_DEVICETYPE   121    // bit 7 = 0 pairing requiest bits 6..0 = 121 for cadence/speed sensor
#define ANT_FREQ          57    // Garmin radio frequency
#define ANT_PERIOD      8086    // Garmin search period
#define ANT_NETWORKKEY {0xb9, 0xa5, 0x21, 0xfb, 0xbd, 0x72, 0xc3, 0x45}
 
// For Software Serial
#define ANT_CTS             12
#define ANT_TXD              8
#define ANT_RXD              7
#define ANT_BAUD          4800 // Baud for ANT chip

SoftwareSerial antSerial(ANT_RXD, ANT_TXD);
 
#define PACKETREADTIMEOUT  100
#define MAXPACKETLEN        80

#define BSC_PRECISION     ((unsigned long)1000)
#define MAX_USHORT        0xFFFF
 
// Global Variables
int rxBufCnt = 0;
unsigned char rxBuf[MAXPACKETLEN];
unsigned long lastRevCount = 0;
unsigned long lastEventTime = 0;
unsigned long lastCadence = 0;
unsigned long cadence = 0;
unsigned long cadenceFraction = 0;
unsigned char antNetKey[] = ANT_NETWORKKEY;

unsigned long cadenceEventTime = 0;
unsigned long cadenceRevolutionCount = 0;
unsigned long accumCadence = 0; //initialize at zero

long packetCount = 0;
 
enum
{
  errDefault,
  errPacketSizeExceeded,
  errChecksumError,
  errMissingSync
};
 
//UI Variables
int upperRangeValue = 85;
int lowerRangeValue = 75;
int dampeningValue = 1;

// Define the number of samples to keep track of.  The higher the number,
// the more the readings will be smoothed, but the slower the output will
// respond to the input.  Using a constant rather than a normal variable lets
// use this value to determine the size of the readings array.
const int maxDampening = 10;

int readings[maxDampening];      // the readings from the analog input
int readingsIndex = 0;           // the index of the current reading
int cadenceReadingsTotal = 0;    // the running total
int cadenceReadingsAverage = 0;  // the average

typedef enum { AUTO, UPPER, LOWER, DAMPEN, NONE } UIMode;

UIMode currentMode = NONE;
bool isActive = true;
int changeDirection = 0;
const float maxEditModeWaitTime = 5000; //time that cursor will display on screen without intput
float lastInputTime = maxEditModeWaitTime;
bool isEditModeActive = false;

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...");
 
  //LCD Setup
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)

  //init the pushbutton pins as input:
  pinMode(BTN_DOWN, INPUT);     
  pinMode(BTN_UP, INPUT);     
  pinMode(BTN_MODE, INPUT); 

  //init ant chip serial
  pinMode(INPUT, ANT_CTS);
  pinMode(ANT_RXD, INPUT);
  pinMode(ANT_TXD, OUTPUT);
  
  //init gear shift outputs
  pinMode(UP_LED, OUTPUT);
  pinMode(DOWN_LED, OUTPUT);
  
  digitalWrite(UP_LED, LOW);
  digitalWrite(DOWN_LED, LOW);
  
  //load saved values from eeprom
  Serial.println("Load settings from eeprom...");
  lowerRangeValue = getSavedValue(0, 75);
  Serial.print("Lower Bound: ");
  Serial.println(lowerRangeValue);

  upperRangeValue = getSavedValue(1, 85);
  Serial.print("Upper Bound: ");
  Serial.println(upperRangeValue);

  dampeningValue = getSavedValue(2, 10);
  Serial.print("Dampening: ");
  Serial.println(dampeningValue);

  isActive = getSavedValue(3, false);
  Serial.print("Auto Change Active: ");
  Serial.println(isActive);

  Serial.println("...Settings loaded!");

  //init all cadence readings to 0: 
  resetCadenceReadings();
 
  antSerial.begin(ANT_BAUD);
   
  configureAntChip();

  Serial.println("...Setup done!");
}
 
void loop()
{  
  refreshUI();
  checkForInput();
  handleAntMessages();
}

void errorHandler(int errIn)
{
#ifdef DEBUG
  Serial.println();
  Serial.print("Error: ");
  Serial.println(errIn);
#endif

  display.clearDisplay();
  display.setRotation(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.setTextSize(1);

  display.print("Err: ");
  display.write(errIn);

  display.display(); 

  while (true) {};
}
 
unsigned char writeByte(unsigned char out, unsigned char chksum)
{ 
#ifdef DEBUG
  Serial.print(out, HEX);
  Serial.print(" ");
#endif 
  antSerial.write(out);
  chksum ^= out;
  return chksum;
}
 
void sendPacket(unsigned msgId, unsigned char argCnt, ...)
{
  va_list arg;
  va_start (arg, argCnt);
  unsigned char byteOut;
  unsigned char chksum = 0;
  int cnt = 0;
 
#ifdef DEBUG
  Serial.print("TX: ");
#endif
 
  chksum = writeByte(MESG_TX_SYNC, chksum); // send sync
  chksum = writeByte(argCnt, chksum);       // send length
  chksum = writeByte(msgId, chksum);        // send message id
   
  // send data
  for (cnt=1; cnt <= argCnt; cnt++) {
    byteOut = va_arg(arg, unsigned int);
    chksum = writeByte(byteOut, chksum);
  }
  va_end(arg);
   
  writeByte(chksum,chksum);                 // send checksum  
#ifdef DEBUG
  Serial.println();
#endif
}
 
void printPacket(unsigned char * packet)
{
  int cnt = 0;
  while (cnt < packet[1]+4)
  {
    Serial.print(packet[cnt++], DEC);
    Serial.print  (" ");
  }
  Serial.println();
}
 
int readPacket(unsigned char *packet, int packetSize, int readTimeout)
{
  unsigned char byteIn;
  unsigned char chksum = 0;
   
  long timeoutExit = millis() + readTimeout;
 
  while (timeoutExit > millis())
  {
    if (antSerial.available() > 0)
    {
      byteIn = antSerial.read();
      timeoutExit = millis() + readTimeout;
      if ((byteIn == MESG_TX_SYNC) && (rxBufCnt == 0))
      {
        rxBuf[rxBufCnt++] = byteIn;
        chksum = byteIn;
      } 
      else if ((rxBufCnt == 0) && (byteIn != MESG_TX_SYNC))
      {
        errorHandler(errMissingSync);
        return -1;
      }
      else if (rxBufCnt == 1)
      {
        rxBuf[rxBufCnt++] = byteIn;       // second byte will be size
        chksum ^= byteIn;
      }
      else if (rxBufCnt < rxBuf[1]+3) // read rest of data taking into account sync, size, and checksum that are each 1 byte
      {
        rxBuf[rxBufCnt++] = byteIn;
        chksum ^= byteIn;
      }
      else
      {
        rxBuf[rxBufCnt++] = byteIn;
        if (rxBufCnt > packetSize)
        {
          errorHandler(errPacketSizeExceeded);
          return -1;
        }
        else
        {
          memcpy(packet, &rxBuf, rxBufCnt); // should be a complete packet. copy data to packet variable, check checksum and return
          packetCount++;
          if (chksum != packet[rxBufCnt-1])
          {
            errorHandler(errChecksumError);
            rxBufCnt = 0;
            return -1;
          }
          else
          {
            rxBufCnt = 0;
            return 1;
          }
        }
      }
    }
  }
  return 0;
}
 
int checkReturn()
{
  byte packet[MAXPACKETLEN];
  int packetsRead;
 
  packetsRead = readPacket(packet, MAXPACKETLEN, PACKETREADTIMEOUT);
 
  // Data <sync> <len> <msg id> <channel> <msg id being responded to> <msg code> <chksum>
  // <sync> always 0xa4
  // <msg id> always 0x40 denoting a channel response / event
  // <msg code? success is 0.  See page 84 of ANT MPaU for other codes
 
  if (packetsRead > 0) {
    Serial.print("RX: ");
    printPacket(packet);
  }
 
  return packetsRead;
}

int getSavedValue(int eepromPosition, int defaultValue)
{
  int savedValue = EEPROM.read(eepromPosition);

  if(savedValue == 255)
  {
    savedValue = defaultValue;
  }

  return savedValue;
}

bool getSavedValue(int eepromPosition, bool defaultValue)
{
  bool savedValue = EEPROM.read(eepromPosition);

  if(savedValue == 255)
  {
    savedValue = defaultValue;
  }

  return savedValue;
}

void printHeader(const char * title)
{
  Serial.print(millis());
  Serial.print(" ");
  Serial.print(packetCount);
  Serial.print(" - ");
  Serial.print(title);
}

bool checkInRange(float value)
{
  if(value > 0)
  {
    if(value < lowerRangeValue)
    {
      //change down
      digitalWrite(DOWN_LED, HIGH);
      digitalWrite(UP_LED, LOW);

      changeDirection = -1;
      cadenceReadingsTotal = 0;
     
      for (int thisReading = 0; thisReading < dampeningValue; thisReading++)
      {
        readings[thisReading] = upperRangeValue -1;  
        cadenceReadingsTotal += upperRangeValue -1;
      }
    } 
    else if(value > upperRangeValue)
    {
      //change up
      digitalWrite(UP_LED, HIGH);
      digitalWrite(DOWN_LED, LOW);

      changeDirection = 1;
      cadenceReadingsTotal = 0;
     
      for (int thisReading = 0; thisReading < dampeningValue; thisReading++)
      {
        readings[thisReading] = lowerRangeValue + 1;  
        cadenceReadingsTotal += lowerRangeValue + 1;
      }
    } 
    else 
    {
      digitalWrite(DOWN_LED, LOW);
      digitalWrite(UP_LED, LOW);
      changeDirection = 0;
    }
  }
}

void configureAntChip()
{
  Serial.println("Ant Chip Config Starting");
 
  // Reset
  sendPacket(MESG_SYSTEM_RESET_ID, 1, 0);
  delay(600);
   
  // Flush read buffer
  while (antSerial.available() > 0)
  {
    antSerial.read();
  }
   
  // Assign Channel
  //   Channel: 0
  //   Channel Type: for Receive Channel
  //   Network Number: 0 for Public Network
  sendPacket(MESG_ASSIGN_CHANNEL_ID, 3, ANT_CHAN, 0, ANT_NET);  
  if (checkReturn() == 0) errorHandler(errDefault);
   
  // Set Channel ID
  //   Channel Number: 0
  //   Device Number LSB: 0 for a slave to match any device
  //   Device Number MSB: 0 for a slave to match any device
  //   Device Type: bit 7 0 for pairing request bit 6..0 for device type
  //   Transmission Type: 0 to match any transmission type
  sendPacket(MESG_CHANNEL_ID_ID, 5, ANT_CHAN, 0, 0, ANT_DEVICETYPE, 0);
  if (checkReturn() == 0) errorHandler(errDefault);
 
  // Set Network Key
  //   Network Number
  //   Key
  sendPacket(MESG_NETWORK_KEY_ID, 9, ANT_NET, antNetKey[0], antNetKey[1], antNetKey[2], antNetKey[3], antNetKey[4], antNetKey[5], antNetKey[6], antNetKey[7]);
  if (checkReturn() == 0) errorHandler(errDefault);
 
  // Set Channel Search Timeout
  //   Channel
  //   Timeout: time for timeout in 2.5 sec increments
  sendPacket(MESG_CHANNEL_SEARCH_TIMEOUT_ID, 2, ANT_CHAN, ANT_TIMEOUT);
  if (checkReturn() == 0) errorHandler(errDefault);
 
  //ANT_send(1+2, MESG_CHANNEL_RADIO_FREQ_ID, CHAN0, FREQ);
  // Set Channel RF Frequency
  //   Channel
  //   Frequency = 2400 MHz + (FREQ * 1 MHz) (See page 59 of ANT MPaU) 0x39 = 2457 MHz
  sendPacket(MESG_CHANNEL_RADIO_FREQ_ID, 2, ANT_CHAN, ANT_FREQ);
  if (checkReturn() == 0) errorHandler(errDefault);
 
  // Set Channel Period
  sendPacket(MESG_CHANNEL_MESG_PERIOD_ID, 3, ANT_CHAN, (ANT_PERIOD & 0x00FF), ((ANT_PERIOD & 0xFF00) >> 8));
  if (checkReturn() == 0) errorHandler(errDefault);
 
  //Open Channel
  sendPacket(MESG_OPEN_CHANNEL_ID, 1, ANT_CHAN);
  if (checkReturn() == 0) errorHandler(errDefault);
 
  Serial.println("Ant Chip Config Done");
}

void resetCadenceReadings()
{
  cadenceReadingsTotal = 0;
  for (int thisReading = 0; thisReading < dampeningValue; thisReading++)
  {
    readings[thisReading] = lowerRangeValue + ((upperRangeValue - lowerRangeValue) / 2);
    cadenceReadingsTotal += lowerRangeValue + ((upperRangeValue - lowerRangeValue) / 2);
  }
}

void writeAverageCadence(int avgCadence)
{
  if(avgCadence < 100)
  {
    display.setTextSize(1);
    display.print(" ");
    display.setTextSize(2);
  }

  if(avgCadence < 10)
  {
    display.print(" ");
  }

  if(avgCadence > 299)
  {
    display.setTextSize(1);
    display.print("^");
    display.setTextSize(2);
    display.print("^^");
  }
  else
  {
    if(avgCadence > 99)
    {
      if(avgCadence < 199)
      {
        display.setTextSize(1);
        display.print("1");
        display.setTextSize(2);
        if(avgCadence < 110)
        {
          display.print("0");
        }
        display.print(avgCadence - 100);
      }
      else
      {
        display.setTextSize(1);
        display.print("2");
        display.setTextSize(2);
        if(avgCadence < 210)
        {
          display.print("0");
        }
        display.print(avgCadence - 200);
      }
    }
    else
    {
      display.print(avgCadence);
    }
  }
  writeLineSpacer(1);
}

void writeLineSpacer(int size)
{
  display.setTextSize(size);
  display.println();
}

void writeCursorIfSelected(bool isSelected)
{
  if(isEditModeActive && isSelected)
  {
    display.write((uint8_t)16);
  }
  else
  {
    display.print(" ");
  }
}

void writeAutoSetting(bool isAuto, bool isSelected)
{
  display.setTextSize(1);
  display.print("auto");
  display.write(UPDOWN_CHAR);

  writeCursorIfSelected(isSelected);

  if(isAuto)
  {
    display.print(" on ");
  }
  else
  {
    display.print("off ");
  }
  writeLineSpacer(1);
}

void writeNumericUIComponent(int value, bool isSelected)
{
  writeCursorIfSelected(isSelected);

  display.setTextSize(2); //write padding

  if(value < 10)
  {
    display.print(" "); //write padding
  }

  display.print(value);
  writeLineSpacer(1);
}

void refreshUI()
{
  // clear the screen and buffer
  display.clearDisplay();
  display.setRotation(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.setTextSize(1);

  writeAverageCadence(cadenceReadingsAverage);
  writeAutoSetting(isActive, currentMode == AUTO);
  writeNumericUIComponent(upperRangeValue, currentMode == UPPER);
  writeNumericUIComponent(lowerRangeValue, currentMode == LOWER);
  writeNumericUIComponent(dampeningValue, currentMode == DAMPEN);

  display.display(); 
}

void writeStateToSerial()
{
  Serial.print("| dampening: ");
  Serial.print(dampeningValue);
  Serial.print(" | total: ");
  Serial.print(cadenceReadingsTotal);
  Serial.print(" | average: ");
  Serial.print(cadenceReadingsTotal / dampeningValue);
  Serial.print(" | readings: ");

  for (int thisReading = 0; thisReading < dampeningValue; thisReading++)
  {
    Serial.print(thisReading);
    Serial.print(":");
    Serial.print(readings[thisReading]);  
    Serial.print(" ");
  }
  Serial.println("|");
}

void changeMode()
{
  switch(currentMode) 
  {
    case AUTO:
      currentMode = UPPER;
      break;
    case UPPER:
      currentMode = LOWER;
      break;
    case LOWER:
      currentMode = DAMPEN;
      break;
    case DAMPEN:
      currentMode = AUTO;
      break;
    default :
      currentMode = AUTO;
      break;
  }
}

void enterEditMode()
{
  lastInputTime = millis();
  isEditModeActive = true;
}

void handleModeButton()
{
  if (digitalRead(BTN_MODE) == HIGH)
  {
    if(isEditModeActive)
    {
      changeMode();
    } 
    else 
    {
      enterEditMode();
    }
  }
}

void checkForInput()
{
  handleModeButton();
  bool isUpButtonPressed = digitalRead(BTN_UP) == HIGH;
  bool isDownButtonPressed = digitalRead(BTN_DOWN) == HIGH;
  
  if(isUpButtonPressed || isDownButtonPressed)
  {
    if(isEditModeActive)
    {
      switch(currentMode) 
      {
        case AUTO:
          if(isActive)
          {
            isActive = false;
            changeDirection = 0;
            digitalWrite(UP_LED, LOW);
            digitalWrite(DOWN_LED, LOW);
          }
          else
          {
            isActive = true;
          }
          EEPROM.write(3, isActive);
          break;

        case LOWER:
          if(isUpButtonPressed)
          {
            if(lowerRangeValue < 90 && lowerRangeValue < upperRangeValue - 5)
              lowerRangeValue++;
          }
          else
          {
            if(lowerRangeValue > 1)
              lowerRangeValue--;
          }
          EEPROM.write(0, lowerRangeValue);
          break;

        case UPPER:
          if(isUpButtonPressed)
          {
            if(upperRangeValue < 99)
              upperRangeValue++;
          }
          else
          {
            if(upperRangeValue > 10 && lowerRangeValue < upperRangeValue - 5)
              upperRangeValue--;
          }
          EEPROM.write(1, upperRangeValue);
          break;
          
        case DAMPEN:
          if(isUpButtonPressed)
          {
            if(dampeningValue < maxDampening)
              dampeningValue++;
          }
          else
          {
            if(dampeningValue > 1)
              dampeningValue--;
          }
          EEPROM.write(2, dampeningValue);
          resetCadenceReadings();
          break;

        default:
          //something is wrong if we end up here...
          break;
      }

      lastInputTime = millis();
    }
    else
    {
      currentMode = AUTO;
      enterEditMode();
    }

#ifdef DEBUG
    writeStateToSerial();
#endif
  }

  if(lastInputTime < millis() - maxEditModeWaitTime)
  {
    isEditModeActive = false;
  }
}

void handleAntMessages()
{
  byte packet[MAXPACKETLEN];
  int packetsRead;
  unsigned char msgId, msgSize;
  unsigned char *msgData;
 
  packetsRead = readPacket(packet, MAXPACKETLEN, PACKETREADTIMEOUT);
  if (packetsRead > 0)
  {
    msgId = packet[2];
    msgSize = packet[1];
    msgData = &packet[3];
     
    switch (msgId) 
    {
      case MESG_RESPONSE_EVENT_ID:
        printHeader("MESG_RESPONSE_EVENT_ID: ");
        printPacket(packet);
        break;
   
      case MESG_CAPABILITIES_ID:
        printHeader("MESG_CAPABILITIES_ID: ");
        printPacket(packet);
        break;
   
      case MESG_BROADCAST_DATA_ID:
        //printHeader("MESG_BROADCAST_DATA_ID: ");
        //printPacket(packet);
        
        if(packet[1] == 9)
        {
          //combine LSB and MSB to get value
          cadenceEventTime = (unsigned short)packet[4];
          cadenceEventTime |= (unsigned short)packet[5] << 8;
          
          if(cadenceEventTime != lastEventTime)
          {
            Serial.print("Cadence Message Event Time: ");
            Serial.println(cadenceEventTime, DEC);
          }
          
          cadenceRevolutionCount = (unsigned short)packet[6];
          cadenceRevolutionCount |= (unsigned short)packet[7] << 8;
          
          if(cadenceRevolutionCount != lastRevCount)
          {
            Serial.print("Cadence Revolution Count: ");
            Serial.println(cadenceRevolutionCount, DEC);
          }

          unsigned long finalCadence;
          unsigned short deltaTime;

          //rollover protection
          deltaTime = (cadenceEventTime - lastEventTime) & MAX_USHORT;

          if (deltaTime > 0) //divide by zero
          {
            //rollover protection
            finalCadence = (unsigned long)((cadenceRevolutionCount - lastRevCount) & MAX_USHORT);
            finalCadence *= (unsigned long)(60); //60 s/min for numerator

            cadenceFraction = (unsigned short)((((finalCadence * 1024) % (unsigned long)deltaTime) * BSC_PRECISION) / deltaTime);
            finalCadence = (unsigned long)(finalCadence * (unsigned long)1024 / deltaTime); //1024/((1/1024)s) in the denominator --> RPM
                                                                                            //...split up from s/min due to ULONG size limit
            cadence = finalCadence;

            accumCadence += (unsigned long)((lastRevCount - cadenceRevolutionCount) & MAX_USHORT);

            lastCadence = cadence;
            lastEventTime = cadenceEventTime;             
            lastRevCount = cadenceRevolutionCount;
                
            Serial.print("Instantaneous cadence: ");
            Serial.print(cadence);
            Serial.print(".");
            Serial.print(cadenceFraction);
            Serial.println(" RPM");

            Serial.print("Accumulated cadence: ");
            Serial.println((unsigned short)((accumCadence >> 16) & MAX_USHORT));
            Serial.println((unsigned short)(accumCadence & MAX_USHORT)); //display limited by 16-bit CPU
                
            if(cadence < 1000)
            {
              // subtract the last reading:
              cadenceReadingsTotal = cadenceReadingsTotal - readings[readingsIndex];         
              // read from the sensor:  
              readings[readingsIndex] = lastCadence; 
              // add the reading to the total:
              cadenceReadingsTotal = cadenceReadingsTotal + readings[readingsIndex];   
              // advance to the next position in the array:  
              readingsIndex++;           
              // if we're at the end of the array...
              if (readingsIndex >= dampeningValue)
              {
                // ...wrap around to the beginning: 
                readingsIndex = 0;
              }          
              // calculate the average:
              cadenceReadingsAverage = cadenceReadingsTotal / dampeningValue;  
                
              if(isActive){
                checkInRange(cadenceReadingsAverage);
              }
#ifdef DEBUG
              writeStateToSerial();
#endif
            }
          }
        }
        break;
   
      default:
        printHeader("MESG_ID_UNKNOWN: ");
        printPacket(packet);
        break;
    }
  }
}
