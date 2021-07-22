#include <MFRC522.h>
#include <MFRC522Extended.h>
#include <SPI.h>
#include <EEPROM.h>
#include <rtc.h>  //Real Time Clock library for STM32Duino
#include <STM32RTC.h> //Real Time Clock library for STM32Duino
#include <Wire.h> //I2C library for rpi0w and stm32 communication

#define SS_PIN PB12        
#define RST_PIN PB5

MFRC522 rfid(SS_PIN, RST_PIN); 

MFRC522::MIFARE_Key key;

String rfid_id = "";

byte Data[] = {1, 4, 0, 0, 0, 28, 0, 0}; //Modbus RTU write request - GUN1
byte Data2[] = {2, 4, 0, 0, 0, 28, 0, 0}; //Modbus RTU write request - GUN2
byte Data3[] = {3, 4, 0, 0, 0, 28, 0, 0}; //Modbus RTU write request - GUN3

int flag = 0; //flag for sequencially raising modbus request for individual gun. 

int flag2 = 0;  //flag for sequencially sending the data to rpi0w from stm32 over i2c. Model Number - Gun1 data - Gun2 data - Gun3 data - Charging time for all three guns.  

int flag3 = 0;

int flag6 = 0, flag7 = 0, flag8 = 0;  //flags for sending relative charging time and relative T_kWh consumption for all three guns.

long int counter = 0, counter1 = 0, counter2 = 0; //counters to store the relative charging time for individual guns.

int lastSecondsData = 0, lastSecondsData1 = 0, lastSecondsData2 = 0;  //Storing the last Seconds data when charging started for individual guns. 
int lastMinuteData = 0, lastMinuteData1 = 0, lastMinuteData2 = 0; //Storing the last Minutes data when charging started for individual guns. 
int lastHoursData = 0, lastHoursData1 = 0, lastHoursData2 = 0;  //Storing the last Hours data when charging started for individual guns.
 
float lastData = 0, lastData1 = 0, lastData2 = 0; //Storing the last T_kWh data when charging started for individual guns.

byte receivedData[90]; //Serial read buffer

String sendDataHMI; //String to store the data to be sent to the HMI.

String sendDataRPI; //String to store the model number data to be sent to the RPI0W.
String sendDataRPI1, sendDataRPI2, sendDataRPI3;  //String to store the individual Gun data to be sent to the RPI0W.
String sendDataRPI4;  //String to store real time energy for all three guns to be sent to the RPI0W.
String sendDataRPI5;  //String to store the charging time to be send to the RPI0W.

unsigned short CRCCalculator(); //Function prototype for Cyclic Redundancy Check (CRC) function.

float dataformatter(byte data1, byte data2, byte data3, byte data4);  //Function prototype for formating the modbus data.

void HMIDataSend(); //Function prototype to send data from stm32 to HMI.

void EEPROM_READ_WRITE(); //EEPROM read and write the rfid's

int findPosition();

void RFIDFunction();
 
int SlaveID[] = {1,2,3};  //GUN Number

int GunStatus[] = {1,1,1};  //Gun Charging/Not Charging status

float T_kWh[3], IP_kWh[3], EP_kWh[3], T_kVArh[3], IP_kVArh[3], EP_kVArh[3], kVAh[3], kW[3], kVAr[3], kVA[3], V[3], I[3], PF[3], F[3]; //Meter parameters for all three guns

String ChargerModel = "P2P-EVSE-AC-01-033-03|"; //Charger Model Number

HardwareSerial HMI(PB7,PB6);  //UART1 for Modbus server for CMS data monitoring.

int readData; //Read i2c data from RPI0W to stm32.

STM32RTC& rtc = STM32RTC::getInstance();  //Instance of Real Time Clock.

/* Change these values to set the current initial date and time */
const byte seconds = 0;
const byte minutes = 29;
const byte hours = 14;


/* Monday 28th June 2021 */
const byte weekDay = 1;
const byte day = 28;
const byte month = 6;
const byte year = 21; 

//int addr = 0;

char rfid1[][8]={{'6','3','8','c','7','8','3','e'},{'7','3','4','3','c','b','a','#'},{'6','6','9','c','d','5','2','9'},{'3','3','8','e','2','2','3','e'},{'6','3','3','0','9','3','e','#'},{'8','6','3','1','a','3','2','9'},{'f','9','2','6','b','7','c','1'},{'4','a','2','0','e','a','8','0'},{'4','a','4','0','1','8','1','#'},{'3','a','e','6','f','f','8','0'}};
String rfidData[]={"","","","","","","","","",""};
String rfid_gun_association[][3] = {{"00000000", "0", "0"},{"00000000", "0", "0"},{"00000000", "0", "0"}};
int array2DPos = 3;
int rfidPos = 3;

void setup() {
  
  rfid.PCD_Init();

  rtc.begin(); // initialize RTC 24H format

  // Set the time
  if (!rtc.isTimeSet()){
    rtc.setHours(hours);
    rtc.setMinutes(minutes);
    rtc.setSeconds(seconds);
  }
  // Set the date
  rtc.setWeekDay(weekDay);
  rtc.setDay(day);
  rtc.setMonth(month);
  rtc.setYear(year);

  /*Relay ON/OFF output*/
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);
  pinMode(PC14, OUTPUT);
  digitalWrite(PC14, LOW);
  pinMode(PC15, OUTPUT);
  digitalWrite(PC15, LOW);

  /*Baudrate setting for UART1*/
  HMI.begin(19200);

  /*Setting for I2C1 remapped pins with slave address*/
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin(4);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  /*UART3 setting pins and baudrate etc.*/
  Serial1.setRx(PB11); //UART3 RX 
  Serial1.setTx(PB10); //UART3 TX 
  Serial1.begin(19200); 

  /*CRC calculate for all three energy meter modbusRTU requests*/
  unsigned short crcvalue = CRCCalculator();
  Data[6] = (byte)(crcvalue % 256);
  Data[7] = (byte)(crcvalue / 256);
  unsigned short crcvalue2 = CRCCalculator2();
  Data2[6] = (byte)(crcvalue2 % 256);
  Data2[7] = (byte)(crcvalue2 / 256);
  unsigned short crcvalue3 = CRCCalculator3();
  Data3[6] = (byte)(crcvalue3 % 256);
  Data3[7] = (byte)(crcvalue3 / 256);

  SPI.setMOSI(PB15);
  SPI.setMISO(PB14);
  SPI.setSCLK(PB13);
  SPI.begin();

  EEPROM_READ_WRITE();
}

void loop() {

  if(Serial1.available())
  {
    Serial1.readBytes(receivedData, Serial1.available());
    /*Receiving meter data on modbusRTU.*/
    if(receivedData[0] == 1 && receivedData[1] == 4 && receivedData[2] == Data[5]*2)
    {
      SlaveID[0] = 1;
      T_kWh[0] = dataformatter(receivedData[3], receivedData[4], receivedData[5], receivedData[6]);
      IP_kWh[0] = dataformatter(receivedData[7], receivedData[8], receivedData[9], receivedData[10]);
      EP_kWh[0] = dataformatter(receivedData[11], receivedData[12], receivedData[13], receivedData[14]);
      T_kVArh[0] = dataformatter(receivedData[15], receivedData[16], receivedData[17], receivedData[18]);
      IP_kVArh[0] = dataformatter(receivedData[19], receivedData[20], receivedData[21], receivedData[22]);
      EP_kVArh[0] = dataformatter(receivedData[23], receivedData[24], receivedData[25], receivedData[26]);
      kVAh[0] = dataformatter(receivedData[27], receivedData[28], receivedData[29], receivedData[30]);
      kW[0] = dataformatter(receivedData[31], receivedData[32], receivedData[33], receivedData[34]);
      kVAr[0] = dataformatter(receivedData[35], receivedData[36], receivedData[37], receivedData[38]);
      kVA[0] = dataformatter(receivedData[39], receivedData[40], receivedData[41], receivedData[42]);
      V[0] = dataformatter(receivedData[43], receivedData[44], receivedData[45], receivedData[46]);
      I[0] = dataformatter(receivedData[47], receivedData[48], receivedData[49], receivedData[50]);
      PF[0] = dataformatter(receivedData[51], receivedData[52], receivedData[53], receivedData[54]);
      F[0] = dataformatter(receivedData[55], receivedData[56], receivedData[57], receivedData[58]);
      HMIDataSend();
    }
    else if(receivedData[0] == 2 && receivedData[1] == 4 && receivedData[2] == Data[5]*2)
    {
      SlaveID[1] = 2;
      T_kWh[1] = dataformatter(receivedData[3], receivedData[4], receivedData[5], receivedData[6]);
      IP_kWh[1] = dataformatter(receivedData[7], receivedData[8], receivedData[9], receivedData[10]);
      EP_kWh[1] = dataformatter(receivedData[11], receivedData[12], receivedData[13], receivedData[14]);
      T_kVArh[1] = dataformatter(receivedData[15], receivedData[16], receivedData[17], receivedData[18]);
      IP_kVArh[1] = dataformatter(receivedData[19], receivedData[20], receivedData[21], receivedData[22]);
      EP_kVArh[1] = dataformatter(receivedData[23], receivedData[24], receivedData[25], receivedData[26]);
      kVAh[1] = dataformatter(receivedData[27], receivedData[28], receivedData[29], receivedData[30]);
      kW[1] = dataformatter(receivedData[31], receivedData[32], receivedData[33], receivedData[34]);
      kVAr[1] = dataformatter(receivedData[35], receivedData[36], receivedData[37], receivedData[38]);
      kVA[1] = dataformatter(receivedData[39], receivedData[40], receivedData[41], receivedData[42]);
      V[1] = dataformatter(receivedData[43], receivedData[44], receivedData[45], receivedData[46]);
      I[1] = dataformatter(receivedData[47], receivedData[48], receivedData[49], receivedData[50]);
      PF[1] = dataformatter(receivedData[51], receivedData[52], receivedData[53], receivedData[54]);
      F[1] = dataformatter(receivedData[55], receivedData[56], receivedData[57], receivedData[58]);
      HMIDataSend();
    }
    else if(receivedData[0] == 3 && receivedData[1] == 4 && receivedData[2] == Data[5]*2)
    {
      SlaveID[2] = 3;
      T_kWh[2] = dataformatter(receivedData[3], receivedData[4], receivedData[5], receivedData[6]);
      IP_kWh[2] = dataformatter(receivedData[7], receivedData[8], receivedData[9], receivedData[10]);
      EP_kWh[2] = dataformatter(receivedData[11], receivedData[12], receivedData[13], receivedData[14]);
      T_kVArh[2] = dataformatter(receivedData[15], receivedData[16], receivedData[17], receivedData[18]);
      IP_kVArh[2] = dataformatter(receivedData[19], receivedData[20], receivedData[21], receivedData[22]);
      EP_kVArh[2] = dataformatter(receivedData[23], receivedData[24], receivedData[25], receivedData[26]);
      kVAh[2] = dataformatter(receivedData[27], receivedData[28], receivedData[29], receivedData[30]);
      kW[2] = dataformatter(receivedData[31], receivedData[32], receivedData[33], receivedData[34]);
      kVAr[2] = dataformatter(receivedData[35], receivedData[36], receivedData[37], receivedData[38]);
      kVA[2] = dataformatter(receivedData[39], receivedData[40], receivedData[41], receivedData[42]);
      V[2] = dataformatter(receivedData[43], receivedData[44], receivedData[45], receivedData[46]);
      I[2] = dataformatter(receivedData[47], receivedData[48], receivedData[49], receivedData[50]);
      PF[2] = dataformatter(receivedData[51], receivedData[52], receivedData[53], receivedData[54]);
      F[2] = dataformatter(receivedData[55], receivedData[56], receivedData[57], receivedData[58]);
      HMIDataSend();
    }
    else 
    {
      /*Receiving data for ON/OFF relay from HMI*/
      if(receivedData[0] == 48)
      {
//        GunStatus[0] = 1;
        if (rfidPos != 3 && rfid_gun_association[rfidPos][1] == String(SlaveID[0]) && rfid_gun_association[rfidPos][2] == String(GunStatus[0]) && GunStatus[0] != 1){
          GunStatus[0] = 1;
          rfid_gun_association[rfidPos][2] = String(GunStatus[0]);
          rfidPos = 3;
          Serial1.write("F1IdleE");
          digitalWrite(PC13, LOW);
        }
//        Serial1.write("F1IdleE");
//        digitalWrite(PC13, LOW);
//        HMI.println(receivedData[0]);
      }
      else if(receivedData[0] == 49)
      {
//        GunStatus[0] = 2;
        array2DPos = findPosition();
        if (array2DPos != 3 && GunStatus[0] != 2){
          GunStatus[0] = 2;
          rfid_gun_association[array2DPos][1] = String(SlaveID[0]);
          rfid_gun_association[array2DPos][2] = String(GunStatus[0]);
          Serial1.write("F1RunningE");
          digitalWrite(PC13, HIGH);
        }
//        Serial1.write("F1RunningE");
//        digitalWrite(PC13, HIGH);
//        HMI.println(receivedData[0]);
      }
      else if(receivedData[0] == 50)
      {
//        GunStatus[1] = 1;
        if (rfidPos != 3 && rfid_gun_association[rfidPos][1] == String(SlaveID[1]) && rfid_gun_association[rfidPos][2] == String(GunStatus[1]) && GunStatus[1] != 1){
          GunStatus[1] = 1;
          rfid_gun_association[rfidPos][2] = String(GunStatus[1]);
          rfidPos = 3;
          Serial1.write("F2IdleE");
          digitalWrite(PC14, LOW);
        }
//        Serial1.write("F2IdleE");
//        digitalWrite(PC14, LOW);
//        HMI.println(receivedData[0]);
      }
      else if(receivedData[0] == 51)
      {
//        GunStatus[1] = 2;
        array2DPos = findPosition();
        if (array2DPos != 3 && GunStatus[1] != 2){
          GunStatus[1] = 2;
          rfid_gun_association[array2DPos][1] = String(SlaveID[1]);
          rfid_gun_association[array2DPos][2] = String(GunStatus[1]);
          Serial1.write("F2RunningE");
          digitalWrite(PC14, HIGH);
        }
//        Serial1.write("F2RunningE");
//        digitalWrite(PC14, HIGH);
//        HMI.println(receivedData[0]);
      }
      else if(receivedData[0] == 52)
      {
//        GunStatus[2] = 1;
        if (rfidPos != 3 && rfid_gun_association[rfidPos][1] == String(SlaveID[2]) && rfid_gun_association[rfidPos][2] == String(GunStatus[2]) && GunStatus[2] != 1){
          GunStatus[2] = 1;
          rfid_gun_association[rfidPos][2] = String(GunStatus[2]);
          rfidPos = 3;
          Serial1.write("F3IdleE");
          digitalWrite(PC15, LOW);
        }
//        Serial1.write("F3IdleE");
//        digitalWrite(PC15, LOW);
//        HMI.println(receivedData[0]);
      }
      else if(receivedData[0] == 53)
      {
//        GunStatus[2] = 2;
        array2DPos = findPosition();
        if (array2DPos != 3 && GunStatus[2] != 2){
          GunStatus[2] = 2;
          rfid_gun_association[array2DPos][1] = String(SlaveID[2]);
          rfid_gun_association[array2DPos][2] = String(GunStatus[2]);
          Serial1.write("F3RunningE");
          digitalWrite(PC15, HIGH);
        }
//        Serial1.write("F3RunningE");
//        digitalWrite(PC15, HIGH);
//        HMI.println(receivedData[0]);
      }
    }
    /*Charging time calculate using the GunStatus.*/
    if(GunStatus[0] == 2){
      if (flag6 == 0){
        lastData = T_kWh[0];
        lastSecondsData = rtc.getSeconds();
        lastMinuteData = rtc.getMinutes();
        lastHoursData = rtc.getHours();
        flag6 = 1;
      }
      counter = (rtc.getSeconds() - lastSecondsData) + 60*(rtc.getMinutes() - lastMinuteData) + 3600*(rtc.getHours() - lastHoursData);
    }else if(GunStatus[0] == 1){
      lastData = T_kWh[0];
      lastSecondsData = rtc.getSeconds();
      lastMinuteData = rtc.getMinutes();
      lastHoursData = rtc.getHours();
      flag6 = 0;
      counter = (rtc.getSeconds() - lastSecondsData) + 60*(rtc.getMinutes() - lastMinuteData) + 3600*(rtc.getHours() - lastHoursData);
    }
    if(GunStatus[1] == 2){
      if (flag7 == 0){
        lastData1 = T_kWh[1];
        lastSecondsData1 = rtc.getSeconds();
        lastMinuteData1 = rtc.getMinutes();
        lastHoursData1 = rtc.getHours();
        flag7 = 1;
      }
      counter1 = (rtc.getSeconds() - lastSecondsData1) + 60*(rtc.getMinutes() - lastMinuteData1) + 3600*(rtc.getHours() - lastHoursData1);
    }else if(GunStatus[1] == 1){
      lastData1 = T_kWh[1];
      lastSecondsData1 = rtc.getSeconds();
      lastMinuteData1 = rtc.getMinutes();
      lastHoursData1 = rtc.getHours();
      flag7 = 0;
      counter1 = (rtc.getSeconds() - lastSecondsData1) + 60*(rtc.getMinutes() - lastMinuteData1) + 3600*(rtc.getHours() - lastHoursData1);
    }
    if(GunStatus[2] == 2){
      if (flag8 == 0){
        lastData2 = T_kWh[2];
        lastSecondsData2 = rtc.getSeconds();
        lastMinuteData2 = rtc.getMinutes();
        lastHoursData2 = rtc.getHours();
        flag8 = 1;
      }
      counter2 = (rtc.getSeconds() - lastSecondsData2) + 60*(rtc.getMinutes() - lastMinuteData2) + 3600*(rtc.getHours() - lastHoursData2);
    }else if(GunStatus[2] == 1){
      lastData2 = T_kWh[2];
      lastSecondsData2 = rtc.getSeconds();
      lastMinuteData2 = rtc.getMinutes();
      lastHoursData2 = rtc.getHours();
      flag8 = 0;
      counter2 = (rtc.getSeconds() - lastSecondsData2) + 60*(rtc.getMinutes() - lastMinuteData2) + 3600*(rtc.getHours() - lastHoursData2);
    }
    delay(2000);
  }
  else
  {

    RFIDFunction();
    
    delay(200);
    
    if(flag == 0)
    {
      Serial1.write(Data, 8); //Writing ModbusRTU request - meter 1
      flag = 1;
    }
    else if(flag == 1)
    {
      Serial1.write(Data2, 8);  //Writing ModbusRTU request - meter 2
      flag = 2;
    }
    else if(flag == 2)
    {
      Serial1.write(Data3, 8);  //Writing ModbusRTU request - meter 3
      flag = 0;
    }
    delay(200);
  }
}

// Cyclic Redundancy Check(CRC) Calculator function - meter 1
unsigned short CRCCalculator() {
  unsigned short crc = 65535;
  int pos = 0;
  while(pos < (sizeof(Data) - 2)) {
    crc ^= Data[pos];
    int i = 8;
    while(i) {
      i -= 1;
      if(crc & 1){
        crc >>= 1;
        crc ^= 40961;
      }
      else{
        crc >>= 1;
      }
    }
    pos += 1;
  }
  return crc;
}

// Cyclic Redundancy Check(CRC) Calculator function - meter 2
unsigned short CRCCalculator2() {
  unsigned short crc = 65535;
  int pos = 0;
  while(pos < (sizeof(Data2) - 2)) {
    crc ^= Data2[pos];
    int i = 8;
    while(i) {
      i -= 1;
      if(crc & 1){
        crc >>= 1;
        crc ^= 40961;
      }
      else{
        crc >>= 1;
      }
    }
    pos += 1;
  }
  return crc;
}

// Cyclic Redundancy Check(CRC) Calculator function - meter 3
unsigned short CRCCalculator3() {
  unsigned short crc = 65535;
  int pos = 0;
  while(pos < (sizeof(Data3) - 2)) {
    crc ^= Data3[pos];
    int i = 8;
    while(i) {
      i -= 1;
      if(crc & 1){
        crc >>= 1;
        crc ^= 40961;
      }
      else{
        crc >>= 1;
      }
    }
    pos += 1;
  }
  return crc;
}

//Formats the received byte array into float values 
float dataformatter(byte data1, byte data2, byte data3, byte data4) {
  byte binaryvalue[32];
  for(int i=0; i<8; i++)
    binaryvalue[7-i] = bitRead(data3, i);
  for(int i=0; i<8; i++)
    binaryvalue[15-i] = bitRead(data4, i);
  for(int i=0; i<8; i++)
    binaryvalue[23-i] = bitRead(data1, i);
  for(int i=0; i<8; i++)
    binaryvalue[31-i] = bitRead(data2, i);
  byte signedvalue = binaryvalue[0];
  byte expo[8];
  for(int i=1; i<9; i++)
    expo[i-1] = binaryvalue[i];
  int exponential = 0;
  for(int i=7; i>=0; i--)
    exponential = exponential + pow(2,7-i) * expo[i];
  exponential = exponential - 127;
  byte mantisa[24];
  mantisa[0] = 1;
  for(int i=9; i<32; i++)
    mantisa[i-8] = binaryvalue[i];
  float value = 0;
  for(int i=0; i<24; i++){
    value = value + pow(2,exponential) * mantisa[i];
    exponential = exponential - 1;
  }
  if(signedvalue == 0)
    value = value * 1;
  else if(signedvalue == 1)
    value = value * (-1);
  return value;
}

void HMIDataSend()  {
  int i;
  sendDataHMI = "F";
  if (String(T_kWh[0]-lastData).length() == 4)
    sendDataHMI = sendDataHMI + "00" + String(T_kWh[0]-lastData) + "A";
  else if (String(T_kWh[0]-lastData).length() == 5)
    sendDataHMI = sendDataHMI + "0" + String(T_kWh[0]-lastData) + "A";
  else if (String(T_kWh[0]-lastData).length() == 6)
    sendDataHMI = sendDataHMI + String(T_kWh[0]-lastData) + "A";
  if (String(kW[0]).length() == 4)
    sendDataHMI = sendDataHMI + "0" + String(kW[0]) + "B";
  else if (String(kW[0]).length() == 5)
    sendDataHMI = sendDataHMI + String(kW[0]) + "B";
  if (String(V[0]).length() == 4)
    sendDataHMI = sendDataHMI + "00" + String(V[0]) + "C";
  else if (String(V[0]).length() == 5)
    sendDataHMI = sendDataHMI + "0" + String(V[0]) + "C";
  else if (String(V[0]).length() == 6)
    sendDataHMI = sendDataHMI + String(V[0]) + "C";
  if (String(I[0]).length() == 4)
    sendDataHMI = sendDataHMI + "0" + String(I[0]) + "D";
  else if (String(I[0]).length() == 5)
    sendDataHMI = sendDataHMI + String(I[0]) + "D";
  if (String(T_kWh[1]-lastData1).length() == 4)
    sendDataHMI = sendDataHMI + "00" + String(T_kWh[1]-lastData1) + "G";
  else if (String(T_kWh[1]-lastData1).length() == 5)
    sendDataHMI = sendDataHMI + "0" + String(T_kWh[1]-lastData1) + "G";
  else if (String(T_kWh[1]-lastData1).length() == 6)
    sendDataHMI = sendDataHMI + String(T_kWh[1]-lastData1) + "G";
  sendDataHMI = sendDataHMI + "000000000000" + "O";
  if (String(kW[1]).length() == 4)
    sendDataHMI = sendDataHMI + "0" + String(kW[1]) + "H";
  else if (String(kW[1]).length() == 5)
    sendDataHMI = sendDataHMI + String(kW[1]) + "H";
  if (String(V[1]).length() == 4)
    sendDataHMI = sendDataHMI + "00" + String(V[1]) + "I";
  else if (String(V[1]).length() == 5)
    sendDataHMI = sendDataHMI + "0" + String(V[1]) + "I";
  else if (String(V[1]).length() == 6)
    sendDataHMI = sendDataHMI + String(V[1]) + "I";
  if (String(I[1]).length() == 4)
    sendDataHMI = sendDataHMI + "0" + String(I[1]) + "J";
  else if (String(I[1]).length() == 5)
    sendDataHMI = sendDataHMI + String(I[1]) + "J";
  if (String(T_kWh[2]-lastData2).length() == 4)
    sendDataHMI = sendDataHMI + "00" + String(T_kWh[2]-lastData2) + "K";
  else if (String(T_kWh[2]-lastData2).length() == 5)
    sendDataHMI = sendDataHMI + "0" + String(T_kWh[2]-lastData2) + "K";
  else if (String(T_kWh[2]-lastData2).length() == 6)
    sendDataHMI = sendDataHMI + String(T_kWh[2]-lastData2) + "K";
  if (String(kW[2]).length() == 4)  
    sendDataHMI = sendDataHMI + "0" + String(kW[2]) + "L";
  else if (String(kW[2]).length() == 5)
    sendDataHMI = sendDataHMI + String(kW[2]) + "L";
  if (String(V[2]).length() == 4)
    sendDataHMI = sendDataHMI + "00" + String(V[2]) + "M";
  else if (String(V[2]).length() == 5)
    sendDataHMI = sendDataHMI + "0" + String(V[2]) + "M";
  else if (String(V[2]).length() == 6)
    sendDataHMI = sendDataHMI + String(V[2]) + "M";
  if (String(I[2]).length() == 4)
    sendDataHMI = sendDataHMI + "0" + String(I[2]) + "N";
  else if (String(I[2]).length() == 5)
    sendDataHMI = sendDataHMI + String(I[2]) + "N";
  sendDataHMI = sendDataHMI + "E";
  char HMIData[sendDataHMI.length()];
  for(i = 0; i < sendDataHMI.length(); i++){
    HMIData[i] = sendDataHMI.charAt(i);
  }
  Serial1.write(HMIData);
}

/*Send Data from stm32 to rpi0w over i2c*/
void requestEvent() { 
  char sendData_rpi0w[29];
  if (flag2 == 0){  //Charger Model Number
    flag2 = 1;
    sendDataRPI = ChargerModel;
    if (sendDataRPI.length() <= 29){
      for(byte i=0;i<sendDataRPI.length();i++){
        sendData_rpi0w[i] = sendDataRPI.charAt(i);
      }
      for(byte i=sendDataRPI.length();i<29;i++){
        sendData_rpi0w[i] = '#';
      }
//      HMI.println(flag2);
      Wire.write(sendData_rpi0w);
    }
  }
  else if (flag2 == 1){ //Gun 1 Data
    flag2 = 2;
    sendDataRPI1 = String(SlaveID[0]) + "|";
    sendDataRPI1 = sendDataRPI1 + String(GunStatus[0]) + "|";
    sendDataRPI1 = sendDataRPI1 + String(T_kWh[0]-lastData) + "|";
    sendDataRPI1 = sendDataRPI1 + String(kW[0]) + "|";
    sendDataRPI1 = sendDataRPI1 + String(V[0]) + "|";
    sendDataRPI1 = sendDataRPI1 + String(I[0]) + "|";
    if (sendDataRPI1.length() <= 29){
      for(byte i=0;i<sendDataRPI1.length();i++){
        sendData_rpi0w[i] = sendDataRPI1.charAt(i);
      }
      for(byte i=sendDataRPI1.length();i<29;i++){
        sendData_rpi0w[i] = '#';
      }
//      HMI.println(flag2);
      Wire.write(sendData_rpi0w);
    }
  }
  else if (flag2 == 2){ //Gun 2 Data 
    flag2 = 3;
    sendDataRPI2 = String(SlaveID[1]) + "|";
    sendDataRPI2 = sendDataRPI2 + String(GunStatus[1]) + "|";
    sendDataRPI2 = sendDataRPI2 + String(T_kWh[1]-lastData1) + "|";
    sendDataRPI2 = sendDataRPI2 + String(kW[1]) + "|";
    sendDataRPI2 = sendDataRPI2 + String(V[1]) + "|";
    sendDataRPI2 = sendDataRPI2 + String(I[1]) + "|";
    if (sendDataRPI2.length() <= 29){
      for(byte i=0;i<sendDataRPI2.length();i++){
        sendData_rpi0w[i] = sendDataRPI2.charAt(i);
      }
      for(byte i=sendDataRPI2.length();i<29;i++){
        sendData_rpi0w[i] = '#';
      }
//      HMI.println(flag2);
      Wire.write(sendData_rpi0w);
    } 
  }
  else if (flag2 == 3){ //Gun 3 Data
    flag2 = 4;
    sendDataRPI3 = String(SlaveID[2]) + "|";
    sendDataRPI3 = sendDataRPI3 + String(GunStatus[2]) + "|";
    sendDataRPI3 = sendDataRPI3 + String(T_kWh[2]-lastData2) + "|";
    sendDataRPI3 = sendDataRPI3 + String(kW[2]) + "|";
    sendDataRPI3 = sendDataRPI3 + String(V[2]) + "|";
    sendDataRPI3 = sendDataRPI3 + String(I[2]) + "|";
    if (sendDataRPI3.length() <= 29){
      for(byte i=0;i<sendDataRPI3.length();i++){
        sendData_rpi0w[i] = sendDataRPI3.charAt(i);
      }
      for(byte i=sendDataRPI3.length();i<29;i++){
        sendData_rpi0w[i] = '#';
      }
//      HMI.println(flag2);
      Wire.write(sendData_rpi0w);
    } 
  }
  else if (flag2 == 4){ //Real Time Energy for OCPP.
    flag2 = 5;
    sendDataRPI4 = String(T_kWh[0]) + "|";
    sendDataRPI4 = sendDataRPI4 + String(T_kWh[1]) + "|";
    sendDataRPI4 = sendDataRPI4 + String(T_kWh[2]) + "|";
    if (sendDataRPI4.length() <= 29){
      for(byte i=0;i<sendDataRPI4.length();i++){
        sendData_rpi0w[i] = sendDataRPI4.charAt(i);
      }
      for(byte i=sendDataRPI4.length();i<29;i++){
        sendData_rpi0w[i] = '#';
      }
//      HMI.println(flag2);
      Wire.write(sendData_rpi0w);
    } 
  }
  else if (flag2 == 5){ //Charging Time data
    flag2 = 0;
    sendDataRPI5 = String(counter) + "|"; 
    sendDataRPI5 = sendDataRPI5 + String(counter1) + "|"; 
    sendDataRPI5 = sendDataRPI5 + String(counter2) + "|"; 
    if (sendDataRPI5.length() <= 29){
      for(byte i=0;i<sendDataRPI5.length();i++){
        sendData_rpi0w[i] = sendDataRPI5.charAt(i);
      }
      for(byte i=sendDataRPI5.length();i<29;i++){
        sendData_rpi0w[i] = '#';
      }
//      HMI.println(flag2);
      Wire.write(sendData_rpi0w);
    } 
  }
}

/*Receive relay ON/OFF from app over i2c*/
void receiveEvent(int howMany)  {
  readData = Wire.read();
  if(readData == 11){
    if (array2DPos == 3){
      GunStatus[0] = 1;
      Serial1.write("F1IdleE");
      digitalWrite(PC13, LOW);
    }  
  }
  else if(readData == 12){
    if (flag3 != 1){
      GunStatus[0] = 2;
      Serial1.write("F1RunningE");
      digitalWrite(PC13, HIGH);
    }
  }
  else if(readData == 21){
    if (array2DPos == 3){
      GunStatus[1] = 1;
      Serial1.write("F2IdleE");
      digitalWrite(PC14, LOW);
    }
  }
  else if(readData == 22){
    if (flag3 != 1){
      GunStatus[1] = 2;
      Serial1.write("F2RunningE");
      digitalWrite(PC14, HIGH);
    }
  }
  else if(readData == 31){
    if (array2DPos == 3){
      GunStatus[2] = 1;
      Serial1.write("F3IdleE");
      digitalWrite(PC15, LOW);
    }
  }
  else if(readData == 32){
    if (flag3 != 1){
      GunStatus[2] = 2;
      Serial1.write("F3RunningE");
      digitalWrite(PC15, HIGH);
    }
  }
}

void EEPROM_READ_WRITE(){
  char value[sizeof(rfid1)/sizeof(rfid1[0])][sizeof(rfid1[0])];
  for (int i = 0; i<sizeof(rfid1)/sizeof(rfid1[0]); i++){
    for (int j = 0; j<sizeof(rfid1[i]); j++){
      EEPROM.write((sizeof(rfid1[i])*i + j), rfid1[i][j]);  
    }
  }
  for (int i = 0; i<sizeof(rfid1)/sizeof(rfid1[0]); i++){
    for (int j = 0; j<sizeof(rfid1[i]); j++){
      value[i][j] = char(EEPROM.read(sizeof(rfid1[i])*i + j));   
    }
  }
  for (int i=0; i<sizeof(rfid1)/sizeof(rfid1[0]); i++){
    for (int j = 0; j<sizeof(rfid1[i]); j++){
      rfidData[i] = rfidData[i] + String(value[i][j]);  
    }
  }
}

int findPosition(){
  int pos=3;
  for (int i=0; i<3; i++){
    if(rfid_id == rfid_gun_association[i][0])
      pos = i;
  }
  return pos;
}

void RFIDFunction(){
  for(int i=0; i<sizeof(rfid1)/sizeof(rfid1[0]); i++){
    HMI.print(rfidData[i]);
    HMI.print("|");
  }
  HMI.println();
  for(int i=0; i<3; i++){
    HMI.print(rfid_gun_association[i][0]);
    HMI.print("|");
  }

  String content="";
  if (rfid.PICC_IsNewCardPresent()){
    rfid.PICC_ReadCardSerial();
    MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
    for (byte i = 0; i < rfid.uid.size; i++)
    {
      content.concat(String(rfid.uid.uidByte[i], HEX));
    }
    rfid_id = content;
    if (rfid_id.length() == 7)
      rfid_id = rfid_id + "#";
    HMI.println(rfid_id);
    for (int i=0; i<sizeof(rfid1)/sizeof(rfid1[0]); i++){
      if (rfid_id == rfidData[i]){
        flag3 = 1;
        for (int j=0; j<3; j++){
          if (rfid_id == rfid_gun_association[j][0]){
            rfidPos = j;
            flag3 = 2;
            break;
          }
          else if(rfid_id != rfid_gun_association[j][0])
          {
            if (rfid_gun_association[j][0] == "00000000")
              flag3 = 3;
          }
        }
        break;
      }
    }
    if (flag3 == 0){
      HMI.println("FAuthFailedE");
      Serial1.write("FAuthFailedE");  
    }
    else if (flag3 == 1){
      HMI.println("FAllGunsBusyE");
      Serial1.write("FAllGunsBusyE");
      flag3 = 0;
    }
    else if (flag3 == 2){
      HMI.println("FStopAuthE");
      Serial1.write("FStopAuthE");
      rfid_gun_association[rfidPos][0] = "00000000";
      flag3 = 0;
    }
    else if (flag3 == 3){
      for (int j=0; j<3; j++){
        if (rfid_gun_association[j][0] == "00000000"){
          HMI.println("FStartAuthE");
          Serial1.write("FStartAuthE");
          rfid_gun_association[j][0] = rfid_id;
          break;
        }
      }
      flag3 = 0;
    }
  }
  else{
    HMI.println("waiting");
    rfid_id = "waiting";
  }  
}
