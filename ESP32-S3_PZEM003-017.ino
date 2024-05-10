/*
  โปรแกรมนี้ใช้ไมโครคอนโทรลเลอร์รุ่น ESP32-S3 ซึ่งมีขา RX, TX คือ 18 และ 17 ตามลับดับ
  จะต้องมีขา Tx Enable ให้กับ Max485 โปรแกรมนี้ใช้ขา 16
*/

//#define PZEM017   //uncomment บรรทัดนี้ ถ้าใช้ PZEM017
//#define rstENE    //uncomment บรรทัดนี้ ถ้าจะ reset kWh ให้เป็นค่า 0

static uint8_t pzemSlaveAddr = 0x01; // ถ้า DeviceID ของ PZEM ไม่มีค่าเป็น 1 เปลี่ยนส่วนนี้

#ifdef PZEM017
// ตั้งค่า shunt -->> 0x0000-100A, 0x0001-50A, 0x0002-200A, 0x0003-300A
static uint16_t NewshuntAddr = 0x0001; //ตั้งค่า Rshunt
#endif

#include <ModbusMaster.h>
float V, kWh, I, P;

#define MAX485_DE 16 // DE RE ต่อขาเดียวกันเลย แล้วต่อเข้าขา GPIO ของ ESP32
/*-----------------------------------------------------------------------*/
#define RXD1  (18)
#define TXD1  (17)

#define reg_V 0x0000
#define reg_I 0x0001
#define reg_P 0x0002
#define reg_kWh 0x0003

float PZEMVoltage, PZEMCurrent, PZEMPower, PZEMEnergy;

unsigned long startMillisPZEM;                        /* start counting time for LCD Display */
unsigned long currentMillisPZEM;                      /* current counting time for LCD Display */
const unsigned long periodPZEM = 1000;                 // refresh every X seconds (in seconds) in LED Display. Default 1000 = 1 second

unsigned long startMillisReadData;                    /* start counting time for data collection */
unsigned long startMillis1;                           // to count time during initial start up (PZEM Software got some error so need to have initial pending time)


ModbusMaster node;

void preTransmission()                                                                                    /* transmission program when triggered*/
{
  /* 1- PZEM-017 DC Energy Meter */
  if (millis() - startMillis1 > 5000)                                                               // Wait for 5 seconds as ESP Serial cause start up code crash
  {
    digitalWrite(MAX485_DE, 1);                                                       /* put DE Pin to high*/
    delay(1);                                                                                       // When both RE and DE Pin are high, converter is allow to transmit communication
  }
}

void postTransmission()                                                                                   /* Reception program when triggered*/
{

  /* 1- PZEM-017 DC Energy Meter */
  if (millis() - startMillis1 > 5000)                                                               // Wait for 5 seconds as ESP Serial cause start up code crash
  {
    delay(3);                                                                                    /* put RE Pin to low*/
    digitalWrite(MAX485_DE, 0);                                                                     /* put DE Pin to low*/
  }
}

float reform_uint16_2_float32(uint16_t u1, uint16_t u2)
{
  uint32_t num = ((uint32_t)u1 & 0xFFFF) << 16 | ((uint32_t)u2 & 0xFFFF);
  float numf;
  memcpy(&numf, &num, 4);
  return numf;
}

float getRTU(uint16_t m_startAddress) {
  uint8_t m_length = 2;
  uint16_t result;
  float x;
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  result = node.readInputRegisters(m_startAddress, m_length);  //readInputRegisters readHoldingRegisters

  if (result == node.ku8MBSuccess) {
    return reform_uint16_2_float32(node.getResponseBuffer(0), node.getResponseBuffer(1));
  }
}

void readData() {
  V = getRTU(reg_V);
  delay(10);
  I = getRTU(reg_I);
  delay(10);
  P = getRTU(reg_P);
  delay(200);
  kWh = getRTU(reg_kWh);
  delay(200);
}

void setup() {
  startMillis1 = millis();
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, 0);
  node.preTransmission(preTransmission);                // Callbacks allow us to configure the RS485 transceiver correctly
  node.postTransmission(postTransmission);

  delay(2000);
  Serial1.begin(9600, SERIAL_8N2, RXD1, TXD1);
  node.begin(pzemSlaveAddr, Serial1);
  delay(2000);
  Serial.begin(9600);

#ifdef PZEM017
  // รอครบ 5 วินาที แล้วตั้งค่า shunt และ address
  while (millis() - startMillis1 < 5000) {
    delay(500);
    Serial.print(".");
  }
  setShunt(pzemSlaveAddr);                            // ตั้งค่า shunt
  changeAddress(0xF8, pzemSlaveAddr);                 // ตั้งค่า address 0x01 ซื่งเป็นค่า default ของตัว PZEM-017
#endif

#ifdef rstENE
  // resetEnergy();                                   // รีเซ็ตค่า Energy[Wh] (หน่วยใช้ไฟสะสม)
#endif

  Serial.println("Leave setup!");
}

void loop()
{
  currentMillisPZEM = millis();
  // อ่านค่าจาก PZEM-017
  if (currentMillisPZEM - startMillisPZEM >= periodPZEM)                                            /* for every x seconds, run the codes below*/
  {
    uint8_t result;                                                                                 /* Declare variable "result" as 8 bits */
    result = node.readInputRegisters(0x0000, 6);                                                    /* read the 9 registers (information) of the PZEM-014 / 016 starting 0x0000 (voltage information) kindly refer to manual)*/
    if (result == node.ku8MBSuccess)                                                                /* If there is a response */
    {
      uint32_t tempdouble = 0x00000000;                                                           /* Declare variable "tempdouble" as 32 bits with initial value is 0 */
      PZEMVoltage = node.getResponseBuffer(0x0000) / 100.0;                                       /* get the 16bit value for the voltage value, divide it by 100 (as per manual) */
      // 0x0000 to 0x0008 are the register address of the measurement value
      PZEMCurrent = node.getResponseBuffer(0x0001) / 100.0;                                       /* get the 16bit value for the current value, divide it by 100 (as per manual) */

      tempdouble =  (node.getResponseBuffer(0x0003) << 16) + node.getResponseBuffer(0x0002);      /* get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
      PZEMPower = tempdouble / 10.0;                                                              /* Divide the value by 10 to get actual power value (as per manual) */

      tempdouble =  (node.getResponseBuffer(0x0005) << 16) + node.getResponseBuffer(0x0004);      /* get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
      PZEMEnergy = tempdouble;
    }
    else // ถ้าติดต่อ PZEM-017 ไม่ได้ ให้ใส่ค่า NAN(Not a Number)
    {
      PZEMVoltage = NAN;
      PZEMCurrent = NAN;
      PZEMPower = NAN;
      PZEMEnergy = NAN;
    }

    // แสดงค่าที่ได้จากบน Serial monitor
    Serial.print("Vdc : "); Serial.print(PZEMVoltage); Serial.println(" V ");
    Serial.print("Idc : "); Serial.print(PZEMCurrent); Serial.println(" A ");
    Serial.print("Power : "); Serial.print(PZEMPower); Serial.println(" W ");
    Serial.print("Energy : "); Serial.print(PZEMEnergy); Serial.println(" Wh ");

    startMillisPZEM = currentMillisPZEM ;                                                       /* Set the starting point again for next counting time */
  }
}

void resetEnergy()                                               // reset energy for Meter 1
{
  uint16_t u16CRC = 0xFFFF;                         /* declare CRC check 16 bits*/
  static uint8_t resetCommand = 0x42;               /* reset command code*/
  uint8_t slaveAddr = pzemSlaveAddr;                 // if you set different address, make sure this slaveAddr must change also
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  preTransmission();                                /* trigger transmission mode*/
  Serial1.write(slaveAddr);                      /* send device address in 8 bit*/
  Serial1.write(resetCommand);                   /* send reset command */
  Serial1.write(lowByte(u16CRC));                /* send CRC check code low byte  (1st part) */
  Serial1.write(highByte(u16CRC));               /* send CRC check code high byte (2nd part) */
  delay(10);
  postTransmission();                               /* trigger reception mode*/
  delay(100);
}

void changeAddress(uint8_t OldslaveAddr, uint8_t NewslaveAddr)                                            //Change the slave address of a node
{

  /* 1- PZEM-017 DC Energy Meter */

  static uint8_t SlaveParameter = 0x06;                                                             /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0002;                                                         /* Modbus RTU device address command code */
  uint16_t u16CRC = 0xFFFF;                                                                         /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, OldslaveAddr);                                                      // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));
  preTransmission();                                                                                 /* trigger transmission mode*/
  Serial1.write(OldslaveAddr);                                                                       /* these whole process code sequence refer to manual*/
  Serial1.write(SlaveParameter);
  Serial1.write(highByte(registerAddress));
  Serial1.write(lowByte(registerAddress));
  Serial1.write(highByte(NewslaveAddr));
  Serial1.write(lowByte(NewslaveAddr));
  Serial1.write(lowByte(u16CRC));
  Serial1.write(highByte(u16CRC));
  delay(10);
  postTransmission();                                                                                /* trigger reception mode*/
  delay(100);
}
