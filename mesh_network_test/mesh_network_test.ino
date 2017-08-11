#include <cfloat>

//Radio libraries
#include "RF69Driver.h"
#include "MeshManager.h"

//Sensor libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MAX31856.h>

//#define DEBUG

const uint64_t POLLING_DELTA = 5000;

//Registers for external interrupt controller because noInterrupts() & setInterrupts() not working?
REGISTER8_T  EIC_CTRL   = REGISTER8(0x40001800 + 0x00);
REGISTER8_T  EIC_STATUS = REGISTER8(0x40001800 + 0x01);

//Read processor serial
REGISTER32_T SERIAL0    = REGISTER32(0x0080A00C);
REGISTER32_T SERIAL1    = REGISTER32(0x0080A040);
REGISTER32_T SERIAL2    = REGISTER32(0x0080A044);
REGISTER32_T SERIAL3    = REGISTER32(0x0080A048);

uint32_t cpuID[4] = {SERIAL0, SERIAL1, SERIAL2, SERIAL3};

//Radio initialisation
RF69Driver driver(8, 3);
MeshManager manager(driver, reinterpret_cast<uint8_t*>(cpuID));

//Sensor initialisation
Adafruit_BME280 bme;
Adafruit_MAX31856 thermo(6);

//Messsage buffer in data segment
uint8_t messageBuff[MAX_MESSAGE];

bool thermocoupleConnected = false;

typedef struct {
  float humidity;
  float temp;
  float pressure;
  float thermocouple = FLT_MAX;
} SensorData;

void cli()
{
  //Disable interrupts and wait for SYNCBUSY to return false
  EIC_CTRL &= !0x2;
  while(EIC_STATUS & 0x80);
}

void sei()
{
  //Enable interrupts and wait for SYNCBUSY to return false
  EIC_CTRL |= 0x2;
  while(EIC_STATUS & 0x80);
}

void printPacket(uint8_t addr, const SensorData& packet)
{
  Serial.println("#");
  
  //Print serial number associated with given packet
  if(addr)
    manager.printSerial(addr);
  else
  {
    char buff[3];
    for(int i = 0; i < 16; ++i)
    {
      sprintf(buff, "%02X", (reinterpret_cast<uint8_t*>(cpuID))[i]);
      Serial.print(buff);
    }
    Serial.println("");
  }

  //Print packet data
  Serial.print("tp: ");
  Serial.println(packet.temp);
  Serial.print("hm: ");
  Serial.println(packet.humidity);
  Serial.print("pr: ");
  Serial.println(packet.pressure);
  Serial.print("tc: ");
  Serial.println(packet.thermocouple);
}

void setup()
{ 
  //Begin serial and wait for connection when debugging
  Serial.begin(115200);
#ifdef DEBUG
  while(!Serial);
#endif

  //Initialise the BME280 sensor
  bool success = bme.begin();
  if (!success)
    Serial.println("No BME280 detected");

  //Check for a thermocouple (pull pin 14 low)
  pinMode(14, INPUT_PULLUP);
  thermocoupleConnected = digitalRead(14) == LOW;

  //Initialise the MAX31856 thermocouple amp
  if(thermocoupleConnected)
  {
    thermo.begin();
    thermo.setThermocoupleType(MAX31856_TCTYPE_K);
  }
  else
    Serial.println("No MAX31856 detected");
  
  //Initialise radio
  if(!manager.init())
    Serial.println("RFM69HCM initialisation failed");
  else
  {
    //Set radio power and configure for address filtering
    driver.setTxPower(14, true);
    RH_RF69::ModemConfig cfg = 
      { RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_FSK | RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT1_0,
      0x00,
      0x80,
      0x10,
      0x00,
      0xe0,
      0xe0,
      RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF69_PACKETCONFIG1_DCFREE_WHITENING | RH_RF69_PACKETCONFIG1_CRC_ON | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NODE_BC};
    driver.setModemRegisters(&cfg);
  }

  //Ensure interrupts are enabled
  sei();
}

unsigned long last = 0;
void loop()
{
  //Print received packets
  uint8_t len = MAX_MESSAGE;
  uint8_t from;
  if(manager.recvfromAck(messageBuff, &len, &from))
  {
    SensorData packet;
    memcpy(&packet, messageBuff, sizeof(SensorData));
    printPacket(from, packet);
  }

  if(millis() - last > POLLING_DELTA)
  {
    last = millis();
    
    //Poll sensors
    SensorData packet;
    
    packet.humidity = bme.readHumidity();
    packet.temp = bme.readTemperature();
    packet.pressure = bme.readPressure();
    if(thermocoupleConnected)
    {
      cli();
      packet.thermocouple = thermo.readThermocoupleTemperature();
      sei();
    }
  
    //Send or print as appropriate
    if(!manager.isServer())
    {
      memcpy(messageBuff, &packet, sizeof(SensorData));
      int err = manager.sendtoWait(messageBuff, sizeof(SensorData), 0);
    }
    else
      printPacket(0, packet);
  }

  //Become server on serial communication
  if(Serial.available() && !manager.isServer())
    manager.becomeServer(true);
  while(Serial.available())
    Serial.read();
}
