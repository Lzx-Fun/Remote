#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "RF24.h"

const uint16_t maxHallValue = 1023;
const uint16_t minHallValue = 0;
const uint16_t centerHallValue = 512;

const uint8_t hallNoiseMargin = 5;//滤除微小的变动
const uint8_t CEPin = 9;
const uint8_t CSPin = 10;
const uint8_t SignalGreenLedPin = 5;
//const uint8_t vib rationPin = 8;
const uint8_t SignalRedLedPin = 4;
const uint8_t PairPin = 3;//对码按键
const uint8_t BatPin = A1;
const uint8_t BatMeasurePin = A2;
const uint8_t hallSensorPin = A3;
const uint8_t PowerControlPin = 6;
const uint8_t PowerLedPin = 7;
const uint8_t Vol100LedPin = A5;
const uint8_t Vol75LedPin = A4;
const uint8_t Vol50LedPin = A0;
const uint8_t Vol20LedPin = 1;
//const uint64_t defaultAddress = 0xE8E8F0F0E0LL;
const uint64_t defaultAddress = 0xE8E8F0F0E1LL;

//bool SignalLedStatus = false;
uint64_t lastTransmission = 0;
uint64_t OverConnectTime = 0;
bool Connect = false;
bool OpenFlag = false;//开机状态
bool OpenInitFlag = false;//开机初始化系统状态，为True后无需再次初始化
uint8_t Level = 0;

const float minVoltage = 3.2; //源码为3.2
const float maxVoltage = 4.2;
const float refVoltage = 5.0;


RF24 radio(CEPin, CSPin);

struct packageTX{
  uint8_t statu;  //0：正常发送数据  1：准备接接收地址  2：成功接收地址
  uint64_t Address;
  uint16_t throttle;
  uint16_t Vol;
};
struct packageRX{
  //bool statu;  
  uint64_t Address;
  uint64_t TempAddress;
  uint32_t Voltage;
  
  //uint16_t throttle;
};
struct packageTX TxPacket;
struct packageRX RxPacket;


void setup() {
  Start_Up_Init();
  //Serial.begin(9600);
}
void loop() {
  
  ControlSwith();
  if(OpenFlag == true)
  {
    if(OpenInitFlag == true)  
    {
      //开机并且已经初始化的状态，执行开机代码
      getThrottlePosition();
      transmitToReceiver();
      PairMode();
      LevelChoose();
      DisplayBatteryVlotage();
      DisplayESCVlotage();
    }
    else
    {
      //开机后未初始化，先进行系统初始化
      GPIO_Init();
      DisplayBatteryVlotage();
      loadEEPROM();
      initiateTransmitter();
      OpenInitFlag = true;//初始化后不再进入
    }
  }
//  Serial.println((long)TxPacket.Address);
//  TxPacket.Vol = analogRead(BatPin);
}

void ControlSwith()
{
  static uint32_t CurrentTime;
  
  if(OpenFlag == false)
  {
    if(analogRead(BatPin) <= 1000) CurrentTime = millis();
    while(analogRead(BatPin) <= 1000)
    {
      if((millis() - CurrentTime) >= 1000) 
      {
        OpenFlag = true;
        digitalWrite(PowerLedPin, HIGH);
        digitalWrite(PowerControlPin, HIGH);
      }
    }
  }
  else
  {
    if(analogRead(BatPin) <= 1000) CurrentTime = millis();
    while(analogRead(BatPin) <= 1000)
    {
      if((millis() - CurrentTime) >= 2000) 
      {
        OpenFlag = false;
        updateEEPROM();
        Led_Off();
        digitalWrite(PowerLedPin, LOW);
        digitalWrite(PowerControlPin, LOW);
      }
    }  
  }
  if(OpenFlag == true) digitalWrite(PowerControlPin, HIGH);
  else 
  {
    Led_Off();
    digitalWrite(PowerControlPin, LOW);
  }
}
//显示遥控器电池电压
void DisplayBatteryVlotage()
{
    static uint32_t CurrentTime;
    if((millis() - CurrentTime) > 1000)//1秒钟检测一次
    {
      if(RemoteBatteryLevel() <= 20)//低于20%会闪烁
      {
         digitalWrite(PowerLedPin, !digitalRead(PowerLedPin));
      }
      else//高于20%会常亮
      {
         digitalWrite(PowerLedPin, HIGH);  
      }
      CurrentTime = millis();
    }
    //Serial.println(analogRead(BatMeasurePin));
}
//计算遥控器的电源
uint8_t RemoteBatteryLevel() 
{
  unsigned short total = 0;
  //uint8_t RemoteBattery = 0;
  for (uint8_t i = 0; i < 20; i++) {
    total += analogRead(BatMeasurePin);
  }
  float voltage = (refVoltage / 1024.0) * ((float)total / 20.0);
  if (voltage <= minVoltage) {
    return 0;
  } else if (voltage >= maxVoltage) {
    return 100;
  } else {
    return (voltage - minVoltage) * 100 / (maxVoltage - minVoltage);
  }
}

void DisplayESCVlotage()
{
  static uint32_t CurrentTime;
  if(Connect == true)
  {
    if((millis() - CurrentTime) > 1000)//1秒钟检测一次
    {
      if(RxPacket.Voltage >= 90)
      {
         digitalWrite(Vol100LedPin, LOW);
         digitalWrite(Vol75LedPin, LOW);
         digitalWrite(Vol50LedPin, LOW);
         digitalWrite(Vol20LedPin, LOW);
      }
      else if((RxPacket.Voltage >= 75)&&(RxPacket.Voltage <= 90))
      {
         digitalWrite(Vol100LedPin, HIGH);
         digitalWrite(Vol75LedPin, LOW);
         digitalWrite(Vol50LedPin, LOW);
         digitalWrite(Vol20LedPin, LOW);
      }
      else if((RxPacket.Voltage >= 50)&&(RxPacket.Voltage <= 75))
      {
         digitalWrite(Vol100LedPin, HIGH);
         digitalWrite(Vol75LedPin, HIGH);
         digitalWrite(Vol50LedPin, LOW);
         digitalWrite(Vol20LedPin, LOW); 
      }
      else if((RxPacket.Voltage >= 20)&&(RxPacket.Voltage <= 50))
      {
         digitalWrite(Vol100LedPin, HIGH);
         digitalWrite(Vol75LedPin, HIGH);
         digitalWrite(Vol50LedPin, HIGH);
         digitalWrite(Vol20LedPin, LOW); 
      }
      else if((RxPacket.Voltage <= 20))
      {
         digitalWrite(Vol100LedPin, HIGH);
         digitalWrite(Vol75LedPin, HIGH);
         digitalWrite(Vol50LedPin, HIGH);
         digitalWrite(Vol20LedPin, HIGH);
      }
      CurrentTime = millis();
    }
//    Serial.println("Connect = true");
//    Serial.println(RxPacket.Voltage);
  }
  else
  {
       digitalWrite(Vol100LedPin, HIGH);
       digitalWrite(Vol75LedPin, HIGH);
       digitalWrite(Vol50LedPin, HIGH);
       digitalWrite(Vol20LedPin, HIGH);
//       Serial.println("Connect = false");
  }
  
}
//在正式开机前只控制这两个管脚
void Start_Up_Init()
{
  pinMode(PowerControlPin, OUTPUT);  
  pinMode(PowerLedPin, OUTPUT);
//  digitalWrite(PowerLedPin, HIGH);
//  digitalWrite(PowerControlPin, HIGH);
}
//在开机后的管脚初始化
void GPIO_Init()
{
  pinMode(PairPin, INPUT_PULLUP);
  pinMode(PowerControlPin, OUTPUT);
  pinMode(Vol100LedPin, OUTPUT);
  pinMode(Vol75LedPin, OUTPUT);
  pinMode(Vol50LedPin, OUTPUT);
  pinMode(Vol20LedPin, OUTPUT);
  pinMode(PowerLedPin, OUTPUT);
  pinMode(SignalGreenLedPin, OUTPUT);
  pinMode(SignalRedLedPin, OUTPUT);
  
  digitalWrite(Vol100LedPin, HIGH);
  digitalWrite(Vol75LedPin, HIGH);
  digitalWrite(Vol50LedPin, HIGH);
  digitalWrite(Vol20LedPin, HIGH);
  digitalWrite(SignalRedLedPin, HIGH);  
  digitalWrite(SignalGreenLedPin, LOW);


}
//在关机的时候需要把灯关闭
void Led_Off()
{
  digitalWrite(Vol100LedPin, HIGH);
  digitalWrite(Vol75LedPin, HIGH);
  digitalWrite(Vol50LedPin, HIGH);
  digitalWrite(Vol20LedPin, HIGH);
  digitalWrite(SignalRedLedPin, HIGH);  
  digitalWrite(SignalGreenLedPin, HIGH);
}
//档位选择
void LevelChoose()
{
  if((digitalRead(PairPin) == LOW)&&(TxPacket.throttle <= 100)) Level++;  
  if(Level == 3) Level = 0;  
}
//配对模式
void PairMode()
{
  //要油门在中位并且按下配对按键才能进入配对模式
  //if((digitalRead(PairPin) == LOW)&&(TxPacket.throttle <= (512 + 150))&&(TxPacket.throttle >= (512 - 150)))
  if(digitalRead(PairPin) == LOW)
  {
    TxPacket.Address = defaultAddress;
    initiateTransmitter();
    transmitAddress();  
  }
}
//初始化发射模块的配置
void initiateTransmitter()
{
  radio.begin();
  radio.setChannel(0);
  radio.powerUp();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  //radio.setAutoAck(txSettings.address, true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe( TxPacket.Address );
}
//发送数据给接收机，并且接收接收机的数据
void transmitToReceiver()
{
    static uint64_t CurrentTime = 0;
    static uint8_t cnt = 0;
    uint32_t OutTime = 500;
    TxPacket.statu = 0;
    if(radio.write( &TxPacket, sizeof(TxPacket) )){   
      //成功连接
      while (radio.isAckPayloadAvailable()) {
        //成功连接并且收到应答信号
        radio.read( &RxPacket, sizeof(RxPacket) );//读取收到的应答信号的包
        Connect = true;        
        if(Level == 0)
        {
          digitalWrite(SignalRedLedPin, LOW); 
          digitalWrite(SignalGreenLedPin, LOW); 
        }else if(Level == 1){
          digitalWrite(SignalRedLedPin, LOW); 
          digitalWrite(SignalGreenLedPin, HIGH);  
        }else if(Level == 2){
          digitalWrite(SignalRedLedPin, HIGH); 
          digitalWrite(SignalGreenLedPin, LOW);  
        }
        CurrentTime = millis();
      }
    }
    if((millis() - CurrentTime) > OutTime)
    {
        if(Level == 0)
        {
          digitalWrite(SignalRedLedPin, cnt%2); 
          digitalWrite(SignalGreenLedPin, cnt%2); 
          cnt++;
          if(cnt==2)cnt = 0;
        }else if(Level == 1){
          digitalWrite(SignalRedLedPin, !digitalRead(SignalRedLedPin)); 
          digitalWrite(SignalGreenLedPin, HIGH);  
        }else if(Level == 2){
          digitalWrite(SignalRedLedPin, HIGH); 
          digitalWrite(SignalGreenLedPin, !digitalRead(SignalGreenLedPin));  
        }
        Connect = false;
        initiateTransmitter();
        CurrentTime = millis();
    }
}
//发送新的地址
void transmitAddress()
{
   unsigned short settingWaitDelay = 500;
   uint64_t beginTime = millis();
   uint64_t PairTime = 0;
   uint64_t address = generateAddress();
   bool confirm = false;
   //Serial.println((long)TxPacket.Address);
   //Serial.println((long)address);
   //先清除上次有可能存在的应答信号的包
   while (radio.isAckPayloadAvailable() && settingWaitDelay >= ( millis() - beginTime)){
      radio.read( &RxPacket, sizeof(RxPacket) ); 
   }
   TxPacket.statu = 1;
   
   if(radio.write( &TxPacket, sizeof(TxPacket) )){
      confirm = true; 
   }else{
      confirm = false; 
   }   
   //发送新产生的地址
   PairTime = millis();
   while(radio.write( &address, sizeof(address) ) && (confirm == true) && ((millis() - PairTime) < 500)){
      //成功连接
      while(radio.isAckPayloadAvailable()) {
        //成功连接并且收到应答信号
          radio.read( &RxPacket, sizeof(RxPacket) );//读取收到的应答信号的包
          //如果地址一样，则表示成功
          if(address == RxPacket.TempAddress){
            TxPacket.statu = 2;
            confirm = false;
          }
      }
    }
    if(TxPacket.statu == 2){
        if(radio.write( &TxPacket, sizeof(TxPacket) )){
            TxPacket.Address = address;
            initiateTransmitter();
            updateEEPROM();
          } 
     }else{
        loadEEPROM();
        initiateTransmitter();
     }
    
}
//更新EEPROM中的值
void updateEEPROM()
{
  EEPROM.put(0, TxPacket.Address);
}
//加载存在EEPROM中的值
void loadEEPROM()
{
  EEPROM.get(0, TxPacket.Address);  
  if(TxPacket.Address == -1) //TxPacket.Address的值为-1表示没有把值存入
  {
    TxPacket.Address = defaultAddress;//填入默认地址
    updateEEPROM();         //把值写入EEPROM中
  }
}
//获取油门的位置
void getThrottlePosition()
{
  unsigned short total = 0;
  int hallThrottleValue = 0;
  for ( uint8_t i = 0; i < 10; i++ )
  {
    total += analogRead(hallSensorPin);
  }  
  hallThrottleValue = total / 10;
  if(hallThrottleValue > centerHallValue)
  {
    if(Level == 0) TxPacket.throttle = constrain( map(hallThrottleValue, centerHallValue, maxHallValue, centerHallValue, 1023), centerHallValue, 1023 );
    else if(Level == 1) TxPacket.throttle = constrain( map(hallThrottleValue, centerHallValue, maxHallValue, centerHallValue, 1023), centerHallValue, 896 );
    else if(Level == 2) TxPacket.throttle = constrain( map(hallThrottleValue, centerHallValue, maxHallValue, centerHallValue, 1023), centerHallValue, 768 );
  }else{
     TxPacket.throttle = constrain( map(hallThrottleValue, minHallValue, centerHallValue, 0, centerHallValue), 0, centerHallValue );
  }
  // Remove hall center noise
  if ( abs(TxPacket.throttle - centerHallValue) < hallNoiseMargin )
  {
    TxPacket.throttle = centerHallValue;
  }
}
// Generate a random address for nrf24 communication
uint64_t generateAddress()
{
  randomSeed( millis() );//喂随机种子
  // Holding the address as char array
  char temp[10];
  // Char arrays with HEX digites
  const char *hexdigits = "0123456789ABCDEF";
  const char *safedigits = "12346789BCDE";

  // Generate a char array with the pipe address
  for(uint8_t i = 0 ; i < 10; i++ )
  {
    char next;
    // Avoid addresses that start with 0x00, 0x55, 0xAA and 0xFF.
    if(i == 0)
      next = safedigits[ random(0, 12) ];
    else if(i == 1)
      next = safedigits[ random(0, 12) ];
    // Otherwise generate random HEX digit
    else
      next = hexdigits[ random(0, 16) ];
    temp[i] = next;
  }
  // Convert hex char array to uint64_t 
  return StringToUint64(temp);
}
// Convert hex String to uint64_t: http://forum.arduino.cc/index.php?topic=233813.0
uint64_t StringToUint64( char * string ){
  uint64_t x = 0;
  char c;
  do {
    c = hexCharToBin( *string++ );
    if (c < 0)
      break;
    x = (x << 4) | c;
  } while (1);
  return x;
}

char hexCharToBin(char c) {
  if (isdigit(c)) {  // 0 - 9
    return c - '0';
  } else if (isxdigit(c)) { // A-F, a-f
    return (c & 0xF) + 9;
  }
  return -1;
}
