#include <SPI.h>
#include <EEPROM.h>
#include <Servo.h>
#include "RF24.h"

//const uint64_t defaultAddress = 0xE8E8F0F0E0LL;//初始的地址，默认起始不会和遥控器连接上，需要进行对码
const uint64_t defaultAddress = 0xE8E8F0F0E1LL;//对码时使用的地址，对码后则不为该地址
const uint8_t CEPin = 9;    //无线模块的CE管脚
const uint8_t CSPin = 10;   //无线模块的CS管脚
const uint8_t LedPin = 6;   //提示灯的管脚
const uint8_t PPMPin = 5;   //PPM信号输出的管脚
const uint8_t PairPin = 4;   //配对的管脚
const uint8_t VoltageCheckPin = A2;   //检测电压管脚
const uint16_t defaultThrottle = 512;
bool PairStatus = false;
bool ConnectStatus = false;
uint64_t CurrentTime = 0;
uint32_t OutTime = 500; //超过1000ms为超出连接时间
uint64_t PairTime = 0;  //在配对模式中用于检测配对时间，如果超出设定时间，则回到配对前的状态
uint64_t PairButtonTime = 0;  //用于检测配对按键是否有按下2秒
bool PairSuccessStatus = false;
bool FirstPressButton = false;
uint64_t BeforeAddress = 0;
RF24 radio(CEPin, CSPin);
Servo esc;

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
void setup() 
{
  
  Serial.begin(9600);
  pinMode(LedPin, OUTPUT);
  pinMode(PairPin, INPUT);
  digitalWrite(PairPin, LOW);
  loadEEPROM();
  initiateReceiver();
  esc.attach(PPMPin);
  
}


void loop() 
{
  //Serial.println((long)RxPacket.Address);
  PairMode();  
  ReceiverToTransmit();
  PPMOutput();
  CalculateVoltage();
  //Serial.println(TxPacket.Vol);
}

void CalculateVoltage()
{
  //满电为42V，最低电压为28V
  uint16_t i = 0;
  int Voltage = 0;
  for(i = 0; i < 10 ;i++)
  {
     Voltage += analogRead(VoltageCheckPin);
  }  
  Voltage = Voltage / 10;
  //RxPacket.Voltage = Voltage;
  //Serial.println(Voltage);
  if(Voltage < 453) RxPacket.Voltage = 0;   //28V的模拟量453，近似28*16.25
  else if(Voltage > 683) RxPacket.Voltage = 100; //42V的模拟量683
  else if((Voltage > 453)&&(Voltage < 683))
  {
      RxPacket.Voltage = (Voltage - 453 ) * 100 /(683 - 453);
  }
  //Serial.println(RxPacket.Voltage); 
}
void PairMode()
{
  uint8_t i,cnt = 0;
  static bool PairButtonState = false;
  static bool PairButtonThreeSec = false;
  for(i = 0 ; i < 20 ; i++)
  {
     cnt += digitalRead(PairPin);   
  }
  if(cnt >= 15) 
  {
    if(PairButtonState == false)
    {
      PairButtonTime = millis();
      PairButtonState = true; 
    }     
  }
  else  PairButtonState = false;
  if((PairButtonState == true)&&(millis() - PairButtonTime) > 3000) 
  {
    PairButtonThreeSec = true;
  }
  if((PairButtonThreeSec == true)&&(TxPacket.statu == 0))//按键按下三秒表示开始配对
  {
     PairButtonThreeSec = false;//把按下三秒的标志位清零，防止反复进入对PairTime有影响
     if(FirstPressButton == false)  BeforeAddress = RxPacket.Address;//在第一次进入的时候先保存原来的地址，以防止配对失败时无法使用原地址
     RxPacket.Address = defaultAddress;//先把地址初始化
     initiateReceiver();     //再初始化接受机
     PairTime = millis();   //计时配对的时间
     PairStatus = true;    //PairStatus为true表示进入了配对模式
     FirstPressButton = true;//FirstPressButton为true表示第一次进入配对模式
     
  }
  if(((millis() - PairTime) > 3000)&&(PairStatus == true))//超过3秒没有配对上
  {
     RxPacket.Address = BeforeAddress;//地址选为原来的地址
     PairButtonThreeSec = false;//把标志位都清零，表示配对模式的结束
     FirstPressButton = false;
     PairStatus = false;
  }
}

void PPMOutput()
{
  if(ConnectStatus == true) esc.writeMicroseconds( map(TxPacket.throttle, 0, 1023, 1000, 2000) );
  else esc.writeMicroseconds(1500);  //虽然没有写值默认为1500，但最好还是在没有信号连接时写入1500
}
void ReceiverToTransmit()
{
  while (radio.available())
  {
    radio.read( &TxPacket, sizeof(TxPacket)); //读取发送来的包
    if(TxPacket.statu == 1) ReceiveAddress(); 
    CurrentTime = millis();
    ConnectStatus = true;
    analogWrite(LedPin, map(TxPacket.throttle, 0, 1023, 0, 255));
    //Serial.println((long)TxPacket.Address);
    radio.writeAckPayload(1, &RxPacket, sizeof(RxPacket));//发送应答信号的包
  }
  if(( (millis() - CurrentTime) > OutTime )&&(PairStatus == false))//超时并且不是配对状态
  {
    //连接超时
    CurrentTime = millis();
    ConnectStatus = false;
    digitalWrite(LedPin, !digitalRead(LedPin));
  }
  if(PairStatus == true)  digitalWrite(LedPin, LOW);
}
//发送地址
void ReceiveAddress()
{
  uint64_t address;
  uint64_t beginTime = millis();
  bool comeStatus = false;
  
  while(2000 >= ( millis() - beginTime))
  {
    if(radio.available()&&(comeStatus == false))  
    {
        radio.read( &address, sizeof(address) );
        RxPacket.TempAddress =  address;
        radio.writeAckPayload(1, &RxPacket, sizeof(RxPacket));
        comeStatus = true;  
    }     
    if(radio.available()&&(comeStatus == true))
    {
        radio.read(&TxPacket, sizeof(TxPacket));
        if(TxPacket.statu == 2)
        {
            //成功表示更换地址
            RxPacket.Address = address; 
            updateEEPROM();
            initiateReceiver();
            PairSuccessStatus = true;
            FirstPressButton = false;
        }
    }   
  }
}
//无线模块初始化的代码
void initiateReceiver(){
  radio.begin();
  radio.setChannel(0); 
  radio.setPALevel(RF24_PA_HIGH); 
  radio.setDataRate(RF24_1MBPS);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, RxPacket.Address);
  radio.startListening();
  //radio.printDetails();
}
//更新EEPROM中的值
void updateEEPROM()
{
  EEPROM.put(0, RxPacket.Address);
}
//加载存在EEPROM中的值
void loadEEPROM()
{
  EEPROM.get(0, RxPacket.Address);  
  if(RxPacket.Address == -1) //TxPacket.Address的值为-1表示没有把值存入
  {
    RxPacket.Address = defaultAddress;//填入一个地址
    updateEEPROM();         //把值写入EEPROM中
  }
}
