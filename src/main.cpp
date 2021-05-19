#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial softSerial(2, 3); 
uint16_t pwr = 0;
uint16_t time= 0; 
uint16_t start = 0;
uint16_t tagFoundCount=0;


#define RFIDEN 4
String tagInfo="";
uint8_t M=0;
char allTags[89]={0};
unsigned long previousMillis;
unsigned long currentMillis;
String command = "";
typedef struct message // définir une structure message
{
int winnerRSSI[10]={-255,-255,-255,-255,-255,-255,-255,-255,-255,-255}; // initialisation des valeurs winners RSSI 
byte tagEPC[10][12]={{0x00},{0x00}};
}message;
message myData; // créer une structure message nommé myData
volatile bool rfidBusy=false;
unsigned long sentStartTime;
unsigned long lastSentTime;

RFID nano; //Create instance
byte zeroEPC[12]={0x00};
byte mmyEPC[12]; //Most EPCs are 12 bytes

boolean  setupNano(long);
bool stop=false;
uint16_t i=0;
byte a=0;
void requestEvent();
void receiveEvent(int howMany);
void restartRFID();
void setup()
{
  Serial.begin(115200);
  Wire.begin(8);
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
/*********************Nano setup*************************/
  nano.msg[0]=1;
  bool nanoStartup = false;
  while (!nanoStartup){
    rfidBusy=true;
    nano.begin(softSerial);
    softSerial.begin(115200); //Start software serial at 115200
    nano.setBaud(38400); //Tell the module to go to the chosen baud rate. Ignore the response msg
    softSerial.begin(38400); //Start the software serial port, this time at user's chosen baud rate
    while (!softSerial); //Wait for port to open
    while (softSerial.available()) softSerial.read();//200ms
    nano.getVersion();
    switch (nano.msg[0])
    {
    case ALL_GOOD:
      Serial.println("RFID started successfully");
      nano.setTagProtocol(); //Set protocol to GEN2
      nano.setAntennaPort(); //Set TX/RX antenna ports
      rfidBusy=false;
      nanoStartup=true;
      break;
    case ERROR_WRONG_OPCODE_RESPONSE:// baud rate is correct but the module is doing a ccontinuous read
      nano.stopReading();
      Serial.println(F("Module continuously reading. Asking it to stop..."));
      delay(1500);
      rfidBusy=false;
      nanoStartup=true;
      break;
    case ERROR_COMMAND_RESPONSE_TIMEOUT:
      Serial.println("ERROR_COMMAND_RESPONSE_TIMEOUT");
      break;
    case ERROR_CORRUPT_RESPONSE:
      Serial.println("ERROR_CORRUPT_RESPONSE");
      
      break;
    case ERROR_UNKNOWN_OPCODE:
      Serial.println("ERROR_UNKNOWN_OPCODE");
      
      break;
    case RESPONSE_IS_TEMPERATURE:
      Serial.println("RESPONSE_IS_TEMPERATURE");
      
      break;
    case RESPONSE_IS_KEEPALIVE:
      Serial.println("RESPONSE_IS_KEEPALIVE");
      
      break;
    case RESPONSE_IS_TEMPTHROTTLE:
      Serial.println("RESPONSE_IS_TEMPTHROTTLE");
      
      break;
    case RESPONSE_IS_TAGFOUND:
      Serial.println("RESPONSE_IS_TAGFOUND");
      
      break;
    case RESPONSE_IS_NOTAGFOUND:
      Serial.println("RESPONSE_IS_NOTAGFOUND");
      
      break;
    case RESPONSE_IS_UNKNOWN:
      Serial.println("RESPONSE_IS_UNKNOWN");
      
      break;
    case RESPONSE_SUCCESS:
      Serial.println("RESPONSE_SUCCESS");
      
      break;
    case RESPONSE_FAIL:
      Serial.println("RESPONSE_FAIL");
      
      break;
    default:
      Serial.println("Something is wrong with the RFID module... restarting it");
      restartRFID();
      break;
    }
    nano.getVersion();
  } 

/********************************************************/
}

void loop(){
if (start==1){   
  Serial.println("Loop Start =1");
  rfidBusy=true;
  nano.setRegion(REGION_EUROPE); //Set to North America
  nano.setReadPower(pwr); //5.00 dBm. Higher values may caues USB port to brown out
  nano.startReading(); //Begin scanning for tags
  previousMillis=millis();
  while((millis()-previousMillis)<time){
    if (nano.check() == true) //Check to see if any new data has come in from module
    { 
      byte responseType = nano.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp
      switch (responseType)
      {
      case RESPONSE_IS_KEEPALIVE:
        Serial.println(F("Scanning"));
        break;
      
      case RESPONSE_IS_TAGFOUND:
        Serial.print("TAG is found # ");Serial.println(tagFoundCount++);
        int rssi = nano.getTagRSSI(); 
        byte tagEPCBytes = nano.getTagEPCBytes();
        for (byte x = 0 ; x < tagEPCBytes ; x++){
          if (nano.msg[31 + x] < 0x10) {
            mmyEPC[x]=0;
          }
          mmyEPC[x]=nano.msg[31+x];
        }
          for(byte j=0;j<10;j++){
            if (memcmp(mmyEPC,myData.tagEPC[j],12)==0){
              if (rssi>myData.winnerRSSI[j]){
                myData.winnerRSSI[j]=rssi;
              }
              break;
            }
            else{
              if(memcmp(myData.tagEPC[j],zeroEPC,12)==0){
                for(byte jj=0;jj<12;jj++){
                  myData.tagEPC[j][jj]=mmyEPC[jj];
                  myData.winnerRSSI[j]=rssi;
                }
              break;
              }
            }
          }
        break;
      
      case ERROR_CORRUPT_RESPONSE:
        Serial.println("Bad CRC");
        break;
      
      default:
        Serial.print("Unknown error");
        break;
      }
    }
  }
  nano.stopReading();
  delay(1000);
  nano.setReadPower(500); //5.00 dBm. Higher values may caues USB port to brown out
  rfidBusy=false;
  start=0;
}else{
    // rfidBusy=true;
    rfidBusy=false;
    nano.stopReading();
    delay(1000);
    nano.setReadPower(500); //5.00 dBm. Higher values may caues USB port to brown out
  }
}  
void restartRFID(){
  rfidBusy=true;
  Serial.println("restarting the RFID module");
  pinMode(RFIDEN,OUTPUT);
  digitalWrite(RFIDEN,LOW);
  delay(1000);
  pinMode(RFIDEN,INPUT);
  delay(1000);
  rfidBusy=false;
}
void requestEvent() {
  if (M<10 & !rfidBusy)
  {
    tagInfo="";
    for (uint8_t i = 0; i < 12; i++){
      if (myData.tagEPC[M][i]==0){
        tagInfo+="00";
      }else if(myData.tagEPC[M][i]<=15){
        tagInfo+="0";tagInfo+=String(myData.tagEPC[M][i],HEX);
      }else tagInfo+=String(myData.tagEPC[M][i],HEX);
    }
    Serial.print("tagInfo = ");Serial.println(tagInfo);
    Wire.write(tagInfo.c_str());
    M++;
  }
    for (uint8_t j = 0; j < 12; j++){
      myData.tagEPC[M-1][j]={{0x00}};
    }
}
void receiveEvent(int howMany) {
  M=0;tagFoundCount=0;
  if (!rfidBusy)
  {  
    Serial.println("Received something from master: ");
    command="";
    while (Wire.available()) { // loop through all but the last
      char c = Wire.read();    // receive byte as an integer
      command +=c;
    }Serial.println(command);
    uint8_t ind1=command.indexOf(',');Serial.print("ind1 = ");Serial.println(ind1);
    uint8_t ind2=command.indexOf(',',ind1+1);Serial.print("ind2 = ");Serial.println(ind2);
    uint8_t ind3=command.indexOf(',',ind2+1);Serial.print("ind3 = ");Serial.println(ind3);
    start = command.substring(0,1).toInt();Serial.print("start = ");Serial.println(start);
    time= command.substring(ind1+1,ind2).toInt(); Serial.print("time = ");Serial.println(time);
    pwr = command.substring(ind2+1,strlen(command.c_str())).toInt();Serial.print("pwr = "); Serial.println(pwr);
  }
}
boolean setupNano(long baudRate)
{
  // nano.begin(softSerial); //Tell the library to communicate over software serial port
  // softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  // while (!softSerial); //Wait for port to open
  // while (softSerial.available()) softSerial.read();delay(250);
  // nano.getVersion();
  // if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE){
  //   nano.stopReading();
  //   Serial.println(F("Module continuously reading. Asking it to stop..."));
  //   delay(1500);
  // }
  // else{
    softSerial.begin(115200); //Start software serial at 115200
    Serial.println("softSerial.begin(115200);");
    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg
    Serial.println("nano.setBaud(baudRate);");
    softSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate
    Serial.println("softSerial.begin(baudRate);");
  // }
  delay(300);
  nano.getVersion();
  Serial.print("nano.msg[0]  = ");Serial.println(nano.msg[0]);
  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right
  nano.setTagProtocol(); //Set protocol to GEN2
  nano.setAntennaPort(); //Set TX/RX antenna ports to 1
  return (true); //We are ready to rock
}
