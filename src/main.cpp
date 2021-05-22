#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial softSerial(2, 3); 
uint16_t pwr = 0;
uint16_t time= 0; 
uint16_t start = 0;
uint16_t tagFoundCount=0;


#define RFIDEN A0
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
bool receptionEvent = false;
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
bool setupNano();
bool nanoGetVersion();
bool nanoSetTagProtocol();
bool nanoSetAntennaPort();
bool nanoSetRegion(uint8_t nanoRegion);
bool nanoSetReadPower(uint16_t nanoPower);
bool nanoStopReading();
bool parseResponse(uint8_t ID, uint8_t msg);
void setup()
{
  Serial.begin(115200);
  Serial.println("turning off rfid module");
  // pinMode(RFIDEN,OUTPUT);
  // digitalWrite(RFIDEN,LOW);
  Wire.begin(8);
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
  nano.disableDebugging();
  // nano.enableDebugging();
  // setupNano();
}

void loop(){
if (receptionEvent){
  receptionEvent=false;
  setupNano();
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
  rfidBusy=false;
}
}
void restartRFID(){
  rfidBusy=true;
  Serial.println("restarting the RFID module");
  pinMode(RFIDEN,OUTPUT);
  digitalWrite(RFIDEN,LOW);
  delay(1000);
  digitalWrite(RFIDEN,HIGH);
  rfidBusy=false;
}
void requestEvent(){
  if ((M<10) && (!rfidBusy))
  {
    // digitalWrite(RFIDEN,LOW);
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
    // digitalWrite(RFIDEN,LOW);
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
  receptionEvent=true;
  // digitalWrite(RFIDEN,HIGH);
}
bool setupNano(){
  nano.msg[0]=1;
  bool nanoStartupOK = false;
  // digitalWrite(RFIDEN,HIGH);
  while (!nanoStartupOK){
    rfidBusy=true;
    nano.begin(softSerial);
    softSerial.begin(115200); //Start software serial at 115200
    nano.setBaud(38400); //Tell the module to go to the chosen baud rate. Ignore the response msg
    softSerial.begin(38400); //Start the software serial port, this time at user's chosen baud rate
    while (!softSerial); //Wait for port to open
    while (softSerial.available()) softSerial.read();delay(200);
    if(nanoGetVersion()){
      if (nanoSetTagProtocol()){//Set protocol to GEN2
        if (nanoSetAntennaPort()){
          if (nanoSetRegion(REGION_EUROPE)){
            if (nanoSetReadPower(pwr)){
              return true;
            }else return false;
          }else return false;
        }else return false;
      }else return false;
    }else return false;
  } 
    rfidBusy=false;
}
bool nanoGetVersion(){
  for (uint8_t i = 0; i < 3; i++){
    nano.getVersion();
    if (parseResponse(0, nano.msg[0])){return true;i=3;break;}else {return false;}
  }
}
bool nanoSetTagProtocol(){
  for (uint8_t i = 0; i < 3; i++){
    nano.setTagProtocol();
    if (parseResponse(1, nano.msg[0])){return true;i=3;break;}else {return false;}
  }
}
bool nanoSetAntennaPort(){
  for(uint8_t i = 0; i < 3; i++){
    nano.setAntennaPort();
        if (parseResponse(2, nano.msg[0])){return true;i=3;break;}else {return false;}
  }
}
bool nanoSetRegion(uint8_t nanoRegion){
  for(uint8_t i = 0; i < 3; i++){
    nano.setRegion(nanoRegion);
        if (parseResponse(3, nano.msg[0])){return true;i=3;break;}else {return false;}
  }
}
bool nanoSetReadPower(uint16_t nanoPower){
  for(uint8_t i = 0; i < 3; i++){
    nano.setReadPower(nanoPower);
        if (parseResponse(4, nano.msg[0])){return true;i=3;break;}else {return false;}
  }
}
bool nanoStopReading(){
  for(uint8_t i = 0; i < 3; i++){
    nano.stopReading();
    if (nano.msg[0]==ALL_GOOD){
      Serial.println(F("nano.msg[0].stopReading = ALL GOOD"));
      return true;
    }else{i=3;
      return false;
      break;
    }    
  }
}
bool parseResponse(uint8_t ID, uint8_t msg){
  Serial.print(" nano.msg[0].");Serial.print(ID);Serial.print(" = ");Serial.println(nano.msg[0],HEX);
  switch (msg)
  {
    case ALL_GOOD:
      Serial.print(ID);Serial.println(": OK");
      rfidBusy=false;
      // nanoStartupOK=true;
      nano.msg[0]=1;
      return true;
      break;
    case ERROR_WRONG_OPCODE_RESPONSE:
      nanoStopReading();
      Serial.print(ID);Serial.println(F("Continuous reading. Stopping..."));
      delay(1500);
      break;
    case ERROR_COMMAND_RESPONSE_TIMEOUT:
      Serial.print(ID);Serial.println(F(": _COMMAND_RESPONSE_TIMEOUT"));
      return false;
      break;
    case ERROR_CORRUPT_RESPONSE:
      Serial.print(ID);Serial.println(F(": _CORRUPT_RESPONSE"));
      return false;
      break;
    case ERROR_UNKNOWN_OPCODE:
      Serial.print(ID);Serial.println(F(": _UNKNOWN_OPCODE"));
      return false;
      break;
    case RESPONSE_IS_TEMPERATURE:
      Serial.print(ID);Serial.println(F(": _IS_TEMPERATURE"));
      return false;
      nanoStopReading();
      break;
    case RESPONSE_IS_KEEPALIVE:
      Serial.print(ID);Serial.println(F(": _IS_KEEPALIVE"));
      return false;
      break;
    case RESPONSE_IS_TEMPTHROTTLE:
      Serial.print(ID);Serial.println(F(": _IS_TEMPTHROTTLE"));
      return false;
      nanoStopReading();
      break;
    case RESPONSE_IS_TAGFOUND:
      Serial.print(ID);Serial.println(F(": _IS_TAGFOUND"));
      return false;
      break;
    case RESPONSE_IS_NOTAGFOUND:
      Serial.print(ID);Serial.println(F(": _IS_NOTAGFOUND"));
      break;
    case RESPONSE_IS_UNKNOWN:
      Serial.print(ID);Serial.println(F(": _IS_UNKNOWN"));
      return false;
      nanoStopReading();
      break;
    case RESPONSE_SUCCESS:
      Serial.print(ID);Serial.println(F(": _SUCCESS"));
      return false;
      break;
    case RESPONSE_FAIL:
      Serial.print(ID);Serial.println(F(": _FAIL"));
      return false;
      break;
    default:
      Serial.print(ID);Serial.println(F(": ???"));
      return false;
      break;
  }
}