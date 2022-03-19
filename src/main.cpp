#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
#include <SoftwareSerial.h>
#include <Wire.h>
// #define debug
#define rfidRxPin 3
#define rfidTxPin 2

SoftwareSerial nanoSerial(2, 3); 
uint16_t pwr = 0;
uint16_t time= 0; 
uint16_t start = 0;
uint16_t tagFoundCount=0;
bool rfidStartup = true;

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
volatile unsigned long sentStartTime;
bool receptionFlag = false;
bool requestFlag = false;
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
void rfidON();
void rfidOFF();
void rfidRestart();
void setup()
{
  #ifdef debug
  #endif
  Serial.begin(115200);
  Serial.println("Setup Start");
  rfidOFF();
  Wire.begin(8);
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
  nano.disableDebugging();
  // nano.enableDebugging();
}

void loop(){
  if (receptionFlag && !rfidBusy){
    rfidBusy=true;
    receptionFlag=false;
    rfidON();
    nano.startReading(); //Begin scanning for tags
    previousMillis=millis();
    while((millis()-previousMillis)<time){
      if (nano.check() == true) //Check to see if any new data has come in from module
      { 
        byte responseType = nano.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp
        switch (responseType)
        {
        case RESPONSE_IS_KEEPALIVE:
          #ifdef debug
          Serial.println(F("Scanning"));
          #endif
          break;
        
        case RESPONSE_IS_TAGFOUND:
        {
          #ifdef debug
          Serial.print("TAG is found # ");Serial.println(tagFoundCount++);
          #endif
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
        }
          break;
        
        case ERROR_CORRUPT_RESPONSE:
          #ifdef debug
          Serial.println("Bad CRC");
          #endif
          break;
        
        default:
          #ifdef debug
          Serial.print("Unknown error");
          #endif
          break;
        }
      }
    }
    nano.stopReading();
    delay(1000);
    rfidBusy=false;
  }
  if (requestFlag && !rfidBusy){
    requestFlag=false;
    rfidOFF();
  }
}
void restartRFID(){
  rfidBusy=true;
  #ifdef debug
  Serial.println("restarting the RFID module");
  #endif
  pinMode(RFIDEN,OUTPUT);
  digitalWrite(RFIDEN,LOW);
  delay(1000);
  digitalWrite(RFIDEN,HIGH);
  rfidBusy=false;
}
void requestEvent(){
  if ((M<10) && (!rfidBusy))
  {
    tagInfo="";
    for (uint8_t i = 0; i < 12; i++){
      if (myData.tagEPC[M][i]==0){
        tagInfo+="00";
      }else if(myData.tagEPC[M][i]<=15){
        tagInfo+="0";tagInfo+=String(myData.tagEPC[M][i],HEX);
      }else tagInfo+=String(myData.tagEPC[M][i],HEX);
    }
    if (tagInfo!="000000000000000000000000")
    {
      tagInfo+=",";
      if (myData.winnerRSSI[M]>-10){
        tagInfo+="-";tagInfo+="00";tagInfo+=abs(myData.winnerRSSI[M]);
      }else if ((myData.winnerRSSI[M]<-10)&&(myData.winnerRSSI[M]>-100)){
        tagInfo+="-";tagInfo+="0";tagInfo+=abs(myData.winnerRSSI[M]);
      }else if (myData.winnerRSSI[M]<-100){
        tagInfo+=String(myData.winnerRSSI[M]);
        myData.winnerRSSI[M]=0;
      }
      tagInfo+="*";
      #ifdef debug
      Serial.print("tagInfo = ");Serial.println(tagInfo);
      #endif
      Wire.write(tagInfo.c_str());
      delay(5);
    }else
    {
      tagInfo="#";
      #ifdef debug
      Serial.print("tagInfo = ");Serial.println(tagInfo);
      #endif
      Wire.write(tagInfo.c_str());
      delay(5);
    }
    
    M++;
  }
    for (uint8_t j = 0; j < 12; j++){
      myData.tagEPC[M-1][j]={0x00};
    }
    requestFlag = true;
}
void receiveEvent(int howMany) {
  M=0;tagFoundCount=0;
  if (!rfidBusy)
  {  
    #ifdef debug
    Serial.println("Received something from master: ");
    #endif
    command="";
    while (Wire.available()) { // loop through all but the last
      char c = Wire.read();    // receive byte as an integer
      command +=c;
    }Serial.println(command);
    uint8_t ind1=command.indexOf(',');
    uint8_t ind2=command.indexOf(',',ind1+1);
    uint8_t ind3=command.indexOf(',',ind2+1);
    start = command.substring(0,1).toInt();
    time= command.substring(ind1+1,ind2).toInt(); 
    pwr = command.substring(ind2+1,strlen(command.c_str())).toInt();
    #ifdef debug
    Serial.print("ind1 = ");Serial.println(ind1);
    Serial.print("ind2 = ");Serial.println(ind2);
    Serial.print("ind3 = ");Serial.println(ind3);
    Serial.print("start = ");Serial.println(start);
    Serial.print("time = ");Serial.println(time);
    Serial.print("pwr = "); Serial.println(pwr);
    #endif
  }
  receptionFlag=true;
  // digitalWrite(RFIDEN,HIGH);
}
bool setupNano(){
  nano.msg[0]=1;
  if (rfidStartup){
    #ifdef debug
    Serial.println("rfid startup");
    #endif
    rfidBusy=true;
    nano.begin(nanoSerial);
    nanoSerial.begin(115200); 
    while (nanoSerial.isListening() == false);
    while (nanoSerial.available()){
      nanoSerial.read();
    }
    delay(200);
    nano.setBaud(38400); 
    nanoSerial.begin(38400); 
    delay(250);
    rfidBusy=false;
  }else{
    rfidBusy=true;
    nanoSerial.begin(38400);
    rfidBusy=false;
  }
  
  rfidBusy=true;
  if (nanoSetTagProtocol()){//1
    if (nanoSetAntennaPort()){//2
      if (nanoSetRegion(REGION_EUROPE)){//3
        if (nanoSetReadPower(pwr)){//4
          return true;
          rfidBusy=false;
        }else {rfidBusy=false;return false;}
      }else {rfidBusy=false;return false;}
    }else {rfidBusy=false;return false;}
  }else {rfidBusy=false;return false;}
}
bool nanoGetVersion(){
  nano.getVersion();
  if (parseResponse(0, nano.msg[0])){return true;i=3;}else {return false;}
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
      #ifdef debug
      Serial.println(F("nano.msg[0].stopReading = ALL GOOD"));
      #endif
      return true;
    }else{i=3;
      return false;
      break;
    }    
  }
}
bool parseResponse(uint8_t ID, uint8_t msg){
  #ifdef debug
  Serial.print(" nano.msg[0].");Serial.print(ID);Serial.print(" = ");Serial.println(nano.msg[0],HEX);
  #endif
  switch (msg)
  {
    case ALL_GOOD:
    #ifdef DEBUG
      Serial.print(ID);Serial.println(": OK");
    #endif
      rfidBusy=false;
      // nanoStartupOK=true;
      nano.msg[0]=1;
      return true;
      break;
    case ERROR_WRONG_OPCODE_RESPONSE:
      #ifdef debug
      Serial.print(ID);Serial.println(F("Continuous reading. Stopping..."));
      #endif
      nanoStopReading();
      delay(1500);
      break;
    case ERROR_COMMAND_RESPONSE_TIMEOUT:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _COMMAND_RESPONSE_TIMEOUT"));
      #endif
      restartRFID();
      return false;
      break;
    case ERROR_CORRUPT_RESPONSE:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _CORRUPT_RESPONSE"));
      #endif
      delay(250);
      return false;
      break;
    case ERROR_UNKNOWN_OPCODE:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _UNKNOWN_OPCODE"));
      #endif
      return false;
      break;
    case RESPONSE_IS_TEMPERATURE:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_TEMPERATURE"));
      #endif
      return false;
      nanoStopReading();
      break;
    case RESPONSE_IS_KEEPALIVE:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_KEEPALIVE"));
      #endif
      return false;
      break;
    case RESPONSE_IS_TEMPTHROTTLE:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_TEMPTHROTTLE"));
      #endif
      return false;
      nanoStopReading();
      break;
    case RESPONSE_IS_TAGFOUND:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_TAGFOUND"));
      #endif
      return false;
      break;
    case RESPONSE_IS_NOTAGFOUND:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_NOTAGFOUND"));
      #endif
      break;
    case RESPONSE_IS_UNKNOWN:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_UNKNOWN"));
      #endif
      return false;
      restartRFID();
      break;
    case RESPONSE_SUCCESS:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _SUCCESS"));
      #endif
      return false;
      break;
    case RESPONSE_FAIL:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _FAIL"));
      #endif
      restartRFID();
      return false;
      break;
    default:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": ???"));
      #endif
      restartRFID();
      return false;
      break;
  }
}
void rfidON(){
  pinMode(RFIDEN,OUTPUT);
  #ifdef debug
  Serial.println("rfidON");
  #endif
  digitalWrite(3,LOW);
  delay(40);
  digitalWrite(RFIDEN,HIGH);
  delay(40);
  SoftwareSerial nanoSerial(2, 3); 
  setupNano();
  rfidStartup=true;
}
void rfidOFF(){
  pinMode(RFIDEN,OUTPUT);
  #ifdef debug
  Serial.println("rfidOFF");
  #endif  
  digitalWrite(3,LOW);
  delay(40);
  digitalWrite(RFIDEN,LOW);
  rfidStartup=true;
}
void rfidRestart(){
  rfidOFF();
  rfidON();
}

