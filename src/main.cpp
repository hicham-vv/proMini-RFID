#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
#include <SoftwareSerial.h>
SoftwareSerial softSerial(2, 3); 

#define N 3 // nombre de tags // il faut initialiser les valeurs du tableau winnerRSSI[N] par -255  à chaque fois cette valeur est changée 

unsigned long previousMillis;
unsigned long currentMillis;

typedef struct message // définir une structure message
{
int winnerRSSI[N]={-255,-255,-255}; // initialisation des valeurs winners RSSI 
byte tagEPC[N][12]={{0x00},{0x00}};
}message;
message myData; // créer une structure message nommé myData
unsigned long sentStartTime;
unsigned long lastSentTime;

RFID nano; //Create instance
byte zeroEPC[12]={0x00};
byte mmyEPC[12]; //Most EPCs are 12 bytes

boolean  setupNano(long);
bool stop=false;
uint16_t i=0;
byte a=0;

void setup()
{
  Serial.begin(115200);
    while (!Serial); //Wait for the serial port to come online
    if (setupNano(38400) == false) //Configure nano to run at 38400bps
    {
      Serial.println(F("Module failed to respond. Please check wiring."));
      while (1); //Freeze!
    }
    nano.setRegion(REGION_NORTHAMERICA); //Set to North America
    nano.setReadPower(500); //5.00 dBm. Higher values may caues USB port to brown out
//    Serial.println(F("Press a key to begin scanning for tags."));
//    while (!Serial.available()); //Wait for user to send a character
//    Serial.read(); //Throw away the user's character
  
    nano.startReading(); //Begin scanning for tags
  previousMillis=millis();
  while((millis()-previousMillis)<1000){
    if (nano.check() == true) //Check to see if any new data has come in from module
    { 
      byte responseType = nano.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp
      if (responseType == RESPONSE_IS_KEEPALIVE)
      {
        Serial.println(F("Scanning"));
      }
      else if (responseType == RESPONSE_IS_TAGFOUND)
      {
        int rssi = nano.getTagRSSI(); //Get the RSSI for this tag read
        byte tagEPCBytes = nano.getTagEPCBytes(); //Get the number of bytes of EPC from response
      for (byte x = 0 ; x < tagEPCBytes ; x++)
      {
        if (nano.msg[31 + x] < 0x10) { 
          mmyEPC[x]=0;
        }
        mmyEPC[x]=nano.msg[31+x];
      }

        for(byte j=0;j<N;j++){

          if (memcmp(mmyEPC,myData.tagEPC[j],12)==0)
          {
            if (rssi>myData.winnerRSSI[j])
                {
                  myData.winnerRSSI[j]=rssi;
                }
                break;
          }
          else{
            if(memcmp(myData.tagEPC[j],zeroEPC,12)==0){
              for(byte jj=0;jj<12;jj++)
              {
              myData.tagEPC[j][jj]=mmyEPC[jj];
              myData.winnerRSSI[j]=rssi;
              }
              break;
            }
          }
        }
      }
      else if (responseType == ERROR_CORRUPT_RESPONSE)
      {
        Serial.println("Bad CRC");
      }
      else
      { 
        Serial.print("Unknown error");
      }
    }
  }
  nano.stopReading();
  delay(1000);
  nano.setReadPower(500); //5.00 dBm. Higher values may caues USB port to brown out

String tagInfo0="";
if (myData.winnerRSSI[0]>-100){tagInfo0+="-0";tagInfo0+=abs(myData.winnerRSSI[0]);}
else tagInfo0+=String(myData.winnerRSSI[0]);
tagInfo0+=",";
for (uint8_t i = 0; i < 12; i++){
  if (myData.tagEPC[0][i]==0) {tagInfo0+="00";}
  else if(myData.tagEPC[0][i]<=15){tagInfo0+="0";tagInfo0+=String(myData.tagEPC[0][i],HEX);}
  else tagInfo0+=String(myData.tagEPC[0][i],HEX);
}

String tagInfo1="";
if (myData.winnerRSSI[1]>-100){tagInfo1+="-0";tagInfo1+=abs(myData.winnerRSSI[1]);}
else tagInfo1+=String(myData.winnerRSSI[1]);
tagInfo1+=",";
for (uint8_t i = 0; i < 12; i++){
  if (myData.tagEPC[1][i]==0) {tagInfo1+="00";}
  else if(myData.tagEPC[1][i]<=15){tagInfo1+="0";tagInfo1+=String(myData.tagEPC[1][i],HEX);}
  else tagInfo1+=String(myData.tagEPC[1][i],HEX);
}

String tagInfo2="";
if (myData.winnerRSSI[2]>-100){tagInfo2+="-0";tagInfo2+=abs(myData.winnerRSSI[2]);}
else tagInfo2+=String(myData.winnerRSSI[2]);
tagInfo2+=",";
for (uint8_t i = 0; i < 12; i++){
  if (myData.tagEPC[2][i]==0) {tagInfo2+="00";}
  else if(myData.tagEPC[2][i]<=15){tagInfo2+="0";tagInfo2+=String(myData.tagEPC[2][i],HEX);}
  else tagInfo2+=String(myData.tagEPC[2][i],HEX);
}

Serial.print("tagInfo 1 = ");Serial.println(tagInfo0);
Serial.print("tagInfo 2 = ");Serial.println(tagInfo1);
Serial.print("tagInfo 3 = ");Serial.println(tagInfo2);
}
void loop(){}

boolean setupNano(long baudRate)
{
  nano.begin(softSerial); //Tell the library to communicate over software serial port
  softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while (!softSerial); //Wait for port to open
  while (softSerial.available()) softSerial.read();
  nano.getVersion();
  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    nano.stopReading();
    Serial.println(F("Module continuously reading. Asking it to stop..."));
    delay(1500);
  }
  else
  {
    softSerial.begin(115200); //Start software serial at 115200
    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg
    softSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate
  }
  nano.getVersion();
  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right
  nano.setTagProtocol(); //Set protocol to GEN2
  nano.setAntennaPort(); //Set TX/RX antenna ports to 1
  return (true); //We are ready to rock
}
