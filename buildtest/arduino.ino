#include <TimerOne.h>
#include <SPI.h>

#include"interface.h"
#include "common.h"



enum {
    PROTOOPTS_STARTBIND = 0,
    PROTOOPTS_BITRATE,
    LAST_PROTO_OPT,
};

enum {
    NRF24L01_BR_1M = 0,
    NRF24L01_BR_2M,
    NRF24L01_BR_250K,
    NRF24L01_BR_RSVD
};

struct ModelStruct Model;
volatile s32 Channels[NUM_OUT_CHANNELS];
struct mcu_pin MODULE_ENABLE[TX_MODULE_LAST];
const int CS_pin = 9;
const int SS_pin = 10;

void setup() {

  pinMode(CS_pin, OUTPUT);
  pinMode(SS_pin, OUTPUT);

  digitalWrite(CS_pin, HIGH);
  
  SPI.begin();

  Serial.begin(1000000); 
  Serial.println("FutureBOX");
    
  Model.proto_opts[PROTOOPTS_BITRATE] = NRF24L01_BR_250K;
  Model.proto_opts[PROTOOPTS_STARTBIND] = 1;
  Model.tx_power = TXPOWER_150mW;
  Model.num_channels = 11L;
  Model.fixed_id = 0;

  Serial.println("sending INIT");
  //CX10_Cmds(PROTOCMD_INIT);
  //V202_Cmds(PROTOCMD_INIT);
  //Skeye_Cmds(PROTOCMD_INIT);
  //delay(5);
  //Serial.println("sending BIND");
  //CX10_Cmds(PROTOCMD_BIND);
  //V202_Cmds(PROTOCMD_BIND);
  //Skeye_Cmds(PROTOCMD_BIND);
}

#define PKT_LEN 13
unsigned char serialData[PKT_LEN];
int en = 0;
int st = 0;

int activeProtocol = 0;

void executeCommand(unsigned char* data) {

  switch (data[(st+2) % PKT_LEN]) {
     case 10: // list supported protocols
      Serial.print("\x35\x7a"); // response header
      Serial.print(10);         // command
      Serial.print(PROTOCOL_COUNT);// response length
      
      for (int i =0; i < PROTOCOL_COUNT; i++) {
        Serial.print(i, HEX);
        Serial.println(ProtocolNames[i]);
      }
      break;
     case 11: // set active protocol
      // if another protocol was selected make sure we deinit first
      if (activeProtocol != 0) {
        PROTOCOL_LOADED(PROTOCMD_DEINIT);
      }
      activeProtocol = data[(st+3) % PKT_LEN];
      PROTOCOL_Load(activeProtocol);
      break;
     case 12: // set model
      Model.proto_opts[PROTOOPTS_BITRATE] = data[(st+3) % PKT_LEN]; //NRF24L01_BR_250K;
      Model.proto_opts[PROTOOPTS_STARTBIND] = data[(st+4) % PKT_LEN]; //1;
      Model.tx_power = static_cast<TxPower>(data[(st+5) % PKT_LEN]); //TXPOWER_150mW;
      Model.num_channels = data[(st+6) % PKT_LEN];//11L;
      Model.fixed_id = data[(st+7) % PKT_LEN] << 24 + data[(st+8) % PKT_LEN] << 16 + data[(st+9) % PKT_LEN] << 8 + data[(st+10) % PKT_LEN];
      break;
     case 0:  // command 0: BIND
      PROTOCOL_LOADED(PROTOCMD_BIND);
      //Serial.println("BIND");
      //X900_Cmds(PROTOCMD_BIND);
      //Skeye_Cmds(PROTOCMD_INIT);
      //V202_Cmds(PROTOCMD_BIND); // should call command based on the model which is configured
      return;
     case 1: // command 1: update channels
     // elevator: forward backward
     // aileron: turn
     // rudder: left/right
     
      // futurebox channels: throttle: 0, elevator: 3, aileron: 1, rudder: 2
      // DevTX channels: Aileron is channel 0, Elevator - 1, Throttle - 2, Rudder - 3
      Channels[0] = data[(st+6) % PKT_LEN];//((data[(st+5) % PKT_LEN] << 1) - 0x100)*1000/0xff;
      Channels[1] = data[(st+5) % PKT_LEN];//((data[(st+4) % PKT_LEN] << 1) - 0x100)*1000/0xff;
      Channels[2] = data[(st+3) % PKT_LEN];//((data[(st+3) % PKT_LEN] << 1) - 0x100)*1000/0xff;
      Channels[3] = data[(st+4) % PKT_LEN];//((data[(st+6) % PKT_LEN] << 1) - 0x100)*1000/0xff;
      /*Channels[4] = data[5];
      Channels[5] = data[6];
      Channels[6] = data[7];
      Channels[7] = data[8];
      Channels[8] = data[9];*/
      //Serial.println("DATA OK");
      //digitalWrite(ledPin, !digitalRead(ledPin));
      break;
    case 2: // command 2: set model
      break;
  }
}


void loop () {
  int av = Serial.available();
  
  if (Serial.available()) {
    Serial.readBytes(serialData + (en++ % PKT_LEN), 1);

    // wait till we buffer at least the length of the packet
    if (en - st < PKT_LEN) return;
    
    // search for the beggining of packet
    if (serialData[st % PKT_LEN] != 0x3f && serialData[(st+1) % PKT_LEN] != 0x7a) {
      st++;
      return;
    }
   
    int sum = 0;
    //for (int i =0; i< PKT_LEN; i++) {Serial.print(serialData[(st + i) % PKT_LEN], HEX); Serial.print("-");  }  Serial.println();
    for (int i =2; i< PKT_LEN-1; i++) {sum+=serialData[(st + i) % PKT_LEN];}

    if ((sum & 0xff) != serialData[(st + PKT_LEN-1) % PKT_LEN]) {
      Serial.print(sum, HEX);
      Serial.print(" :: ");
      Serial.println(" - BAD PACKET");
      st++;
    } else {
      en = en % PKT_LEN;
      st = ( st + 13 ) % PKT_LEN;
      executeCommand(serialData);
    }
    
  }
  /*while (av > 9)
    {
      Serial.readBytes(serialData, 10);
      for (int i =0; i< 10; i++) {Serial.print(serialData[i], HEX); Serial.print("-"); } Serial.println();
      av-= 9;

      if (av < 9) executeCommand(serialData);
    }*/
      //Serial.println(serialData);
      
      
      //executeCommand(serialData);
    //}
    
}

