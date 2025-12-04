#include "esp32_e220900t22s_jp_lib.h"
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>


CLoRa lora;


/** prototype declaration **/
// The task functions are now regular functions called directly from loop()
void LoRaRecv(void); 
void LoRaSend(void);
void ReadDataFromConsole(char *msg, int max_msg_len);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);

  delay(1000); // Serial init wait
  Serial.println();

  lora.SetDefaultConfigValue(lora.config);

  Serial.println(lora.config.air_data_rate);
  lora.SetBWSF(125,7);
  lora.SetRSSIflag(1);
  Serial.println(lora.config.air_data_rate);
 
  // E220-900T22S(JP)へのLoRa初期設定
  if (lora.InitLoRaModule(lora.config)) {
    Serial.printf("init error\n");
    return;
  }

  Serial.println(lora.config.air_data_rate);

  // ノーマルモード(M0=0,M1=0)へ移行する
  lora.SwitchToNormalMode();

}

unsigned long time_lastsent = 0;

void loop() {
  // Call the receive and send logic sequentially in the main loop
  LoRaRecv();

  LoRaSend();

  // A small delay ensures the system remains responsive and doesn't hog the CPU.
  delay(1); 
  if(millis() - time_lastsent > 200 && true){
    time_lastsent = millis();
    sendAck();
  }
}

int receivecounter = 0;
// Renamed and repurposed from LoRaRecvTask
void LoRaRecv(void) {
  if (lora.ReceiveFrame(&lora.data) == 0) {
    receivecounter++;
    Serial.print("RecieveFrame ");
    Serial.println(receivecounter);
    Serial.printf("recv data:\n");
    for (int i = 0; i < lora.data.recv_data_len; i++) {
      Serial.printf("%c", lora.data.recv_data[i]);
    }
    Serial.printf("\n");
    Serial.printf("hex dump:\n");
    for (int i = 0; i < lora.data.recv_data_len; i++) {
      Serial.printf("%02x ", lora.data.recv_data[i]);
    }
    Serial.printf("\n");
    Serial.printf("RSSI: %d dBm\n", lora.data.rssi);
    Serial.printf("\n");

    Serial.flush();

    if((char)lora.data.recv_data[0] == 'r')
      sendAck();
  }
}

void sendAck(void){
  char msg[200] = "acknowledge\0";
  // Only send if ReadDataFromConsole actually put something in msg
  if (lora.SendFrame(lora.config, (uint8_t *)msg, strlen(msg)) == 0) {
    Serial.printf("ack succeeded.\n");
    Serial.printf("\n");
  } else {
    Serial.printf("ack failed.\n");
    Serial.printf("\n");
  }
  Serial.flush();
}

// Renamed and repurposed from LoRaSendTask
void LoRaSend(void) {
  char msg[200] = {0};

  // Check if serial data is available before trying to read and send
  if (Serial.available()) { 
      // ESP32がコンソールから読み込む
      ReadDataFromConsole(msg, (sizeof(msg) / sizeof(msg[0])));
      
      // Only send if ReadDataFromConsole actually put something in msg
      if (strlen(msg) > 0) {
          if (lora.SendFrame(lora.config, (uint8_t *)msg, strlen(msg)) == 0) {
            Serial.printf("send succeeded.\n");
            Serial.printf("\n");
          } else {
            Serial.printf("send failed.\n");
            Serial.printf("\n");
          }
      }
      Serial.flush();
  }
}

void ReadDataFromConsole(char *msg, int max_msg_len) {
  int len = 0;
  char *start_p = msg;


  while (len < max_msg_len) {
    if (Serial.available() > 0) {
      char incoming_byte = Serial.read();
      if (incoming_byte == 0x00 || incoming_byte > 0x7F)
        continue;
      *(start_p + len) = incoming_byte;
      // 最短で3文字(1文字 + CR LF)
      if (incoming_byte == 0x0a && len >= 2 && (*(start_p + len - 1)) == 0x0d) {
        break;
      }
      len++;
    } else if (len > 0) {
        // If we started reading but Serial runs dry before we get CR/LF, 
        // we break to prevent a long block.
        break;
    } else {
        // If we haven't started reading, exit immediately to keep loop() fast.
        // This relies on LoRaSend() checking Serial.available() first.
        break; 
    }
    // Minimal delay to yield control if reading a long line
    delay(1); 
  }

  // msgからCR LFを削除
  // Find the first CR (0x0D) or LF (0x0A) and terminate the string there.
  for (int i = 0; i < len; i++) {
    if (msg[i] == 0x0D || msg[i] == 0x0A) {
      msg[i] = '\0';
      break; 
    }
  }
}