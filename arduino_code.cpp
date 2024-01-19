/* Version 1.0 - 19/01/2024
   Made by Sabri Chamtouri
   gitHub portfolio: https://github.com/sabricham
   gitHub full documentation & schematics: https://github.com/sabricham/music-nfc-box

   Components used:
   - Arduino Nano
   - PN532 NFC reader module from NXP
   - NTAG215 NFC cards from NXP
   - DFPlayer Mini mp3 module
   - Logitech Z150 Audio amplifier & 2x 2" Audio drivers
   - 2GB Micro SD Card
   - 5VDC-800mA Power Supply unit

   Description:
   This program, to load into Arduino Nano, communicates with a PN532 module to scan for nfc tags and whenever one appears analyzes it's payload 
   and starts a specific music album using a DFPlayer Mini connected to the audio amplifier board from Logitech Z150 which drives 2x 2" Audio drivers
   The nfc tags are NTAG215 cards with an image on each visualizing the album code it contains 

   Requirements
   - PN532 library required from: https://github.com/elechouse/PN532 (PN532 I2C & PN532 folders for I2C config, download and zip them separately before importing in arduino)
   - NDEF library required from: https://github.com/don/NDEF (download and import zip file)
   - DFPlayer Mini library required from: https://github.com/DFRobot/DFRobotDFPlayerMini (download and import zip file)
   - NFC tags must be written previously, format used in this project: URL / 1 record only / 5 numbers -> ex. 01002 will mean folder #1 & song #2
     to write onto NTAG215 you can use NXP TagWriter smartphone application available for Android & Iphone devices 

   Important notes:
   - PN532 used in I2C configuration: SDA pin to A4, SCL pin to A5 on Arduino Nano (every arduino has it's own specific SDA/SCL pins)
   - DFPlayer Mini works in UART SoftwareSerial configuration: RX pin to D2, TX pin to D3 (connected directly)
   - DFPlayer Mini requires a 1k resistor in series between Arduino TX (pin D2 in this example) and module RX, otherwise UART communication might not work (from datasheet)
   - DFPlayer Mini requires a specific naming structure for the SD Card: (from datasheet)
      -01  [folder: 2 digits zero-filled number]
        -001.mp3  [song: 3 digits zero-filled number]
        -002.mp3
      -02
        -001.mp3
        -002.mp3
*/
#include <SoftwareSerial.h>
#include <string.h>

#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>
#include <DFRobotDFPlayerMini.h>

#define dfplayer_tx 2
#define dfplayer_rx 3

#define btn_shuffle 4
#define btn_pause_play 5
#define btn_next 6
#define btn_previous 7

//DFPlayer module
SoftwareSerial softSerial(dfplayer_rx, dfplayer_tx);
DFRobotDFPlayerMini dfPlayer;
void printDetail(uint8_t type, int value);

//PN532 module
PN532_I2C pn532_i2c(Wire);
NfcAdapter pn532 = NfcAdapter(pn532_i2c);
String tagId = "None";
byte nuidPICC[4];

//NFC tag variables
uint8_t selected_song;
uint8_t selected_folder;
bool select_song = false;

//Music player structure
int num_songs[6] = {15, 13, 14, 11, 10, 13}; 
//num songs per album in the SD card (my own initialization),
//first album null: counting starts from 1 for DFPlayer

bool play=false;

void setup(void) 
{
  Serial.begin(115200);
  Serial.println("System initialized");  

  //DFPlayer initialization
  softSerial.begin(9600);  
  if (!dfPlayer.begin(softSerial, true, true)) {  
    Serial.println("Unable to begin DFPlayer:\nplease check for connection issues and try reinsert SD card\n");
    while(true){
      delay(1);
    }
  }
  else {
    Serial.println("DFPlayer module: online");
    dfPlayer.volume(20);
  }  

  //PN532 initialization
  pn532.begin();

  //buttons initialization
  pinMode(btn_shuffle, OUTPUT);
  pinMode(btn_pause_play, OUTPUT);
  pinMode(btn_next, OUTPUT);
  pinMode(btn_previous, OUTPUT);
}

void loop() 
{
  readNFC();
  checkButtons();
  updatePlayer();
}

void updatePlayer()
{
  //flag select_song is activated when an nfc tag, with the correct payload format, has been read
  if(select_song)
  {
    dfPlayer.playFolder(selected_folder, selected_song);
    select_song=false;
  }
}

void identifyPayload(char *payload, int payload_length)
{
  //correct payload format (created using URL type writing): [XNNNNNO], X ignored, N number, O not present
  //example: [X01002] = folder #1, song #2
  //folder #0 and song #0 cannot be chosen, counting starts from 1 for DFPlayer: first song is #1 & first folder is #1
  //ignore first character, check length to be == 6 and check that characters from [1] to [6] are digits
  int count=6;
  int i;
  for(i=1; i<payload_length; i++)
    if(!isdigit(payload[i])) break;

  //check if payload format correct
  if(i==count) {
    //get folder number: digits [1],[2]
    selected_folder=(uint8_t)((payload[1]-'0')*10 + (payload[2]-'0'));

    //get song number: digits [3],[4],[5]
    selected_song=(uint8_t)((payload[3]-'0')*100 + (payload[4]-'0')*10 + (payload[5]-'0'));

    //check if payload matches for the SD Card data
    if(selected_folder > 0 && selected_folder <= sizeof(num_songs)/sizeof(num_songs[0]) && selected_song > 0 && selected_song <= num_songs[selected_folder - 1])
    {
      //activate flag
      select_song=true;

      //debug
      Serial.print("Payload correct: folder-");
      Serial.print(selected_folder);
      Serial.print(" song-");
      Serial.println(selected_song);
    }
    else Serial.println("Payload inconsistent");
  }
  else Serial.println("Payload not correct");
}

void readNFC() 
{
  if (pn532.tagPresent())
  {
    NfcTag tag = pn532.read();

    //nfc tag must have one only record
    if(tag.getNdefMessage().getRecordCount()==1)
    {
      //retrieve payload from the nfc tag
      int payload_length;
      byte *payload;
      payload_length = tag.getNdefMessage().getRecord(0).getPayloadLength();

      payload=(byte *)malloc(payload_length*sizeof(byte));
      tag.getNdefMessage().getRecord(0).getPayload(payload);

      identifyPayload((char *)payload, payload_length);
      
      updatePlayer();

      delay(1000); //avoid reading more than once a tag 
    }   
  }
}