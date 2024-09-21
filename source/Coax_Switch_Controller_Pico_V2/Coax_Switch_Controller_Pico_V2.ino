/*
  _  _  ___  ___  ___   __   ___     _   _  _  __   ___  __  _    _ 
 ( )( )(  _)(__ \(_  ) (  ) (_  )   / ) ( )( )(  ) (__ \(  )( \/\/ )
  \\//  ) _)/ __/ / /  /__\  / /   / /   \\// /__\ / __/ )(  \    / 
  (__) (___)\___)(___)(_)(_)(___) (_/    (__)(_)(_)\___)(__)  \/\/                                                                                          

  Smart Controller for remote antenna switch (Ameritron or similar), Version 2.
  Designed to be run on a Raspberry Pi Pico. Developed in Arduino IDE.
  VE2ZAZ/VA2IW, September 2024. https://ve2zaz,net
  Usage licence: Attribution 4.0 International — CC BY 4.0 - Creative Commons. Read https://creativecommons.org/licenses/by/4.0/deed.en

Release History
===============
V2: 09/2024 Corrected erratic channel number behavior in when in relay bit mode (due to an if statment with an "=" instead of a '==' !!!)
V1: 07/2023 - Initial Release
*/

// COMPILER DIRECTIVES
#include "Adafruit_ILI9341.h"
#include "Adafruit_GFX.h"
#include <XPT2046_Touchscreen.h>
#include <EEPROM.h>
// Pins used on the Rasprerry Pi Pico. Numbers correspond to GPxx pin assignment.
#define TFT_DC 21
#define TFT_CS 17
#define TFT_MOSI 19
#define TFT_CLK 18
#define TFT_RST 20
#define TFT_MISO 16
#define TCS_PIN  13
#define TIRQ_PIN 14
#define control_eclair_affich 5
#define Relay_A 4
#define Relay_B 6
#define Relay_C 7
#define Relay_D 8
#define Relay_E 9
#define Relay_F 10
#define Relay_G 11
#define Relay_H 12
#define PTT_In 2
#define Buzzer 22
#define TX_Delay_Out 3

// EEPROM Base addresses
#define Tx_Antenna_Base_Addr 0
#define Rx_Antenna_Base_Addr 1
#define Ant_Data_Base_Addr 2
#define Output_Mode_Base_Addr 202
#define Num_Channels_Base_Addr 205
#define Protection_Xfer_Base_Addr 206
#define Tx_Delay_Base_Addr 207

// GLOBAL VARIABLES

//String Antenna_Data[8][4] {{"1 Dummy Load    ",   "TX-RX",   "0",  "0"},               // Le texte qui apparaîtra sur les étiquettes pour chaque antenna. Maximum 16 charactères
//                           {"2 20/15/10 Yagi ",   "TX-RX",   "11", "40"},
//                           {"3 40/80 H-Sloper",   "TX-RX",   "1",  "11"},              //"TX-RX" = Antenne utilisable en TX
//                           {"4 6m Moxon      ",   "TX-RX",   "40", "65"},              //"RX" = Antenne utilisable seulement en Rx
//                           {"5 Loop-On-Ground",   "RX",      "0",  "0"},               // Les deux nombres: fréquences minimum (>=) et maximum (<=).
//                           {"6 Mag Loop      ",   "RX",      "0",  "0"},               //   0 et 0 rendent une antenna manuelle, i.e. jamais sélectionnée automatiquement.
//                           {"7 Long Wire     ",   "TX-RX",   "0",  "0"},
//                           {"8    -----      ",   "RX",      "0",  "0"}};
String Antenna_Data[8][4];                                      // Antenna data array in RAM

unsigned long int init_display_delay_millis = 0;                // Initial millis() function values to calculate time delays
unsigned long int init_flashing_delay_millis = 0;               //      "
unsigned long int init_refresh_delay_millis = 0;                //      "
unsigned long int init_tx_delay_millis = 0;
unsigned int antenna_Tx = 0;                                    // Selected Tx antenna from 0 to 7
unsigned int antenna_Tx_old;                                    // Previously selected Tx antenna from 0 to 7
unsigned int new_antenna;                                       // Buffer to temporarely save the new antenna value
unsigned int antenna_Rx = 0;                                    // Selected Rx antenna from 0 to 7
double avg_px = 0;                                              // Touch position variables
double avg_py = 0;                                              //      "
int avg_count = 0;                                              //      "
bool ptt_active = false;                                        // Push-To-Talk state (Tx=1, Rx=0)
bool alarm_antenna_rx = false;                                  // Alarm flag for transmitting on a Rx-only antenna
bool flashing_on;                                               // "TX" label flashing flag
char charact;                                                   // Character received on the user serial port
String rx_data;                                                 // Character string received on the user serial port
unsigned int civ_charact;                                       // Character received on the CI-V port
unsigned int civ_rx_data[15];                                   // Character string received on the CI-V port
unsigned int civ_byte_pos = 0;                                  // Character position in the Character string received on the CI-V port
unsigned int frequency;                                         // Transceiver frequency, expressed in MHz, and truncated to the MHz
int old_frequency = -1;                                         // Previous transceiver frequency
bool new_freq_avail = false;                                     // New frequency available flag
String output_mode = "BCD";                                     // Output mode string variable, contains either "BCD" or "BIT"
int num_channels = 8;                                           // Number of antenna ports configured
bool freq_range_error = false;                                  // Flags whether there is an overlap in frequency between the different antennas
bool validation_error = false;                                  // Flags whether there is an error in the antenna data received
String protect_to_ch1 = "Y";                                    // Protect-to-channel-1 string variable, contains either "Y" or "N"
unsigned int i,j;                                               // In-loop couter functions used throughout
unsigned char tx_delay = 25;                                     // The transmit delay variable containing the delay in ms.
bool old_ptt_active = false;

// CLASS INSTANTIATIONS
// Built-in SPI (hardware port) of R-Pi Pico for the display. Position for CS, DC and RST pins must be specified.
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
// Tactile (touch) function is implemented using the IRQ pin
XPT2046_Touchscreen ts(TCS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling

// Function that produces a 30ms sound beep via the magnetic transducer
void buzzer_One_Beep()
{
  tone(Buzzer, 2400);                                           // Produce a 2400 Hz tone, the suggested frequency for the magnetic transducer
  init_refresh_delay_millis = millis();                         // A 30 ms delay
  while (millis() <= (init_refresh_delay_millis + 30));         //      "
  noTone(Buzzer);                                               // Stop the tone
}

// Function that draws the antenna banners and descriptions on screen
void draw_Ant_Banners()
{
  tft.fillScreen(ILI9341_BLACK);                                // Clear the screen to black
  tft.setTextSize(2);                                           // Set the font size
  tft.setTextColor(ILI9341_WHITE);                              // Set the font color
  for (i = 0; i < num_channels; i++)                            // Loop as many times as there are antenna ports defined
  {
    tft.fillRect(16, 10 + i * 28, 200, 23, ILI9341_BLUE);       // Draw the blue banner
    tft.setCursor(20, 15 + i * 28);                             // Write the Antenna description text
    tft.print(Antenna_Data[i][0]);                              //      "
  }
}

// Fuction that configures the output pins based on number of channels and type of outputs desired 
void config_Outputs()
{
  antenna_Tx = 0;                                         // Set the Rx and Tx antennas to 0 (antenna 1)
  antenna_Rx = 0;                                         //    "
  digitalWrite(Relay_A, LOW);                             // Set all 8 ports to low level (default level)
  digitalWrite(Relay_B, LOW);                             //    "
  digitalWrite(Relay_C, LOW);                             //    "
  digitalWrite(Relay_D, LOW);                             //    "
  digitalWrite(Relay_E, LOW);                             //    "
  digitalWrite(Relay_F, LOW);                             //    "
  digitalWrite(Relay_G, LOW);                             //    "
  digitalWrite(Relay_H, LOW);                             //    "
  // Set the proper pins as outputs, based on the number of channels set
  if (output_mode == "BIT")                               // In bit (individual relay) mode?
  {   // Yes
    pinMode(Relay_A, OUTPUT);                             // The first two relay lines are always activated
    pinMode(Relay_B, OUTPUT);                             //    "
    if (num_channels >= 3) pinMode(Relay_C, OUTPUT);      // Activate the following relay based on whether the channel is activated
    else pinMode(Relay_C, INPUT);                         //    "
    if (num_channels >= 4) pinMode(Relay_D, OUTPUT);      //    "
    else pinMode(Relay_D, INPUT);                         //    "
    if (num_channels >= 5) pinMode(Relay_E, OUTPUT);      //    "
    else pinMode(Relay_E, INPUT);                         //    "
    if (num_channels >= 6) pinMode(Relay_F, OUTPUT);      //    "
    else pinMode(Relay_F, INPUT);                         //    "
    if (num_channels >= 7) pinMode(Relay_G, OUTPUT);      //    "
    else pinMode(Relay_G, INPUT);                         //    "
    if (num_channels == 8) pinMode(Relay_H, OUTPUT);       //   "  Here, there was a '=' instead of a '==' in version 1. Corrected in this version 2!
    else pinMode(Relay_H, INPUT);                         //    "
  }
  else                                                    // In BCD mode
  {
    pinMode(Relay_A, OUTPUT);                             // The first BCD line is always activated
    if (num_channels >= 3) pinMode(Relay_B, OUTPUT);      // Activate the following BCD line based on whether the channels are using it
    else pinMode(Relay_B, INPUT);                         //    "
    if (num_channels >= 5) pinMode(Relay_C, OUTPUT);      // Activate the following BCD line based on whether the channels are using it
    else pinMode(Relay_C, INPUT);                         //    "
    pinMode(Relay_D, INPUT);                              // De-activate the remaining output lines
    pinMode(Relay_E, INPUT);                              //    "
    pinMode(Relay_F, INPUT);                              //    "
    pinMode(Relay_G, INPUT);                              //    "
    pinMode(Relay_H, INPUT);                              //    "
  }
}

// Initialization function for the Pico's first core, Arduino style, executed only once at power up
void setup()
{
  pinMode(TCS_PIN, OUTPUT);                               // Ensure that the CS lines are at an output high logic level
  digitalWrite(TCS_PIN, HIGH);                            //    "
  pinMode(TFT_CS, OUTPUT);                                //    "
  digitalWrite(TFT_CS, HIGH);                             //    "
  pinMode(PTT_In, INPUT);                                 // The PTT_In pin is initialized as an input
  pinMode(control_eclair_affich, OUTPUT);                 // The Display LED lighting control pin is initialized at an output high logic level
  digitalWrite(control_eclair_affich, HIGH);              //    "
  digitalWrite(TX_Delay_Out, HIGH);                       // The Tx delay output pin (provides a delayed PTT connection
  pinMode(TX_Delay_Out, OUTPUT);         
  config_Outputs();                                       // Configure the relay output lines
  tft.begin(24000000);                                    // Start the screen instance
  tft.setRotation(3);                                     // Rotate the displayed data by 270 degrees
  ts.begin();                                             // Start the touch instance
 
  Serial.begin(9600);                                     // USB serial port (command interface)
  EEPROM.begin(256);                                      // Start the EEPROM function (Flash memory emulation) and reserve 256 bytes

  while (!Serial && (millis() <= 1000));                  // A one second pause

  // Retrieve Rx and Tx antenna values from EEPROM
  antenna_Tx = EEPROM.read(Tx_Antenna_Base_Addr);         // Read Tx antenna number
  antenna_Rx = EEPROM.read(Rx_Antenna_Base_Addr);         // Read Rx antenna number
//Serial.println(EEPROM.read(Tx_Antenna_Base_Addr));                                            //     "  
//Serial.println(EEPROM.read(Rx_Antenna_Base_Addr));                                            //     "  
  if (antenna_Tx > 7)                                     // Previous data exists in EEPROM (empty if value = 0xFF)?
  {                                                       // No
    antenna_Tx = 0;                                       // Initializa both antennas to zero (port 1) 
    antenna_Rx = 0;                                       //     "
    EEPROM.write(Tx_Antenna_Base_Addr, antenna_Tx);       // Save both antenna values into EEPROM
    EEPROM.write(Rx_Antenna_Base_Addr, antenna_Rx);       //     "
    for (j = 0; j < num_channels; j++) Antenna_Data[j][0] = "Antenna " + String(j+1) + "       ";     // Initialize the antenna data to default values
    for (j = 0; j < num_channels; j++) Antenna_Data[j][1] = "TX-RX";                                  //     "
    for (j = 0; j < num_channels; j++) Antenna_Data[j][2] = "0 ";                                     //     "
    for (j = 0; j < num_channels; j++) Antenna_Data[j][3] = "0 ";                                     //     "
    // Transfer the antenna data to EEPROM
    for (i = 0; i < num_channels; i++)                                                                // Loop as many times as there are antenna ports
    {
      for (j = 0; j < 16; j++)  EEPROM.write(Ant_Data_Base_Addr + (i * 25) + j, Antenna_Data[i][0].charAt(j));        // Save the antenna description
      for (j = 16; j < 21; j++) EEPROM.write(Ant_Data_Base_Addr + (i * 25) + j, Antenna_Data[i][1].charAt(j - 16));   // Save the antenna type
      for (j = 21; j < 23; j++) EEPROM.write(Ant_Data_Base_Addr + (i * 25) + j, Antenna_Data[i][2].charAt(j - 21));   // Save the minimum frequency
      for (j = 23; j < 25; j++) EEPROM.write(Ant_Data_Base_Addr + (i * 25) + j, Antenna_Data[i][3].charAt(j - 23));   // Save the maximum frequency
    }
    for (j = 0; j < 3; j++) EEPROM.write(Output_Mode_Base_Addr + j, output_mode.charAt(j));           // Save the output mode
    EEPROM.write(Num_Channels_Base_Addr, '8');                                                        // Save the number of antenna ports
    EEPROM.write(Protection_Xfer_Base_Addr, 'Y');                                                     // Save the protect-to-channel-1 feature
    EEPROM.write(Tx_Delay_Base_Addr, tx_delay);                                                     // Save the protect-to-channel-1 feature
    EEPROM.commit();                                                                                  // Finally, commit to EEPROM
  }
  else    // Valid data already exists in EEPROM
  { 
    for (i = 0; i < num_channels; i++)                                                                // Loop as many times as there are antenna ports
    {
      Antenna_Data[i][0] = "";                                                                        // Retrieve antenna description from EEPROM
      for (j= 0; j < 16; j++) Antenna_Data[i][0] += char(EEPROM.read(Ant_Data_Base_Addr+(i*25)+j));   //      "
      Antenna_Data[i][1] = "";                                                                        // Retrieve antenna type from EEPROM
      for (j= 16; j < 21; j++) Antenna_Data[i][1] += char(EEPROM.read(Ant_Data_Base_Addr+(i*25)+j));  //      "
      Antenna_Data[i][2] = "";                                                                        // Retrieve antenna minimum frequency from EEPROM
      for (j= 21; j < 23; j++) Antenna_Data[i][2] += char(EEPROM.read(Ant_Data_Base_Addr+(i*25)+j));  //      "
      Antenna_Data[i][3] = "";                                                                        // Retrieve antenna maximum frequency from EEPROM
      for (j= 23; j < 25; j++) Antenna_Data[i][3] += char(EEPROM.read(Ant_Data_Base_Addr+(i*25)+j));  //      "
    }
    output_mode = String(char(EEPROM.read(Output_Mode_Base_Addr)))                                    // Retrieve Output mode from EEPROM
                + String(char(EEPROM.read(Output_Mode_Base_Addr + 1)))                                //      "
                + String(char(EEPROM.read(Output_Mode_Base_Addr + 2)));                               //      "
    num_channels = char(EEPROM.read(Num_Channels_Base_Addr)) - '0';                                   // Retrieve the number of ports from EEPROM
    protect_to_ch1 = char(EEPROM.read(Protection_Xfer_Base_Addr));                                    // Retrieve the protect-to-channel-1 feature from EEPROM
    tx_delay = EEPROM.read(Tx_Delay_Base_Addr);  
  }  
  draw_Ant_Banners();                                               // Draw the banners and antenna descriptions on screen
}

// Initialization function for the Pico's second core, Arduino style, executed only once at power up
void setup1()
{
  Serial1.begin(4800);                                              // CI-V serial port to transceiver

  // Send initial frequency request. If not answered (in Loop1), antennas defined in EEPROM still apply
  const byte tx_data[] = {0xFE, 0xFE, 0x94, 0xE0, 0x03, 0xFD};      // Frequency request CI-V message syntax
  Serial1.write(tx_data, sizeof(tx_data));                          // Send request to transceiver
}

// Main loop function running on the Pico's first core, Arduino style.
void loop()
{
  // PTT read and Rx-only antenna management
  if (ptt_active != !digitalRead(PTT_In))                           // Is level on PTT_In pin different from PTT variable?
  {
    ptt_active = !digitalRead(PTT_In);                              // Yes, change PTT variable state
    if (ptt_active && (Antenna_Data[antenna_Tx][1] == "RX   "))     // In Tx and on receive-only antenna?
    {                                                               // Yes  
      alarm_antenna_rx = true;                                      // Flag an antenna alarm
      if (protect_to_ch1 == "Y")                                    // Is protect-to-channel-1 feature enabled?
      {                                                             // Yes
        antenna_Tx_old = antenna_Tx;                                // Save previous Tx antenna
        antenna_Tx = 0;                                             // Change current Tx antenna to antenna 0 (dummy load)
      }
    }
    else if (alarm_antenna_rx)                                      // Back in Rx and with active antenna alarm?
    {
      alarm_antenna_rx = false;                                     // Deactivate alarm
      if (protect_to_ch1 == "Y") antenna_Tx = antenna_Tx_old;       // In case of protect-to-channel-1 feature enabled, recover original Tx antenna
    }
  }

  // Update antenna relays
  if (ptt_active)                                                   // PTT active?
  {                                                                 // Yes
    if (output_mode == "BIT")                                       // In bit (individual relay) output mode? 
    {                                                               // Yes
      digitalWrite(Relay_A, (antenna_Tx == 0) ? 1 : 0 );            // Select the proper relay line based on current Tx antenna
      digitalWrite(Relay_B, (antenna_Tx == 1) ? 1 : 0 );            //     "  
      digitalWrite(Relay_C, (antenna_Tx == 2) ? 1 : 0 );            //     "  
      digitalWrite(Relay_D, (antenna_Tx == 3) ? 1 : 0 );            //     "
      digitalWrite(Relay_E, (antenna_Tx == 4) ? 1 : 0 );            //     "  
      digitalWrite(Relay_F, (antenna_Tx == 5) ? 1 : 0 );            //     "  
      digitalWrite(Relay_G, (antenna_Tx == 6) ? 1 : 0 );            //     "  
      digitalWrite(Relay_H, (antenna_Tx == 7) ? 1 : 0 );            //     "  
    }
    else                                                            // In BCD output mode
    {
      digitalWrite(Relay_A, bitRead(antenna_Tx, 0));                // Select the proper BCD lines based on current Tx antenna
      digitalWrite(Relay_B, bitRead(antenna_Tx, 1));                //     "
      digitalWrite(Relay_C, bitRead(antenna_Tx, 2));                //     "
    }
  }
  else                                                              // PTT inactive (in Rx)
  {                                                                 // Yes
//Serial.println(antenna_Tx);                                            //     "  
    if (output_mode == "BIT")                                       // In bit (individual relay) output mode? 
    {                                                               // Yes
      digitalWrite(Relay_A, (antenna_Rx == 0) ? 1 : 0 );            // Select the proper relay line based on current Rx antenna
      digitalWrite(Relay_B, (antenna_Rx == 1) ? 1 : 0 );            //     "    
      digitalWrite(Relay_C, (antenna_Rx == 2) ? 1 : 0 );            //     "    
      digitalWrite(Relay_D, (antenna_Rx == 3) ? 1 : 0 );            //     "  
      digitalWrite(Relay_E, (antenna_Rx == 4) ? 1 : 0 );            //     "    
      digitalWrite(Relay_F, (antenna_Rx == 5) ? 1 : 0 );            //     "    
      digitalWrite(Relay_G, (antenna_Rx == 6) ? 1 : 0 );            //     "    
      digitalWrite(Relay_H, (antenna_Rx == 7) ? 1 : 0 );            //     "    
    }
    else                                                            // In BCD output mode
    {
      digitalWrite(Relay_A, bitRead(antenna_Rx, 0));                // Select the proper BCD lines based on current Tx antenna
      digitalWrite(Relay_B, bitRead(antenna_Rx, 1));                //     "
      digitalWrite(Relay_C, bitRead(antenna_Rx, 2));                //     "
    }
  }

  // manage the Tx delay output
  if (antenna_Rx != antenna_Tx)                                       // Are transmit and receive antennas different?
  {
    if (ptt_active)                                                   // PTT active?
    {                                                                 // Yes
      if (!old_ptt_active)                                            // Was PTT active before?
      {                                                               // No 
        old_ptt_active = true;                                        // Set the previous ptt active flag
        init_tx_delay_millis = millis();                              // Rre-initialize the Tx delay timer value
      }
      else if ((unsigned long)(millis() - init_tx_delay_millis) >= (tx_delay)) // Tx delay has reached configured delay?
                 digitalWrite(TX_Delay_Out, LOW);                     // Tes, release the Tx delay output
    }
    else
    {
      digitalWrite(TX_Delay_Out, HIGH);                               // Operate the Tx delay output
      old_ptt_active = false;                                         // Reset the previous ptt active flag
    }
  }
  else digitalWrite(TX_Delay_Out, LOW);                               // transmit and receive antennas are the same, de-activate delay output
  
  // Refresh TX and RX boxes (buttons) based on PTT status
  for (i = 0; i < num_channels; i++)                                // Loop as many times as there are antenna ports
  {
    if (ptt_active)                                                 // Active PTT? TX boxes are in red, Rx in grey
    {
      tft.drawRect(226, 10 + i * 28, 36, 23, ILI9341_RED);          // Draw TX buttons (boxes)
      tft.drawRect(270, 10 + i * 28, 36, 23, 0x4108);               // Very dark grey. RGB=5bits/6bits/5bits
    }
    else                                                            // Inactive PTT, Rx boxes in green, Tx in grey
    {
      tft.drawRect(226, 10 + i * 28, 36, 23, 0x4108);               // Very dark grey. RGB=5bits/6bits/5bits
      tft.drawRect(270, 10 + i * 28, 36, 23, ILI9341_GREEN);        // Draw RX buttons (boxes)
    }
  }

  // "TX" and "RX" label display
  for (i = 0; i < num_channels; i++)                                // Loop as many times as there are antenna ports
  {
    // TX label management
    tft.setTextColor(ILI9341_RED);                                  // "TX" label in red
    tft.setCursor(232, 15 + i * 28);                                // Set cursor in proper box
    if (i == antenna_Tx)                                            // Write TX label only on active Tx antenna
    {
      if ((Antenna_Data[i][1] == "TX-RX")) tft.print("TX");         // Write "TX" in box corresponding to selected Tx antenna only if antenna is of TX-RX type
      else tft.print("--");                                         // Otherwise write "--" (Rx-only type antenna)
    }
    else tft.fillRect(232, 15 + i * 28, 25, 16, ILI9341_BLACK);     // Erase other Tx box contents
    if ((alarm_antenna_rx) && (!flashing_on)) tft.fillRect(232, 15 + i * 28, 25, 16, ILI9341_BLACK);  // Produces label flashing when Tx alarm is active

    // RX label management                              
    tft.setTextColor(ILI9341_GREEN);                                // "RX" label in green
    tft.setCursor(277, 15 + i * 28);                                // Set cursor in proper box
    if (i == antenna_Rx) tft.print("RX");                           // Write "RX" in box corresponding to selected Rx antenna
    else tft.fillRect(277, 15 + i * 28, 25, 16, ILI9341_BLACK);     // Erase other Rx box contents
  }
  // Tactile (Touch) action management
  if (ts.touched())                                                 // Display touched?
  {                                                                 // Yes
    if (!digitalRead(control_eclair_affich))                        // Is display LED lighting off?
    {                                                               // Yes
      init_display_delay_millis = millis();                         // Save initial timer value for LED lighting
      digitalWrite(control_eclair_affich, HIGH);                    // Turn on LED lighting
      while (ts.touched());                                         // Wait until touch is released
    }
    while (ts.touched())                                            // Is touch happening?
    {   // Présentement touché                                      // Yes
      TS_Point p = ts.getPoint();                                   // Retrieve the touch x and y coordinates
      avg_px += p.x;                                                // Accumulate the coordinate for average coordinate claculation (further down)
      avg_py += p.y;                                                //     "
      avg_count++;                                                  // Increment the average counter
    }
    init_refresh_delay_millis = millis();                           // Insert a 100ms pause
    while (millis() <= (init_refresh_delay_millis + 100));          //      "
    avg_px = round(float(avg_px) / avg_count);                      // Calculate average x and y coordinates
    avg_py = round(float(avg_py) / avg_count);                      //      "

    // Selected TX button decoding as fuction of touch coordinates
    if ((avg_px > 16 && avg_px <= 3200) && (!ptt_active))           // Touch in TX boxes column zone?
    {
      if      (avg_py > 300 && avg_py <= 675)   new_antenna = 0;    // Yes, Decoding of which Tx antenna box is touched. 
      else if (avg_py > 676 && avg_py <= 1125)  new_antenna = 1;    //     "
      else if (avg_py > 1126 && avg_py <= 1575) new_antenna = 2;    //     "
      else if (avg_py > 1576 && avg_py <= 2025) new_antenna = 3;    //     "
      else if (avg_py > 2026 && avg_py <= 2475) new_antenna = 4;    //     "
      else if (avg_py > 2476 && avg_py <= 2925) new_antenna = 5;    //     "
      else if (avg_py > 2926 && avg_py <= 3375) new_antenna = 6;    //     "
      else if (avg_py > 3376 && avg_py <= 3750) new_antenna = 7;    //     "
      if (new_antenna <= (num_channels-1))                          // TX box touched corresponds to enabled antenna port?  
      {                                                             // Yes
        antenna_Tx = new_antenna;                                   // Assign new Tx antenna
        antenna_Rx = antenna_Tx;                                    // Rx antenna follows Tx antenna when Tx antenna is changed
        EEPROM.write(Tx_Antenna_Base_Addr, antenna_Tx);             // Save both antenna values into EEPROM
        EEPROM.write(Rx_Antenna_Base_Addr, antenna_Rx);             //     "
        EEPROM.commit();                                            // Commit changes to EEPROM
        buzzer_One_Beep();                                          // Emit audible beep once      
      }
    }
    // Selected RX button decoding as fuction of touch coordinates
    else if (avg_px > 3300 && avg_px <= 3700)                       // Touch in RX boxes column zone?
    {
      if      (avg_py > 300 && avg_py <= 675)  new_antenna = 0;     // Yes, Decoding of which Rxantenna box is touched. 
      else if (avg_py > 676 && avg_py <= 1125)  new_antenna = 1;    //     "
      else if (avg_py > 1126 && avg_py <= 1575) new_antenna = 2;    //     "
      else if (avg_py > 1576 && avg_py <= 2025) new_antenna = 3;    //     "
      else if (avg_py > 2026 && avg_py <= 2475) new_antenna = 4;    //     "
      else if (avg_py > 2476 && avg_py <= 2925) new_antenna = 5;    //     "
      else if (avg_py > 2926 && avg_py <= 3375) new_antenna = 6;    //     "
      else if (avg_py > 3376 && avg_py <= 3750) new_antenna = 7;    //     "
      if (new_antenna <= (num_channels-1))                          // RX box touched corresponds to enabled antenna port?  
      {
        antenna_Rx = new_antenna;                                   // Assign new Rx antenna
        EEPROM.write(Rx_Antenna_Base_Addr, antenna_Rx);             // Commit changes to EEPROM
        EEPROM.commit();                                            // Sauvegarde finale ici
        buzzer_One_Beep();                                          // Emit audible beep once      
      }
    }
    avg_px = 0;                                                     // Clear average coordinate calculation variables
    avg_py = 0;                                                     //     "
    avg_count = 0;                                                  //     "
  }

  // "TX" flashing management
  if ((unsigned long)(millis() - init_flashing_delay_millis) >= 150)    // TX label flashing time has reached 150 ms?
  {
    init_flashing_delay_millis = millis();                              // Yes, re-initialize the initial flashing timer value
    flashing_on = !flashing_on;                                         // invert the flashing state
  }

  // Buzzer management during a TX alarm
  if ((alarm_antenna_rx) && (flashing_on)) buzzer_One_Beep();           // If both Tx alarm and flasing state are active, emit one beep

  // Display backlight shutdown management
  if ((unsigned long)(millis() - init_display_delay_millis) >= 3600000) // Display LED shut off time has reached 1 hour?
  {
    digitalWrite(control_eclair_affich, LOW);                           // Yes, Turn off display lighting
    init_display_delay_millis = millis();                               // Yes, re-initialize the initial Display lighting timer value
  }

  // Antenna selection when a change of frequency on the transceiver occurs
  if (new_freq_avail)                                                   // New frequency flag raised?
  {
    for (i = 0; i < num_channels; i++)                                  // Loop as many times as there are antenna ports. This scans through all antennas
    {
      if ((frequency >= Antenna_Data[i][2].toInt()) && (frequency <= Antenna_Data[i][3].toInt()))    // Is frequency within range of selected antenna?
      {                                                                 // Yes
        antenna_Tx = i;                                                 // Assign switch to the new antenna once found
        antenna_Rx = i;                                                 //      "
        new_freq_avail = false;                                         // Cancel the new frequency flag
      }
    }
  }

  // Serial user interface command management
  if (Serial.available())                                                         // Data available in USB serial port buffer?
  {
    charact = Serial.read();                                                      // Yes, read one character   
    if (charact == '\n')                                                          // is command line completely received (line feed character)?
    {
      // A: Antenna configuration command                         
      if (toupper(rx_data.charAt(0)) == 'A')                                      // Is command A or a?
      {
        Serial.println("\n=========Received antenna data============="); 
        int ind1 = rx_data.indexOf(',');                                          // Find location of first comma after command
        int ind2 = rx_data.indexOf(',', ind1 + 1 );                               // Find location of second comma
        String antenna_number = rx_data.substring(ind1 + 1, ind2);                // Capture second data string
        Serial.print("Antenna Number: ");                                         // Send the Antenna number to serial port
        Serial.println(antenna_number);                                           //      "   
        int ind3 = rx_data.indexOf(',', ind2 + 1 );                               // Find location of third comma
        String antenna_descr = rx_data.substring(ind2 + 1, ind3);                 // Capture third data string
        for(j = antenna_descr.length(); j < 16; j++) antenna_descr += " ";        // Add spaces if needed to make a 16-character string  
        Serial.print("Antenna Description: ");                                    // Send the Antenna description to serial port
        Serial.println(antenna_descr);                                            //     "  
        int ind4 = rx_data.indexOf(',', ind3 + 1 );                               // Find location of fourth comma                               
        String antenna_type_txrx = (rx_data.substring(ind3 + 1, ind4));           // Capture forth data string
        antenna_type_txrx.toUpperCase();                                          // Convert to uppercase
        for(j = antenna_type_txrx.length(); j < 5; j++) antenna_type_txrx += " "; // Add spaces if needed to make a 5-character string  
        Serial.print("Antenna Type: ");                                           // Send the Antenna type to serial port
        Serial.println(antenna_type_txrx);                                        //     "  
        int ind5 = rx_data.indexOf(',', ind4 + 1 );                               // Find location of fifth comma          
        String antenna_freq_min = rx_data.substring(ind4 + 1, ind5);              // Capture fifth data string  
        for(j = antenna_freq_min.length(); j < 2; j++) antenna_freq_min += " ";   // Add a space if needed to make a 2-character string  
        Serial.print("Antenna minimum frequency: ");                              // Send the Antenna minimum frequency to serial port
        Serial.println(antenna_freq_min);                                         //     " 
        String antenna_freq_max = rx_data.substring(ind5 + 1);                    // Capture sixth and last data string
        for(j = antenna_freq_max.length(); j < 2; j++) antenna_freq_max += " ";   // Add a space if needed to make a 2-character string     
        Serial.print("Antenna maximum frequency: ");                              // Send the Antenna maximum frequency to serial port
        Serial.println(antenna_freq_max);                                         //     " 
        rx_data = "";                                                             // Erase received data string

        validation_error = true;                                                  // Preset the validation error flag
        if      ((antenna_number.toInt() < 1) || (antenna_number.toInt() > 8)) 
                  Serial.println("\n**Error**: Antenna number must be between 2 and 8.\n");      
        else if (antenna_number.toInt() > num_channels) 
                  Serial.println("\n**Error**: Antenna number is higher than number of outputs defined.\n");      
        else if (antenna_descr.length() > 16) 
                  Serial.println("\n**Error**: Antenna description must have a maximum of 16 characters.\n");      
        else if ((antenna_type_txrx != "TX-RX") && (antenna_type_txrx != "RX   "))  
                  Serial.println("\n**Error**: Antenna type must be either TX-RX or RX.\n");
        else if ((antenna_freq_min.toInt() == 0) && (antenna_freq_min != "0 ")) 
                  Serial.println("\n**Error**: Antenna minimum frequency must be a number in MHz.\n");      
        else if ((antenna_freq_max.toInt() == 0) && (antenna_freq_max != "0 ")) 
                  Serial.println("\n**Error**: Antenna maximum frequency must be a number in MHz.\n");      
        else if (antenna_freq_min.toInt() > antenna_freq_max.toInt()) 
                  Serial.println("\n**Error**: Maximum frequency is lower than minimum frequency\n");         
        else
        {
          validation_error = false;                                                           // At this point, no validation error, so clear the validation error flag                        
          // Verify that there is no frequency orange overlap with existing antennas
          for (j = 0; j < num_channels; j++)                                                  // Loop as many times as there are antenna ports. This scans through all antennas
          {
            if (((antenna_number.toInt() - 1) != j)                                           // Selected antennna frequency overlap range against other antenna ranges? 
              && (!((Antenna_Data[j][2].toInt() == 0) && (Antenna_Data[j][3].toInt() == 0)))  // Exclude check of antennas with 0 and 0 as defined range 
              && (!((antenna_freq_min.toInt() == 0) && (antenna_freq_max.toInt() == 0)))      // Exclude check if defined antenna has 0 and 0 as defined range
              && (((Antenna_Data[j][2].toInt() <= antenna_freq_min.toInt())                   //     " 
                   && (antenna_freq_min.toInt() <= Antenna_Data[j][3].toInt()))               //     " 
                || ((Antenna_Data[j][2].toInt() <= antenna_freq_max.toInt())                  //     "  
                   && (antenna_freq_max.toInt() <= Antenna_Data[j][3].toInt()))))             //     "  
            {  
              Serial.print("\n**Error**: Antenna frequency range conflicts with Antenna #");  // Yes, send error message to serial port
              Serial.print(j + 1);                                                            //     " 
              Serial.println(" frequency range.\n");                                          //     " 
              freq_range_error = true;                                                        //     " 
              break;                                                                          //  Leave loop before the end
            }
          }
        }
        if ((!freq_range_error) && (!validation_error))                                       // Antenna data contains no errors and frequency overlap?
        {                                                                                     // Yes
          Serial.println("=========Antenna data accepted=============");                      // Send acceptance message to serial port
          Antenna_Data[antenna_number.toInt()- 1][0] = antenna_descr;                         // Transfer the temporary data variable values to the antenna data array
          Antenna_Data[antenna_number.toInt()- 1][1] = antenna_type_txrx;                     //     " 
          if ((antenna_freq_min[0] == '0') && (antenna_freq_min[1] != ' '))                   // Does minimum frequency string has the "0X" format (zero followed by a digit)?
          {                                                                                   // Yes
            antenna_freq_min.remove(0,1);                                                     // Remove the zero
            antenna_freq_min += " ";                                                          // Add a space after the digit
          }
          Antenna_Data[antenna_number.toInt() - 1][2] = antenna_freq_min;                     // Transfer the temporary data variable value to the antenna data array
          if ((antenna_freq_max[0] == '0') && (antenna_freq_max[1] != ' '))                   // Does maximum frequency string has the "0X" format (zero followed by a digit)?
          {                                                                                   // Yes
            antenna_freq_max.remove(0,1);                                                     // Remove the zero
            antenna_freq_max += " ";                                                          // Add a space after the digit
          }
          Antenna_Data[antenna_number.toInt() - 1][3] = antenna_freq_max;                     // Transfer the temporary data variable value to the antenna data array
          draw_Ant_Banners();                                                                 // Re-draw the antenna banners
          // Save the antenna data to EEPROM
          for (i = 0; i < num_channels; i++)                                                  // Loop as many times as there are antenna ports. This scans through all antennas          
          {
            for (j = 0; j < 16; j++)  EEPROM.write(Ant_Data_Base_Addr + (i * 25) + j, Antenna_Data[i][0].charAt(j));        // Save the Antenna description to EEPROM
            for (j = 16; j < 21; j++) EEPROM.write(Ant_Data_Base_Addr + (i * 25) + j, Antenna_Data[i][1].charAt(j - 16));   // Save the Antenna type to EEPROM
            for (j = 21; j < 23; j++) EEPROM.write(Ant_Data_Base_Addr + (i * 25) + j, Antenna_Data[i][2].charAt(j - 21));   // Save the Antenna minimum frequency to EEPROM
            for (j = 23; j < 25; j++) EEPROM.write(Ant_Data_Base_Addr + (i * 25) + j, Antenna_Data[i][3].charAt(j - 23));   // Save the Antenna maximum frequency to EEPROM
          }
          EEPROM.commit();                                                                    // Commit to EEPROM
        }
        freq_range_error = false;                                                             // Clear the frequency overlap error flag
      }
      // D: Tx Delay value in milliseconds 
      else if (toupper(rx_data.charAt(0)) == 'D')                                             // Is command D or d?
      {                                                                                       // Yes
        int ind1 = rx_data.indexOf(',');                                                      // Find location of first comma after command
        String tx_del = rx_data.substring(ind1 + 1);                                          // Captures second data string        
        Serial.print("Transmit Delay: ");                                                     // Send transmit delay value to serial port
        Serial.println(tx_del);                                                               //      "
        if ((tx_del.toInt() < 10) || (tx_del.toInt() > 100))                                   // If transmit delay is not between 0 and 100, send error message 
             Serial.println("\n**Error**: The Transmit delay value must be between 10 and 100.\n"); //      "      
        else                                                                                  // Transmit delay is not between 0 and 100
        {                               
          tx_delay = tx_del.toInt() - 5;                                                      // Save Transmit delay value to the right variable                               
          EEPROM.write(Tx_Delay_Base_Addr, tx_delay);                                           // Save Transmit delay value to EEPROM
          EEPROM.commit();                                                                    // Commit into EEPROM
       }       
      }
      // L: Configuration display command
      else if (toupper(rx_data.charAt(0)) == 'L')                                             // Is command L or l?
      {
        Serial.println("\n===============Configuration================");                     // Print the data listing header
        Serial.println("Ant # |Description      |Mode  |Fmin |Fmax");                         //       " 
        Serial.println("------|-----------------|------|-----|-----");                        //       " 
        for (i = 0; i < num_channels; i++)                                                    // Loop as many times as there are antenna ports. This scans through all antennas
        {
          Serial.print(String(i + 1));                                                        // Send antenna number to serial port
          Serial.print("      ");                                                             // Add spaces for alignment purpose
          Serial.print(Antenna_Data[i][0]);                                                   // Send antenna description to serial port
          Serial.print("  ");                                                                 // Add spaces for alignment purpose
          Serial.print(Antenna_Data[i][1]);                                                   // Send antenna description to serial port
          Serial.print("  ");                                                                 // Add spaces for alignment purpose
          Serial.print(Antenna_Data[i][2]);                                                   // Send antenna minimum frequency to serial port
          Serial.print("    ");                                                               // Add spaces for alignment purpose
          Serial.print(Antenna_Data[i][3]);                                                   // Send antenna maximum frequency to serial port
          Serial.println();                                                                   // Send an empty line to serial port
        }
        Serial.println("--------------------------------------------");                       // Send a dividing line to serial port
        Serial.print("Output Mode: ");                                                        // Send output mode value to serial port
        Serial.println(output_mode);                                                          //       " 
        Serial.print("Number of Ports: ");                                                    // Send number of channels value to serial port
        Serial.println(num_channels,DEC);                                                         //       " 
        Serial.print("Protection to Port 1: ");                                               // Send protection-to-port-1 value to serial port
        Serial.println(protect_to_ch1);                                                       //       " 
        Serial.print("Transmit Delay: ");                                                     // Send protection-to-port-1 value to serial port
        Serial.println(tx_delay + 5,DEC);                                                       //       " 
        Serial.println("============================================");                       // Send a dividing line to serial port
      }
      // O: Output mode select command
      else if (toupper(rx_data.charAt(0)) == 'O')                                             // Is command O or o?
      {                                                                                       // Yes
        int ind1 = rx_data.indexOf(',');                                                      // Find location of first comma after command
        String o_mode = rx_data.substring(ind1 + 1);                                          // Captures second data string        
        o_mode.toUpperCase();                                                                 // Convert to uppercase
        Serial.print("Output mode: ");                                                        // Send output mode value to serial port
        Serial.println(o_mode);                                                               //      "
        if ((o_mode != "BCD") && (o_mode != "BIT"))                                           // If mode value not "BIT" nor "BCD", send error message
              Serial.println("\n**Error**: The output mode must be either BCD or BIT.\n");    //      "
        else                                                                                  // Mode value is "BIT" or "BCD"
        {
          output_mode = o_mode;                                                               // Save received mode value to the right variable
          config_Outputs();                                                                   // Re-configure output pins accordingly
          for (j = 0; j < 3; j++) EEPROM.write(Output_Mode_Base_Addr + j, o_mode.charAt(j));  // Save mode value to EEPROM
          EEPROM.commit();                                                                    // Commit into EEPROM
        }       
      }
      // N: Number of antenna ports configuration command
      else if (toupper(rx_data.charAt(0)) == 'N')                                             // Is command N or n?
      {                                                                                       // Yes
        int ind1 = rx_data.indexOf(',');                                                      // Find location of first comma after command
        String n_chan = rx_data.substring(ind1 + 1);                                          // Captures second data string        
        Serial.print("Number of channels: ");                                                 // Send number of channels value to serial port
        Serial.println(n_chan);                                                               //      "
        if ((n_chan.toInt() < 2) || (n_chan.toInt() > 8))                                     // If number of channels is not between 2 and 8, send error message 
              Serial.println("\n**Error**: The number of channels must be between 2 and 8.\n"); //      "      
        else                                                                                  // Number of channels is between 2 and 8
        {                               
          num_channels = n_chan.toInt();                                                      // Save Number of channels value to the right variable                               
          config_Outputs();                                                                   // Re-configure output pins accordingly
          EEPROM.write(Num_Channels_Base_Addr, n_chan.charAt(0));                             // Save Number of channels value to EEPROM
          EEPROM.commit();                                                                    // Commit into EEPROM
          draw_Ant_Banners();                                                                 // Re-draw antenna banners to display
        }       
      }
      // ?/H: Help display command                                                            
      else if ((rx_data.charAt(0) == '?') || (toupper(rx_data.charAt(0)) == 'H'))                       // Is command ? or H or h?
      {                                                                                                 // Yes
        Serial.println("\n===========Help: List of Commands============");                              // Send help text to serial port
        Serial.println("A: Configures one antenna");                                                    //      "
        Serial.println("   - Syntax: A,<1..8>,<a..z/A..Z/0..9>,<TX-RX/RX>,<0..99>,<0..99>");            //      "
        Serial.println("              | Ant# |  Description   | Ant Type | Fmin  | Fmax");              //      "
        Serial.println("   - Antenna Type: Rx only = RX, both Tx and Rx = TX-RX");
        Serial.println("   - Description: Alpha-numeric, including symbols and lowercase,"); 
        Serial.println("     maximum 16 characters");
        Serial.println("   - To disable antenna selection vs. frequency: use Fmin = 0 and Fmax = 0\n");
        Serial.println("D: Sets the delay on the Transmit Delay output, from 10 to 100");
        Serial.println("   - Syntax: D,<10..100>\n");
        Serial.println("H: Shows this help information");
        Serial.println("   - Syntax: H\n");
        Serial.println("L: Lists antenna configuration and parameter settings");
        Serial.println("   - Syntax: L\n");
        Serial.println("N: Sets the number of antenna ports on the remote switch, from 2 to 8");
        Serial.println("   - Syntax: N,<2..8>\n");
        Serial.println("O: Sets the output type to BCD lines or to individual relay (bit) lines");
        Serial.println("   - Syntax: O,<BCD/BIT>\n");
        Serial.println("P: Controls the protection switch to Antenna 1 when transmitting"); 
        Serial.println("   into a receive-only antenna");
        Serial.println("   - Syntax: P,<Y/N>\n");
        Serial.println("=============================================");               
      }
      // P: Protection to antenna port 1 configuration command                              
      else if (toupper(rx_data.charAt(0)) == 'P')                                             // Is command P or p?
      {                                                                                       // Yes
        int ind1 = rx_data.indexOf(',');                                                      // Find location of first comma after command
        String protect = rx_data.substring(ind1 + 1);                                         // Captures second data string        
        protect.toUpperCase();                                                                // Convet to uppercase
        Serial.print("Protection to channel 1: ");                                            // Send protect-to-channel-1 value to serial port
        Serial.println(protect);                                                              //      "
        if ((protect != "Y") && (protect != "N"))                                             // If protect-to-channel-1 value not "Y" nor "N", send error message
              Serial.println("\n**Error**: The protection command takes either Y or N.\n");   //      "
        else                                                                                  // Protect-to-channel-1 value is either "Y" or "N"
        {
          protect_to_ch1 = protect;                                                           // Save Protect-to-channel-1 value to the right variable
          EEPROM.write(Protection_Xfer_Base_Addr, protect.charAt(0));                         // Save Protect-to-channel-1 value to EEPROM
          EEPROM.commit();                                                                    // Commit into EEPROM
        }       
      }
      // Command not recognized
      else Serial.println("\n**Error**: The command received does not exist.\n");             // Command not recognized, send an error message                    
      rx_data = "";                                                                           // Clear received command string  
    }
    // Command not completely received
    else                                                                                      // Command string not completely received
    {
      rx_data += charact;                                                                     // Add received character to the command string
      if (rx_data.length() > 50)                                                              // Is comand string of 50 characters or more?
      {                                                                                       // Yes
        rx_data = "";                                                                         // Clear received command string 
      }
    }
  }
}

// Main loop function running on the Pico's second core, Arduino style.
// This is the place where other CAT protocols (Yaesu, Kenwood, etc) could be added in the future.
void loop1()
{
  // CI-V received message management
  if (Serial1.available())                                                                    // Data byte(s) available in the CI-V port receive buffer?
  {                                                                                           // Yes
    civ_rx_data[civ_byte_pos] = Serial1.read();                                               // Read one data byte
    if (civ_rx_data[civ_byte_pos] == 0xFD)                                                    // End of message byte received?
    {                                                                                         // Yes
      frequency = ((civ_rx_data[8] & 0x0F) + (((civ_rx_data[8] >> 4) & 0x0F) * 10));          // Extract the MHz frequency bytes
      if (frequency != old_frequency)                                                         // New frequency (different MHz from previous frequency)?
      {                                                                                       // Yes
        old_frequency = frequency;                                                            // Save the new frequency as previous frequency for future checks
        new_freq_avail = true;                                                                // Flag that a new frequency is available
      }
      civ_byte_pos = 0;                                                                       // Re-initialize the CI-V message byte pointer value
    }
    else civ_byte_pos++;                                                                      // End of message byte not received, increment the CI-V message byte pointer value
  }
}
// End of source code
