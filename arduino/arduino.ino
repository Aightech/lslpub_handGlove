/* Glove reader - MCP3208
 *  
 * Author: Alexis Devillard
 * Date: 10/06/2019 
 * 
 * Inspired by : https://playground.arduino.cc/Code/MCP3208/
 * 
 */

 /*
  PINOUT:  MCP3208 => Arduino
  -------   ___ 
  CH_7 | 1 | u | 16 | Vdd  - 5V 
  CH_6 | 2 |   | 15 | Vref - 5V
  CH_5 | 3 |   | 14 | Agnd - GND
  CH_4 | 4 |   | 13 | Dclk - 13 (CLCK)
  CH_3 | 5 |   | 12 | Dout_PIN - 12 (MISO)
  CH_2 | 6 |   | 11 | Din  - 11 (MOSI)
  CH_1 | 7 |   | 10 | /CS  - 8-10
  CH_0 | 8 |___| 9  | Dgnd - GND

  */



#define CS0_PIN 8  //Select Pin 
#define CS1_PIN 9  //Select Pin 
#define CS2_PIN 10 //Select Pin 

#define Dout_PIN 11 //MOSI 
#define Din_PIN  12 //MISO 
#define Dclk_PIN 13 //Clock 

int readvalue[23]; 

int read_adc(int channel, int CSpin)
{
    int adcvalue = 0;
    // Command bits:
    // -----------   | start |  mode |    Channel to read    |       dont care       |
    //mode:-single:1 |-------|-------|-------|-------|-------|-------|-------|-------|
    //     -diff  :0 |   1   |   1   |  CHN  |  CHN  |  CHN  |   -   |   -   |   -   |
    byte commandbits = B11000000; 
    //allow channel selection
    commandbits|=(channel<<3);
    
    digitalWrite(CSpin,LOW); //Select adc
    
    // setup bits to be written
    for (int i=7; i>=3; i--)
    {
      digitalWrite(Dout_PIN,commandbits&1<<i);
      //cycle clock
      digitalWrite(Dclk_PIN,HIGH);
      digitalWrite(Dclk_PIN,LOW);    
    }
    
    //ignores 2 null bits
    digitalWrite(Dclk_PIN,HIGH);    
    digitalWrite(Dclk_PIN,LOW);
    digitalWrite(Dclk_PIN,HIGH);  
    digitalWrite(Dclk_PIN,LOW);
    
    //read bits from adc
    for (int i=11; i>=0; i--)
    {
      adcvalue += digitalRead(Din_PIN)<<i;
      //cycle clock
      digitalWrite(Dclk_PIN,HIGH);
      digitalWrite(Dclk_PIN,LOW);
    }
    digitalWrite(CSpin, HIGH); //turn off device
    return adcvalue;
}

void setup()
{ 
    //set pin modes 
    pinMode(CS0_PIN, OUTPUT); 
    pinMode(CS1_PIN, OUTPUT); 
    pinMode(CS2_PIN, OUTPUT);  
    pinMode(Dout_PIN, OUTPUT); 
    pinMode(Din_PIN , INPUT); 
    pinMode(Dclk_PIN, OUTPUT); 
    
    //disable device to start with 
    digitalWrite(CS0_PIN,HIGH);
    digitalWrite(CS1_PIN,HIGH);
    digitalWrite(CS2_PIN,HIGH); 
    digitalWrite(Dout_PIN,LOW); 
    digitalWrite(Dclk_PIN,LOW); 
    
    Serial.begin(9600); 
} 


void loop() 
{ 
  char c = -1;
  if(Serial.available() > 0)
  {
    c = Serial.read();
  }
  switch(c)
  {
    case -1:
    //nothing
    break; 
    
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    {
      int i = c - '0';
      readvalue[0] = read_adc(i%8, CS0_PIN + i/8); 
      Serial.write((char*)readvalue,2);
    }
    break;
    case 'a':
    {
      for(int i = 0 ; i<10 ; i++)
        readvalue[i] = read_adc(i%8, CS0_PIN + i/8); 
      Serial.write((char*)readvalue, 2*10); 
    }
    break;
    case 'b':
    {
      for(int i = 0 ; i<24 ; i++)
        readvalue[i] = read_adc(i%8, CS0_PIN + i/8); 
      Serial.write((char*)readvalue, 2*23); 
      
    }
    break;
    
    default:
    //nothing
    break;
    
   
  }
  delay(10); 
} 
