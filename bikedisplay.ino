#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

int sensorPin0 = A0;    // select the input pin for the potentiometer
float sensorValue0 = 0;  // variable to store the value coming from the sensor
float sensorOutput0 = 0;

int sensorPin1 = A1;    // select the input pin for the potentiometer
float sensorValue1 = 0;  // variable to store the value coming from the sensor
float sensorOutput1 = 0;

//variables to keep track of the timing of recent interrupts
unsigned long onebutton_time = 0;  
unsigned long onelast_button_time = 0;
int onecounter = 0;

unsigned long twobutton_time = 0;  
unsigned long twolast_button_time = 0;
int twocounter = 0;

#define BME280_ADDRESS 0x76
unsigned long int hum_raw,temp_raw,pres_raw;
signed long int t_fine;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;

void setup() {
    uint8_t osrs_t = 1;             //Temperature oversampling x 1
    uint8_t osrs_p = 1;             //Pressure oversampling x 1
    uint8_t osrs_h = 1;             //Humidity oversampling x 1
    uint8_t mode = 3;               //Normal mode
    uint8_t t_sb = 5;               //Tstandby 1000ms
    uint8_t filter = 0;             //Filter off 
    uint8_t spi3w_en = 0;           //3-wire SPI Disable
    
    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;
    
    Serial.begin(9600);
    Wire.begin();
    
    writeReg(0xF2,ctrl_hum_reg);
    writeReg(0xF4,ctrl_meas_reg);
    writeReg(0xF5,config_reg);
    readTrim();

    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);  
    attachInterrupt(0, onebutton, CHANGE);
    attachInterrupt(1, twobutton, CHANGE);

    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
    // init done
}

void loop() {  
//    signed long int temp_cal;
//    unsigned long int press_cal,hum_cal;
//    int temp_act = 0.0, press_act = 0.0, hum_act=0.0;
//    
//    readData();
//    
//    temp_cal = calibration_T(temp_raw);
//    press_cal = calibration_P(pres_raw);
//    hum_cal = calibration_H(hum_raw);
//    temp_act = (double)temp_cal / 100.0;
//    press_act = (double)press_cal / 100.0;
//    hum_act = (double)hum_cal / 1024.0;
  
    voltage_aprilia();
    delay(5000);
    voltage_arduino();
    delay(2000);
    outside_temperature();
    delay(5000);
    outside_humidity();
    delay(2000);
}

void voltage_aprilia() {
  for(int i=0; i < 10; i++)
  {
    sensorValue0 = analogRead(sensorPin0);
    sensorOutput0 = sensorValue0/40;
    display.clearDisplay();  
    display.setCursor(20,0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println("APRILIA VOLTAGE");
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(19,10);
    display.print(sensorOutput0 - 0.2, 1);
    display.print("V");
    display.display();
    delay(500);
  }
}

void voltage_arduino() {
  for(int i=0; i < 4; i++)
  {
    sensorValue1 = analogRead(sensorPin1);
    sensorOutput1 = sensorValue1/40;
    display.clearDisplay();  
    display.setCursor(20,0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println("ARDUINO VOLTAGE");
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(32,10);
    display.print(sensorOutput1, 1);
    display.print("V");
    display.display();
    delay(500);
  }
}

void outside_temperature() {
  for(int i=0; i < 5; i++)
  {
    signed long int temp_cal;
    int temp_act = 0.0; 
    readData();
    temp_cal = calibration_T(temp_raw);
    temp_act = (double)temp_cal / 100.0;
    
    display.clearDisplay();  
    display.setCursor(9,0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println("OUTSIDE TEMPERATURE");
    display.setTextSize(3);
    display.setTextColor(WHITE);
    if (temp_act < 10)
    {
      display.setCursor(49,10);
    }
    else
    {
      display.setCursor(42,10);
    }
    display.print(temp_act);
    display.print("C");
    display.display();
    delay(1000);
  }
}

void outside_humidity() {
  for(int i=0; i < 2; i++)
  {
    unsigned long int hum_cal;
    int hum_act=0.0;
    readData();
    hum_cal = calibration_H(hum_raw);
    hum_act = (double)hum_cal / 1024.0;
    
    display.clearDisplay();
    display.setCursor(19,0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println("OUTSIDE HUMIDITY");
    display.setTextSize(3);
    display.setTextColor(WHITE);
    if (hum_act > 99)
    {
      display.setCursor(30,10);
    }
    else
    {
      display.setCursor(44,10);
    } 
    display.print(hum_act);
    display.print("%");
    display.display();
    delay(1000);
  }
}

void onebutton() {
    onebutton_time = millis();
    //check to see if increment() was called in the last 250 milliseconds
    if (onebutton_time - onelast_button_time > 250)
    {
        if (onecounter < 1) {onecounter++;}
        else {onecounter = 0;}
        onelast_button_time = onebutton_time;
    }
}

void twobutton() {
    twobutton_time = millis();
    //check to see if increment() was called in the last 250 milliseconds
    if (twobutton_time - twolast_button_time > 250)
    {
        if (twocounter < 1) {twocounter++;}
        else {twocounter = 0;}
        twolast_button_time = twobutton_time;
    }
}

void readTrim() {
    uint8_t data[32],i=0;
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,24);
    
    while(Wire.available()) {
        data[i] = Wire.read();
        i++;
    }
    
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xA1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,1);
    data[i] = Wire.read();
    i++;
    
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xE1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,7);

    while(Wire.available()) {
        data[i] = Wire.read();
        i++;    
    }

    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];   
}

void writeReg(uint8_t reg_address, uint8_t data) {
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(reg_address);
    Wire.write(data);
    Wire.endTransmission();    
}

void readData() {
    int i = 0;
    uint32_t data[8];
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,8);
    while(Wire.available())
    {
        data[i] = Wire.read();
        i++;
    }
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    hum_raw  = (data[6] << 8) | data[7];
}

signed long int calibration_T(signed long int adc_T) {
    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}

unsigned long int calibration_P(signed long int adc_P) {
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }    
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);   
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;    
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

unsigned long int calibration_H(signed long int adc_H) {
    signed long int v_x1;
    
    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) + 
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) * 
              (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) * 
              ((signed long int) dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);   
}
