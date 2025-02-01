//------------------SU POMPASI KISMI-------------------
int waterPump = 12;
int waterPumpState = LOW; 
unsigned long previousMillis_waterPump = 0;
const long waterPumpStart = 120000; // for x sec 1200000
const long waterPumpStop = 1200000; // on pump for x seconds  120000



//---------------------LED KISMI--------------------------
int relay1 = 8;                  // the number of the relay pin
int relay_state = LOW;             // relay_state used to set the LED
unsigned long previousMillis_LED = 0;        // will store last time LED was updated
long OnTime = 57600000 ;           // milliseconds of on-time
long OffTime = 28800000;          // milliseconds of off-time


//------------------CO2 KISMI------------------
#include <SoftwareSerial.h>
unsigned long ilk_zaman = 0;
int CO2;

#include <SoftwareSerial.h>
SoftwareSerial mySerial(13, 12); // RX, TX
unsigned char hexdata[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; //Read the gas density command /Don't change the order

//------------------ISINEM KISMI------------------
#include <Adafruit_AM2315.h>
#include <Wire.h> 
Adafruit_AM2315 am2315;

int r_temp_result=true;                
int r_hum_result=true;                // default olarak olan değer.   eğer false ise  true olanlar false, false olanlar true olarak mustafa urgan değiştirecek

unsigned long temp_hum_s_in_t=0;    
unsigned long son_zaman;             // sistemin ortak son zamanı    cengiz değiştirecek sisteme göre 

int t_h_s_val=0;        
float avg_t=0;         
float avg_h=0;         
int sum_t;
int sum_h;
float temperature,humidity;
/******************/

//------------------FAN KISMI------------------

const int fan_control_pin = 11 ;
int fanState=LOW;
int FanOpenTime=2000;
int FanOffTime=5000;
int FanOpenTimeHum=5000;
int FanOffTimeHum=3000;
int FanOnTimeCO2=5000;
int FanOffTimeCO2=2000;

//-----------------EC SENSÖR KISMI-----------------
//millis kullanımı
unsigned long EC_previousMillis = 0;
//unsigned long currentMillis = millis();
//int interval = 6000;
int EC_state = 0;
// ortalama alma
float ecsum =0;
float ecavg ;

int avg_Val_EC =0;
///
int EC_R1= 1000; // Value of resistor for EC probe
int EC_Read = A0;// arduino ile değiştirilecek
int ECPower = A1;// arduino ile değiştirilecek
int EC_Temp_pin = A5;// arduino ile değiştirilecek
float EC_Temp_C; // Do not change
float EC_Temp_F; // Do not change
float EC_Temp1_Value = 0;
float EC_Temp_Coef = 0.019; // You can leave as it is
/////////////////This part needs your attention during calibration only///////////////
float EC_Calibration_PPM =1080 ; //Change to PPM reading measured with a separate meter
float EC_K=0.06; //You must change this constant once for your probe(see video)
float EC_PPM_Con=0.5; //You must change this only if your meter uses a different factor
/////////////////////////////////////////////////////////////////////////////////////
float CalibrationEC= (EC_Calibration_PPM*2)/1000;
float EC_Temperature;
float EC;
float EC_at_25;
int EC_ppm;
float EC_A_to_D= 0;
float EC_Vin= 5;
float EC_Vdrop= 0;
float EC_R_Water;
float EC_Value=0;




//-----------------*C KISMI-----------------
// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2 //pin for sensor

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
const unsigned long dallasTemp_interval = 5000;
unsigned long dallasTemp_previousTime = 0;
 
/*
   The setup function. We only start the sensors here
*/
//-----------------PH KISMI-----------------
#define SENSOR A3            //pH meter Analog output to Arduino Analog Input 0
#define OFFSET 0.00            //deviation compensate
#define LED 10
#define SAMPLING_INTERVAL 20
#define PRINT_INTERVAL 800
#define ARRAY_LENGTH  40  //times of collection
int PH;
int PH_ARRAY[ARRAY_LENGTH];   //Store the average value of the sensor feedback
int PH_ARRAY_INDEX=0;


//-----------------POMPA KISMI-----------------
class pump1and2{
  int enA;
  int enB;
  int in1_1;
  int in1_2;
  int in1_3;
  int in1_4;

  long in1_1S;
  long in1_2S;
  long in1_3S;
  long in1_4S;

  int in1_1on;
  int in1_2on;
  int in1_3on;
  int in1_4on;

  int in1_1off;
  int in1_2off;
  int in1_3off;
  int in1_4off;

  unsigned long preMillis1;
  unsigned long preMillis;
  unsigned long preMillis2;

  public:
  pump1and2(int in1, int in2, int enAA, int in3, int in4, int enBB, long in1on, long in1off, long in2on, long in2off, long in3on, long in3off, long in4on, long in4off){
    
    in1_1=in1; 
    in1_2=in2;
    in1_3=in3; 
    in1_4=in4;
    enA=enAA;
    enB=enBB;

    pinMode(in1_1,OUTPUT);
    pinMode(in1_2,OUTPUT);
    pinMode(in1_3,OUTPUT);
    pinMode(in1_4,OUTPUT);
    pinMode(enA,OUTPUT);
    pinMode(enB,OUTPUT);
    
    in1_1on=in1on;
    in1_2on=in2on;
    in1_3on=in3on;
    in1_4on=in4on;

    in1_1off=in1off;
    in1_2off=in2off;
    in1_3off=in3off;
    in1_4off=in4off;

    in1_1S=LOW;
    in1_2S=LOW;
    in1_3S=LOW;
    in1_4S=LOW;

    preMillis=0;
    preMillis1=0;
    preMillis2=0;
    
    }

  void Update(){
    unsigned long curMillis=millis();
    unsigned long cur2Millis=millis();
    unsigned long cur3Millis=millis();

    analogWrite(enA,255);
    analogWrite(enB,255);
    //----------------------ASİT POMPASI--------------------
    if((in1_1S==HIGH)&&((curMillis-preMillis)>in1_1on)){//&&(pOH<=8.2)&&(pOH>=7.8)){

    in1_1S=LOW;
    preMillis=curMillis;
    digitalWrite(in1_1,in1_1S);
    }

    else if((in1_1S==LOW)&&((curMillis-preMillis)>in1_1off)){//&&(pOH<7.8)){

    in1_1S=HIGH;
    preMillis=curMillis;
    digitalWrite(in1_1,in1_1S);
    }
  //----------------------BAZ POMPASI--------------------
    if((in1_3S==HIGH)&&((cur2Millis-preMillis1)>in1_3on)){//&&(pOH<=8.2)&&(pOH>=7.8)){

    in1_3S=LOW;
    preMillis1=cur2Millis;
    digitalWrite(in1_3,in1_3S);
    }

    else if((in1_3S==LOW)&&((cur2Millis-preMillis1)>in1_3off)){//&&(pOH>8.2)){

    in1_3S=HIGH;
    preMillis1=cur2Millis;
    digitalWrite(in1_3,in1_3S);
    }


    //----------------------EC POMPA KISMI-----------------------
if((in1_1S==HIGH)&&((cur3Millis-preMillis2)>in1_2on)){//&&(ecavg<=3000)&&ecavg>=2300){

    in1_1S=LOW;
    in1_3S=LOW;
    preMillis2=cur3Millis;
    digitalWrite(in1_1,in1_1S);
    digitalWrite(in1_3,in1_3S);
    }

    else if((in1_1S==LOW)&&(cur3Millis-preMillis2)>in1_2off){

    in1_1S=HIGH;
    in1_3S=HIGH;
    preMillis2=cur3Millis;
    digitalWrite(in1_1,in1_1S);
    digitalWrite(in1_3,in1_3S);
    }
  }
  
  };


//-----------------POMPA AYARLARI-----------------             12 11 13 10 9 8 6 5 7 4 3 2 
pump1and2 p1andp2(24,22,26,27,25,23,10000,2000,1000,2000,10000,2000,4,4);//13 12 11 10 9 8 7 6 5 4 3 2
pump1and2 p3andp4(31,29,33,32,30,28,2,2,6000,1000,2,2,3000,10000);
//  pump1and2(int in1, int in2, int enAA, int in3, int in4, int enBB, long in1on, long in1off, long in2on, long in2off, long in3on, long in3off, long in4on, long in4off){



void setup()
{

//------------------SU POMPASI KISMI-------------------
pinMode(waterPump, OUTPUT);

Serial.begin(9600);


//--------------------LED KISMI-----------------------
  // set the digital pin as output:
  pinMode(relay1, OUTPUT); 


/******************/
//------------------CO2 KISMI------------------
  Serial.begin(9600);
  while (!Serial) {

  }
  mySerial.begin(9600);


//------------------ISINEM KISMI------------------
Serial.begin(9600);
am2315.begin();  
/*****************/   

//------------------FAN KISMI------------------
pinMode(fan_control_pin, OUTPUT);
digitalWrite(fan_control_pin, fanState);  
  
//-----------------EC KISMI-----------------
Serial.begin(9600);
pinMode(EC_Read,INPUT);
pinMode(ECPower,OUTPUT);
//////////////////////////////////////////////////////////////////////////////////////////
//Calibrate (); // After calibration put two forward slashes before this line of code
//////////////////////////////////////////////////////////////////////////////////////////



  //-----------------*C KISMI-----------------
/// start serial port
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library");

  // Start up the library
  sensors.begin();
  
  //-----------------PH KISMI-----------------
pinMode(LED,OUTPUT);  
  Serial.begin(9600);  
  Serial.println("PH SENSOR KIT VOLTAGE TEST!");    //Test the serial monitor
  

}

unsigned long son_zaman_CO2;

void loop()
{

//------------------SU POMPASI KISMI----------------------

unsigned long currentMillis_waterPump = millis();
Serial.println("millis");
Serial.print(currentMillis_waterPump);

if ((waterPumpState == HIGH) && (currentMillis_waterPump - previousMillis_waterPump >= waterPumpStop)){
  
  waterPumpState = LOW;
  previousMillis_waterPump = currentMillis_waterPump;
  
  Serial.println("current");
  Serial.print(currentMillis_waterPump);
  Serial.print("previous");
  Serial.print(previousMillis_waterPump);
  digitalWrite(waterPump, waterPumpState);
}
else if ((waterPumpState == LOW) && (currentMillis_waterPump - previousMillis_waterPump >= waterPumpStart)){
  
  waterPumpState = HIGH;
  previousMillis_waterPump = currentMillis_waterPump;
  
  Serial.println("current");
  Serial.print(currentMillis_waterPump);
  Serial.print("previous");
  Serial.print(previousMillis_waterPump);
  digitalWrite(waterPump, waterPumpState);    
  }

//----------------------LED KISMI----------------------
// check to see if it's time to change the state of the LED
  unsigned long currentMillis_LED = millis();
     
  if((relay_state == HIGH) && (currentMillis_LED - previousMillis_LED >= OnTime))
  {
    relay_state = LOW;  // Turn it off
    previousMillis_LED = currentMillis_LED;  // Remember the time
    digitalWrite(relay1, relay_state);  // Update the actual LED 
  }
  else if ((relay_state == LOW) && (currentMillis_LED - previousMillis_LED >= OffTime))
  {
    relay_state = HIGH;  // turn it on
    previousMillis_LED = currentMillis_LED;   // Remember the time
    digitalWrite(relay1, relay_state);    // Update the actual LED
  }

//------------------CO2 KISMI------------------
son_zaman_CO2 = millis();
  if (son_zaman_CO2 - ilk_zaman >= 1000)
  {
    ilk_zaman = son_zaman_CO2;
    mySerial.write(hexdata, 9);


    for (int i = 0, j = 0; i < 9; i++)
    {
      if (mySerial.available() > 0)
      {
        long hi, lo, CO2;
        int ch = mySerial.read();

        if (i == 2) {
          hi = ch;    //High concentration
        }
        if (i == 3) {
          lo = ch;    //Low concentration
        }
        if (i == 8) {
          CO2 = hi * 256 + lo; //CO2 concentration
          Serial.print("CO2 concentration: ");
          Serial.print(CO2);
          Serial.println("ppm");
        }
        
        ilk_zaman = son_zaman_CO2;

      }

    }
  }





  
//------------------ISINEM KISMI------------------
     /******************/
  son_zaman=millis();                                            //son zaman değiştirilecek sistemin ortak zamanına göre
   
  if(son_zaman-temp_hum_s_in_t>=2000){                              

  am2315.readTemperatureAndHumidity(&temperature,&humidity);
  Serial.println("");
  Serial.print("Temp  *C= "); 
  Serial.println(temperature);
  Serial.print("Hum g/m3= "); 
  Serial.println(humidity);
  
  
  if(temperature<20 || temperature>24){     // mustafa Polat değerleri değiştirecek
    r_temp_result=false;    
  }
  else{
    r_temp_result=true;    
  }

  if(humidity<20 || humidity >22){      //Mustafa Polat   değerleri değiştirecek
    r_hum_result=false;
  }

  else{
  r_hum_result=true;
  }

  sum_t=sum_t+temperature;
  sum_h=sum_h+humidity;
  avg_t=sum_t/5;          // 5 kere ol. için
  avg_h=sum_h/5;          // 5 kere ol. için
  t_h_s_val=t_h_s_val+1;

  if(t_h_s_val==5){                                           // sistem 10 snde 1 kere ortalama yazıyor. sensör 2 snde 1 ölç. için 5 kere oluyor
    Serial.print("");
    Serial.print("Your avarage Room Temperature is :");
    Serial.println(avg_t);
    Serial.print("Your avarage Room Humidity is :");
    Serial.println(avg_h);
    t_h_s_val=0;
    avg_t=0;
    avg_h=0;
    sum_t=0;
    sum_h=0;
  }
  temp_hum_s_in_t=son_zaman;                    //son zaman değiştirilecek sistemin ortak zamanına göre
  }
                /*****************/
//------------------FAN KISMI------------------

unsigned long currentMillis=millis();
unsigned long currentMillis2=millis();
unsigned long currentMillis3=millis();

long previousMillis=0;

long previousMillis2=0;

long previousMillis3=0;

  if((fanState==LOW)&&(currentMillis-previousMillis>=FanOffTime)&&(r_temp_result==true)){
    
  fanState=HIGH;
  previousMillis=currentMillis;
  digitalWrite(fan_control_pin,fanState);
    
    }

  else if ((fanState==HIGH)&&(currentMillis-previousMillis>=FanOpenTime)&&(r_temp_result==false)){
    
  fanState=LOW;
  previousMillis=currentMillis;
  digitalWrite(fan_control_pin,fanState);
    
    }

if((fanState==LOW)&&(currentMillis2-previousMillis2>=FanOffTimeHum)&&(r_hum_result==true)){
  
  fanState=HIGH;
  previousMillis2=currentMillis2;
  digitalWrite(fan_control_pin,fanState);
  
  }

else if((fanState==HIGH)&&(currentMillis2-previousMillis2>=FanOpenTimeHum)&&(r_hum_result==false)){
  
  fanState=LOW;
  previousMillis2=currentMillis2;
  digitalWrite(fan_control_pin,fanState);
  
  }  



if((fanState==HIGH)&&(currentMillis3-previousMillis3>=FanOnTimeCO2)&&((CO2<=1300)||(CO2>=1400))){
  
  fanState=LOW;
  previousMillis3=currentMillis3;
  digitalWrite(fan_control_pin,fanState);
  
  }  

else if((fanState==LOW)&&(currentMillis3-previousMillis3>=FanOffTimeCO2)){
  
  fanState=HIGH;
  previousMillis3=currentMillis3;
  digitalWrite(fan_control_pin,fanState);
  
  }
  
  //-----------------EC KISMI-----------------
if (millis() - EC_previousMillis == 3000){


 EC_previousMillis = millis();
 //delay(6000); //Do not make this less than 6 sec (6000)
 if (EC_state == 0){
 GetEC(); //Calls GetEC()
 EC_state ++;
 
 }else{
 EC_state --;
 }
 }
 else{}



//-----------------*C KISMI-----------------
/*// call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  float tempC = sensors.getTempCByIndex(0);

  // Check if reading was successful
  if (tempC != DEVICE_DISCONNECTED_C)
  {
    Serial.print("Temperature for the device 1 (index 0) is: ");
    Serial.println(tempC);
    lcd.setCursor(0, 0);
    lcd.print("Temperature:");
    lcd.setCursor(0, 1);
    lcd.print(tempC);
    lcd.print((char)223);
    lcd.print("C");
    lcd.print("|");
    lcd.print(DallasTemperature::toFahrenheit(tempC));
    lcd.print(" F");
  }
  else
  {
    Serial.println("Error: Could not read temperature data");
  }*/

  
//-----------------PH KISMI-----------------
static unsigned long SAMPLING_TIME = millis();
  static unsigned long PRINT_TIME = millis();
  static float VOLTAGE;
  if(millis()-SAMPLING_TIME > SAMPLING_INTERVAL)
  {
      PH_ARRAY[PH_ARRAY_INDEX++]=analogRead(SENSOR);
      if(PH_ARRAY_INDEX==ARRAY_LENGTH)PH_ARRAY_INDEX=0;
      VOLTAGE = AVERAGE_ARRAY(PH_ARRAY, ARRAY_LENGTH)*5.0/1024;
      PH =7-VOLTAGE/57.14;
      SAMPLING_TIME=millis();
  }
  if(millis() - PRINT_TIME > PRINT_INTERVAL)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
  Serial.print("VOLTAGE OUTPUT: ");
        Serial.println(VOLTAGE,3);
        digitalWrite(LED,digitalRead(LED)^1);
        PRINT_TIME=millis();
        Serial.print("ph value: ");
        Serial.println(PH,1);
         digitalWrite(LED,digitalRead(LED)^1);
        PRINT_TIME=millis();
  }
  

 //-----------------POMPA KISMI-----------------
  p1andp2.Update();
  p3andp4.Update();

}
////////////////////////////////////////////////////////////////////////////////////

//-----------------PH KISMI-----------------
double AVERAGE_ARRAY(int* ARR, int NUMBER){
  int i;
  int max,min;
  double AVG;
  long AMOUNT=0;
  if(NUMBER<=0){
    Serial.println("ERROR!/n");
    return 0;
  }
  if(NUMBER<5){   //less than 5, calculated directly statistics
    for(i=0;i<NUMBER;i++){
      AMOUNT+=ARR[i];
    }
    AVG = AMOUNT/NUMBER;
    return AVG;
  }else{
    if(ARR[0]<ARR[1]){
      min = ARR[0];max=ARR[1];
    }
    else{
      min=ARR[1];max=ARR[0];
    }
    for(i=2;i<NUMBER;i++){
      if(ARR[i]<min){
        AMOUNT+=min;        //arr<min
        min=ARR[i];
      }else {
        if(ARR[i]>max){
          AMOUNT+=max;    //arr>max
          max=      AMOUNT+=ARR[i];
        }else{
          AMOUNT+=ARR[i]; //min<=arr<=max
        }
      }//if
    }//for
    AVG = (double)AMOUNT/(NUMBER-2);
  }//if
  return AVG;
}


//-----------------EC KISMI-----------------
void GetEC()
{
  {
//-----------------*C KISMI-----------------
// Set current time
   unsigned long dallasTemp_currentTime = millis();
if (dallasTemp_currentTime - dallasTemp_previousTime >= dallasTemp_interval){  
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  sensors.requestTemperatures(); // Send the command to get temperatures
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  float tempC = sensors.getTempCByIndex(0);

  // Check if reading was successful
  if (tempC != DEVICE_DISCONNECTED_C)
  {
    Serial.print("Temperature : ");
    Serial.print(tempC);
    Serial.print(" C\n");    

  }
  else
  {
    Serial.println("Error: Could not read temperature data");
  }
    dallasTemp_previousTime = dallasTemp_currentTime;
}  
}

int EC_val;
double EC_Temp;
EC_val=analogRead(EC_Temp_pin);
EC_Temp = log(((10240000/EC_val) - 10000));
EC_Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * EC_Temp * EC_Temp ))*
EC_Temp);
EC_Temp_C = EC_Temp - 273.15; // Kelvin to Celsius
EC_Temp_F = (EC_Temp_C * 9.0)/ 5.0 + 32.0; // Celsius to Fahrenheit
EC_Temp1_Value = EC_Temp_C;
EC_Temperature = EC_Temp_C;
digitalWrite(ECPower,HIGH);
EC_A_to_D= analogRead(EC_Read);
EC_A_to_D= analogRead(EC_Read);
digitalWrite(ECPower,LOW);
EC_Vdrop= (EC_Vin*EC_A_to_D) / 1024.0;
EC_R_Water = (EC_Vdrop*EC_R1) / (EC_Vin-EC_Vdrop);
EC = 1000/ (EC_R_Water*EC_K);
EC_at_25 = EC*1000 / (1+ EC_Temp_Coef*(EC_Temperature-25.0));
EC_ppm=(EC_at_25)*(EC_PPM_Con*1000);
//Serial.print(" EC: ");
//Serial.print(EC_at_25);
//Serial.print(" Siemens(S/cm) \n");

ecsum = ecsum + EC_at_25;
avg_Val_EC = avg_Val_EC+1;

 if (avg_Val_EC == 10){
 ecavg = ecsum/10;
 Serial.print("Average EC: ");
 Serial.print(ecavg);
 Serial.print(" Siemens(S/cm) \n");
 Serial.print("\n");
 avg_Val_EC = 0;
 ecavg = 0;
 ecsum = 0 ;
 }
 else{}

}
////////////////////////////////////////////////////////////////////////////////////
void Calibrate ()
{
{
Serial.println("Calibration routine started");
float EC_Temperature_end=0;
float EC_Temperature_begin=0;
int EC_val;
double EC_Temp;
EC_val=analogRead(EC_Temp_pin);
EC_Temp = log(((10240000/EC_val) - 10000));
EC_Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * EC_Temp * EC_Temp ))*
EC_Temp);
EC_Temp_C = EC_Temp - 273.15; // Kelvin to Celsius
EC_Temp_F = (EC_Temp_C * 9.0)/ 5.0 + 32.0; // Celsius to Fahrenheit
EC_Temp1_Value = EC_Temp_C;
EC_Temperature_begin=EC_Temp_C;
EC_Value = 0;
int EC_i=1;
while(EC_i<=10){
digitalWrite(ECPower,HIGH);
EC_A_to_D= analogRead(EC_Read);
EC_A_to_D= analogRead(EC_Read);
digitalWrite(ECPower,LOW);
EC_Value=EC_Value+EC_A_to_D;
EC_i++;
//delay(6000);
};
EC_A_to_D=(EC_Value/10);
EC_val=analogRead(EC_Temp_pin);
EC_Temp = log(((10240000/EC_val) - 10000));
EC_Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * EC_Temp * EC_Temp ))*
EC_Temp);
EC_Temp_C = EC_Temp - 273.15; // Kelvin to Celsius
EC_Temp_F = (EC_Temp_C * 9.0)/ 5.0 + 32.0; // Celsius to Fahrenheit
EC_Temp1_Value = EC_Temp_C;
EC_Temperature_end=EC_Temp_C;
EC =CalibrationEC*(1+(EC_Temp_Coef*(EC_Temperature_end-25.0)));
EC_Vdrop= (((EC_Vin)*(EC_A_to_D))/1024.0);
EC_R_Water=(EC_Vdrop*EC_R1)/(EC_Vin-EC_Vdrop);
float EC_K_cal= 1000/(EC_R_Water*EC);
Serial.print("Replace K in line 23 of code with K = ");
Serial.println(EC_K_cal);
Serial.print("Temperature difference start to end were = ");
EC_Temp_C=EC_Temperature_end-EC_Temperature_begin;
Serial.print(EC_Temp_C);
Serial.println("*C");
Serial.println("Temperature difference start to end must be smaller than 0.15*C");
Serial.println("");
}

Calibrate ();
} // ortalama değeri çalışır vaziyette
