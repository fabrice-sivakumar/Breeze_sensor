/* A demo sketch for the Modern Device Rev P Wind Sensor
* Requires a Wind Sensor Rev P from Modern Device
* http://moderndevice.com/product/wind-sensor-rev-p/
* 
* The Rev P requires at least at least an 8 volt supply. The easiest way to power it 
* if you are using an Arduino is to use a 9 volt or higher supply on the external power jack
* and power the sensor from Vin.
* 
* Hardware hookup 
* Sensor     Arduino Pin
* Ground     Ground
* +10-12V      Vin
* Out          A0
* TMP          A2
*
* Paul Badger 2014
* code in the public domain
*/

const int OutPin  = A0;   // wind sensor analog pin  hooked up to Wind P sensor "OUT" pin
const int TempPin = A2;   // temp sesnsor analog pin hooked up to Wind P sensor "TMP" pin



void setup() {
    Serial.begin(115200);
    pinMode(3,OUTPUT);
    //digitalWrite(3, HIGH);   // Should shut down the sensor
    delay(1000); 
}

void loop() {

    
    // read wind

    int windADunits = analogRead(OutPin);
     Serial.print("RW ");   // print raw A/D for debug
     Serial.print(windADunits);
     Serial.print("\t");
    
    
    // wind formula derived from a wind tunnel data, annemometer and some fancy Excel regressions
    // this scalin doesn't have any temperature correction in it yet
    float windMPH =  pow(abs(((float)windADunits - 264.0) / 85.6814), 3.36814);
    
    Serial.print(windMPH);
    Serial.print(" MPH\t");    
    float windms = windMPH * 0.44704 ;
    Serial.print(windms);
    Serial.print(" mps\t"); 

 


    // temp routine and print raw and temp C
    int tempRawAD = analogRead(TempPin);  
     Serial.print("RT ");    // print raw A/D for debug
     Serial.print(tempRawAD);
     Serial.print("\t");
    
    // convert to volts then use formula from datatsheet 
    // Vout = ( TempC * .0195 ) + .400
    // tempC = (Vout - V0c) / TC   see the MCP9701 datasheet for V0c and TC
    
    float tempC = ((((float)tempRawAD * 5.0) / 1024.0) - 0.400) / .0195; 
    Serial.print(tempC);
    Serial.println(" C");

    /*float WS_MPH = pow((((windADunits - 1.336) / (3.038517 * (pow(abs(tempC),0.115157)))) / 0.087288),3.009364) ;
    Serial.print(WS_MPH);
    Serial.print(" MPH\t"); */

    delay(1000);
}
