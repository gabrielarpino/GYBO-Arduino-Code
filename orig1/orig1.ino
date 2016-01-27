 if (gotTrigger == 1)
  {


    /////////////////////// Moisture Sensor /////////////////////////////////////////

    Serial.begin(57600);

    //Serial.print("Moisture Sensor Value:");
    int moisture = analogRead(A0);
    Serial.begin(9600);
    
    if (moist < 300 | moist == 300 | moist == 0){
      uart.print("The soil is dry");
    }
    if (moist > 700 | moist == 700){
      uart.print("The soil has overflowed with water");
    }
    if (moist > 300 && moist < 700){
      uart.print("The soil is humid");
    }
    delay(5000);
    //Serial.println("We got our TRIGGER!");
    //uart.print("TRIGGER");


    ////////////////////// Temperature /////////////////////////

    float celsius = getTemperature();
    uart.print("Celsius:  ");
    uart.print(celsius);
    delay(5000);

    //////////////////////// Light sensor //////////////////////////

    int LDRReading = analogRead(LDR_Pin);
    if (LDRReading < 150 | LDRReading == 0 | (LDRReading == 150)){
      uart.print("It is bright");
    }
    if ((LDRReading > 150 && LDRReading < 512) | LDRReading == 512){
      uart.print("It is well lit");
    }
    if ((LDRReading > 512) && (LDRReading < 800)){
      uart.print("It is dark");
    }
    if ((LDRReading == 800) && (LDRReading > 800)){
      uart.print("It is very dark");
    }
    delay(5000);

    //////////////////////// Humidity ////////////////////////

    //To properly caculate relative humidity, we need the temperature.
    float temperature = celsius; //replace with a thermometer reading if you have it
    float relativeHumidity  = getHumidity(temperature);

    if (relativeHumidity < 25 | relativeHumidity == 0 | (relativeHumidity == 25)){
      uart.print("It feels uncomfortably dry. Relative Humidity: ");
    }
    if (relativeHumidity > 60 | relativeHumidity == 60){
      uart.print("It feels uncomfortably wet. Relative Humidity: ");
    }
    if (relativeHumidity < 60 && relativeHumidity > 25){
      uart.print("It feels comfortable. Relative Humidity: ");
    }
    uart.print(relativeHumidity);
    
    delay(5000); //just here to slow it down so you can read it   

    ///////////////////// Force Sensor ///////////////////////

    int FSRReading = analogRead(FSR_Pin); 

    uart.print("Force: ");
    uart.print(FSRReading);
    delay(250); //just here to slow down the output for easier reading
}
//////////////////////////// Individual Stats ///////////////////////////////

////////////////////////// Moisture //////////////////////////////////////
  
  if (buffer[0] == 'm' && buffer[1] == 'o' && buffer[2] == 'i' && buffer[3] == 's' && buffer[4] == 't' && buffer[5] == 'u' && buffer[6] == 'r' && buffer[7] == 'e'){
    Serial.begin(57600);
    //Serial.print("Moisture Sensor Value:");
    int moist = analogRead(A0);
    Serial.begin(9600);
    
    if (moist < 300 | moist == 300 | moist == 0){
      uart.print("The soil is dry");
    }
    if (moist > 700 | moist == 700){
      uart.print("The soil has overflowed with water");
    }
    if (moist > 300 && moist < 700){
      uart.print("The soil is humid");
    }

    

    }
////////////////////////////// Temp ///////////////////////////////////
  if (buffer[0] == 't' && buffer[1] == 'e' && buffer[2] == 'm' && buffer[3] == 'p' && buffer[4] == 'e' && buffer[5] == 'r' && buffer[6] == 'a' && buffer[7] == 't' && buffer[8] == 'u' && buffer[9] == 'r' && buffer[10] == 'e'){
    float celsius = getTemperature();
    uart.print("Celsius:  ");
    uart.print(celsius);
  }
////////////////////////// Humid //////////////////////////////////////
  if (buffer[0] == 'h' && buffer[1] == 'u' && buffer[2] == 'm' && buffer[3] == 'i' && buffer[4] == 'd' && buffer[5] == 'i' && buffer[6] == 't' && buffer[7] == 'y'){
    float celsius = getTemperature();
    float temperature = celsius; //replace with a thermometer reading if you have it
    float relativeHumidity  = getHumidity(temperature);

    
    if (relativeHumidity < 25 | relativeHumidity == 0 | (relativeHumidity == 25)){
      uart.print("It feels uncomfortably dry. Relative Humidity: ");
    }
    if (relativeHumidity > 60 | relativeHumidity == 60){
      uart.print("It feels uncomfortably wet. Relative Humidity: ");
    }
    if (relativeHumidity < 60 && relativeHumidity > 25){
      uart.print("It feels comfortable. Relative Humidity: ");
    }
    uart.print(relativeHumidity);
  }

///////////////////////// Light ////////////////////////////////////////
  if (buffer[0] == 'l' && buffer[1] == 'i' && buffer[2] == 'g' && buffer[3] == 'h' && buffer[4] == 't'){
    int LDRReading = analogRead(LDR_Pin);
    
    if (LDRReading < 150 | LDRReading == 0 | (LDRReading == 150)){
      uart.print("It is bright");
    }
    if ((LDRReading > 150 && LDRReading < 512) | LDRReading == 512){
      uart.print("It is well lit");
    }
    if ((LDRReading > 512) && (LDRReading < 800)){
      uart.print("It is dark");
    }
    if ((LDRReading == 800) && (LDRReading > 800)){
      uart.print("It is very dark");
    }
   }
//////////////////// Force /////////////////////////////////////////////
    if (buffer[0] == 'f' && buffer[1] == 'o' && buffer[2] == 'r' && buffer[3] == 'c' && buffer[4] == 'e'){
    int FSRReading = analogRead(FSR_Pin); 

    uart.print("Force: ");
    uart.print(FSRReading);}



}
