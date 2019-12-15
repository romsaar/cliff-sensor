#define sensorR A0 // Sharp IR Right
#define sensorL A1 // sharp IR Left

void setup() {
  Serial.begin(9600); // start the serial port
}

void loop() {
  
  // 5v vcc
  float ARR = analogRead(sensorR);// value from right sensor 
  bool right_IR = true;
  float ARL = analogRead(sensorL);// value from left sensor 
  bool left_IR = true; 
    Serial.print("right analog read ");
    Serial.println(ARR);
    Serial.print("left analog read ");
    Serial.println(ARL);
   
   if (ARR >= 400) // 7cm value
    right_IR=true; // right ir sensor find deck
    
    else
    right_IR=false; // right ir sensor find cliff
    
    if (ARL >= 400)  // 7cm value
    left_IR=true; // left ir sensor find deck
    
    else
    left_IR=false; // left ir sensor find cliff
    
    delay(1000); // slow down serial port 

    

  }
