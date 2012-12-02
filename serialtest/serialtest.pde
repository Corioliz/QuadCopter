

//byte index = 0;
int apukerroin = 1;
byte bytelen = 0;

byte readvalue = 0;
int readvec[25] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//int tempvec[5] = {0, 0, 0, 0, 0};
float tempvalue = 0;
float tempvec[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

byte index = 0;

void setup() {
  
  Serial.begin(9600); //Open serial port
  
}

void loop() {
  
  //Read serial for command values 
  if( Serial.available() > 0 ) {
    //changeval = 1;
    index = 4;
    //readvalue = 0;
    bytelen = Serial.available();
    apukerroin = 1;
    tempvalue = 0;
    
    for( int i = 0; i < bytelen; i++) {
       readvec[i] = int(Serial.read() - 48);
    }
    
    for( int i = bytelen - 1; i >= 0 ; i--) {
      
      /* 
      tempvalue += readvec[i] * apukerroin;
      apukerroin *= 10; 
      */
      
       if( readvec[i] == (-4) ) {
         tempvec[index] = tempvalue;
         index--;
         tempvalue = 0;
         apukerroin = 1;
       } else {
         tempvalue += readvec[i] * apukerroin;
         apukerroin *= 10; 
       }
       
    }
    
    tempvec[index] = tempvalue;
    //index--;
    tempvalue = 0;
    apukerroin = 1;
    
    Serial.println("Values");
    
    //
    for(int i = index; i < 10; i++){
      Serial.println(tempvec[i], DEC); //Print value
    }
    //
    
    //Serial.println(tempvalue,DEC);
    
  }
  
  delay(250);
  
}

/*
void loop() {
  
  while( Serial.available() > 0 ) {
    
    readvalue = Serial.read() - 48;
    
    if( readvalue == '\n') {
      
      apukerroin = 1;
      
      for( int i = index - 1; i >= 0 ; i--) {
         tempvec += readvec[i] * apukerroin;
         apukerroin *= 10; 
      }
      
      Serial.println(tempvec);
      
      index = 0;
      readvec[index] = NULL;
      
    } else {
      
      readvec[index] = readvalue;
      index++;
      readvec[index] = '\0';
      
    }
    
  }
  
}
*/
