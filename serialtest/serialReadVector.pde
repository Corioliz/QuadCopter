  int kakka[6] = {0, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println( Serial.available() );
  delay(1000);

  if (readCsvToVector(kakka)) {
    for (byte i = 0; i < 6; i++){
      Serial.print(kakka[i]); Serial.print(",");
    }
    Serial.print('\n');
  }
}

boolean readCsvToVector(int* pidVector) {
  byte len; 
  if (Serial.available() <= 0)
    return 0;
  len = Serial.available();
  char stream[20];
  char number[5];
  // Read the message
  for (byte i = 0; i < len; ++i) {
    stream[i] = Serial.read();
  }
  byte i = 0, j = 0, k = 0;
  while ( i < len ) {
    while (stream[i] != ',' && i < len) {
      number[j] = stream[i];
      ++i;
      ++j;
    }
    ++i;
    number[j] = '\0';
    j = 0;
    pidVector[k++] = atoi(number);
  }
  return 1;
}


