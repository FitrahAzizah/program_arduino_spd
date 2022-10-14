#include <Wire.h>
#include <VL53L0X.h>

#define dirPin1 2
#define stepPin1 3
#define dirPin2 4
#define stepPin2 5
#define LS1 7
#define LS2 50
#define PB1 47 //PushButton 1 (Merah)
#define PB2 49
#define PB3 51
#define PB4 53 //PushButton 4 (hijau)
#define degSens 9
#define stepsPerRevolution 800 // based on SW1-SW3 settings on motor driver's module

VL53L0X sensor;

const float cmConv = 1.25;
const int degConv = 2;
int mtSpd = 1000; //in milli second to control motor stepper speed
int statePB1 = 0;
int statePB2 = 0;
int statePB3 = 0;
int statePB4 = 0;

int readDist();
float WeightedAverageFilter(float incomingValue, float previousValue);


void setup() {
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(LS1, INPUT);
  pinMode(LS2, INPUT);
  pinMode(PB1, INPUT);
  pinMode(PB2, INPUT);
  pinMode(PB3, INPUT);
  pinMode(PB4, INPUT);
  pinMode(degSens, INPUT);

  digitalWrite(LS1, HIGH);
  digitalWrite(LS2, HIGH);
  digitalWrite(PB1, HIGH);
  digitalWrite(PB2, HIGH);
  digitalWrite(PB3, HIGH);
  digitalWrite(PB4, HIGH);

  Serial.begin(9600);
  Serial.println("This device is ready to rock!");

  Wire.begin();

  sensor.init();
  sensor.setTimeout(500); // Set sensor timeout in milliseconds (0 = no timeout)
  sensor.startContinuous(); // Sensing interval

  if ((LS1 != false) || (degSens != false)) {
    goHome(100);
  }

}

void fwd(int spd, int cm) {
  digitalWrite(dirPin1, HIGH); // putar searah jarum jam
  for (unsigned long i = 0; i < stepsPerRevolution * cm * cmConv; i++) {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(spd); // ganti delay untuk mempercepat motor
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(spd); // ganti delay untuk mempercepat motor

    boolean sens = digitalRead(LS2);
    if (sens == false) break;

  }
}

void rwd(int spd, int cm) {
  digitalWrite(dirPin1, LOW); // putar searah jarum jam
  for (unsigned long i = 0; i < stepsPerRevolution * cm * cmConv; i++) {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(spd); // ganti delay untuk mempercepat motor
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(spd); // ganti delay untuk mempercepat motor

    boolean sens = digitalRead(LS1);
    if (sens == false) break;
  }

}

void cw(int spd, int deg) {
  int stepDeg = deg / 0.9 * degConv;
  digitalWrite(dirPin2, HIGH); // putar searah jarum jam
  for (unsigned long i = 0; i < stepDeg; i++) {
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(2000); // ganti delay untuk mempercepat motor
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(2000); // ganti delay untuk mempercepat motor
  }
}

void ccw(int spd, int deg) {
  int stepDeg = deg / 0.9 * degConv;
  digitalWrite(dirPin2, LOW); // putar searah jarum jam
  for (unsigned long i = 0; i < stepDeg; i++) {
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(2000); // ganti delay untuk mempercepat motor
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(2000); // ganti delay untuk mempercepat motor
  }
}

void goHome(int spd) {
  boolean sens1, sens2;

  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  while (1) {
    while (1) {
      sens1 = digitalRead(LS1);
      if (sens1 == false) break;
      digitalWrite(stepPin1, HIGH);
      delayMicroseconds(spd); // ganti delay untuk mempercepat motor
      digitalWrite(stepPin1, LOW);
      delayMicroseconds(spd); // ganti delay untuk mempercepat motor
    }

    while (1) {
      sens2 = digitalRead(degSens);
      if (sens2 == false) break;
      digitalWrite(stepPin2, HIGH);
      delayMicroseconds(2000); // ganti delay untuk mempercepat motor
      digitalWrite(stepPin2, LOW);
      delayMicroseconds(2000); // ganti delay untuk mempercepat motor
    }

    //Serial.print(sens1);
    //Serial.println(sens2);
    if ((sens1 == false) && (sens2 == false)) break;

  }
}

void getSerial() {
  String buf = Serial.readStringUntil('\n');
  Serial.println(buf);

  int ind1 = buf.indexOf(',');  //finds location of first ,
  String SNoC = buf.substring(0, ind1);   //captures first data String
  int NoC = SNoC.toInt();
  int datLon = (NoC * 6);

  int dir1[NoC], dir2[NoC], dist[NoC], deg[NoC], spd[NoC], meTim[NoC];

  int bInd[datLon + 1];
  String bStr[datLon];

  bInd[0] = ind1;
  //Serial.println(datLon);
  for (int i = 0; i < datLon; i++) {
    bInd[i + 1] = buf.indexOf(',', bInd[i] + 1 ); //finds location
    bStr[i] = buf.substring(bInd[i] + 1, bInd[i + 1]); //captures second data String
    //    Serial.print(i);
    //    Serial.println(bStr[i]);
  }
  int Lp = NoC;
  for (int i = 0; i < NoC; i++) {
    for (int x = i * 6; x < i * 6 + 6; x++) {
      int a = abs((i * 6) - x);
      switch (a) {
        case 0 :
          dir1[i] = bStr[x].toInt();
          //          if(dir1[i] == 0)Serial.println("FWD");
          //          else Serial.println("RWD");
          break;
        case 1 :
          dir2[i] = bStr[x].toInt();
          //          if(dir2[i] == 0)Serial.println("CW");
          //          else Serial.println("CCW");
          break;
        case 2 :
          dist[i] = bStr[x].toInt();
          //          Serial.println(dist[i]);
          break;
        case 3 :
          deg[i] = bStr[x].toInt();
          //          Serial.println(deg[i]);
          break;
        case 4 :
          spd[i] = bStr[x].toInt();
          //          Serial.println(spd[i]);
          break;
        case 5 :
          meTim[i] = bStr[x].toInt();
          //          Serial.println(meTim[i]);
          break;
      }
    }
  }
  NoC = Lp;
  for (int k = 0; k < NoC; k++) {
    Serial.print("Measurement no. : ");
    Serial.println(k + 1);
    if (dir1[k] == 0) fwd(spd[k], dist[k]);
    else rwd(spd[k], dist[k]);
    if (dir2[k] == 0) cw(spd[k], deg[k]);
    else ccw(spd[k], deg[k]);
    delay(meTim[k]);
  }
  //goHome(100);
}

int readDist() {
  //  unsigned int reading;
  //
  //  for (char i = 0; i < 10; i++) {
  int bReading = sensor.readRangeContinuousMillimeters();
  //    reading += bReading;
  //  }
  //  reading = reading/10;
  return bReading;
}

void loop() {

  if (Serial.available()) {
    getSerial();
  }

  statePB1 = digitalRead(PB1);
  if (statePB1 == LOW) {
    fwd(500, 5);
  }
  else fwd(500, 0);

  statePB2 = digitalRead(PB2);
  if (statePB2 == LOW) {
    rwd(500, 5);
  }
  else rwd(500, 0);

  statePB3 = digitalRead(PB3);
  if (statePB3 == LOW) {
    cw(2000, 9);
  }
  else cw(500, 0);

  statePB4 = digitalRead(PB4);
  if (statePB4 == LOW) {
    ccw(2000, 9);
  }
  else ccw(500, 0);
  //  int Distance = readDist();
  //  Serial.print("Now Distance : ");
  //  Serial.println(Distance);

}
