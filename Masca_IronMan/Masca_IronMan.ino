#include <Servo.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// =========================
// NOȚIUNI INTEGRATE:
// 1. GPIO - pentru senzorul ultrasonic
// 2. PWM - pentru controlul servomotoarelor
// 3. Întreruperi - pentru detecție eficientă
// =========================

// Definirea pinilor pentru senzorul ultrasonic HC-SR04
#define TRIG_PIN 2
#define ECHO_PIN 3

// Definirea pinilor pentru servomotoare (PWM)
#define SERVO1_PIN 4
#define SERVO2_PIN 5

// Definirea pinilor pentru DFPlayer
#define DFPLAYER_RX 6
#define DFPLAYER_TX 7

// Crearea obiectelor
Servo servo1;
Servo servo2;
SoftwareSerial dfPlayerSerial(DFPLAYER_TX, DFPLAYER_RX);
DFRobotDFPlayerMini dfPlayer;

// Variabile pentru distanță și stare
volatile bool proximityDetected = false;
volatile unsigned long lastInterruptTime = 0;
float distance = 0;
bool isClose = false;
bool lastState = false;

// Variabile pentru timeout măsurare distanță
unsigned long lastMeasurementTime = 0;
const unsigned long MEASUREMENT_INTERVAL = 1000; // 5 secunde în milisecunde

// Variabile pentru controlul vitezei servomotoarelor
int currentAngle1 = 0;  // Poziția curentă servo1
int currentAngle2 = 0;  // Poziția curentă servo2
int targetAngle1 = 0;   // Poziția țintă servo1
int targetAngle2 = 0;   // Poziția țintă servo2
const int SERVO_SPEED = 2;      // Viteza mișcării (grade pe pas)
const int SERVO_DELAY = 15;     // Delay între pași (ms)

// Constante
const float THRESHOLD_DISTANCE = 30.0; // 30 cm
const unsigned long DEBOUNCE_TIME = 200; // 200ms pentru debounce

void setup() {
  Serial.begin(9600);
  
  // =========================
  // 1. CONFIGURARE GPIO
  // =========================
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // =========================
  // 2. CONFIGURARE PWM (Servomotoare)
  // =========================
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  
  // Poziția inițială a servomotoarelor
  servo1.write(0);
  servo2.write(0);
  currentAngle1 = 0;
  currentAngle2 = 0;
  targetAngle1 = 0;
  targetAngle2 = 0;
  
  // =========================
  // 3. CONFIGURARE ÎNTRERUPERI
  // =========================
  // Configurăm întreruperea pe pinul ECHO pentru detecție rapidă
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoInterrupt, CHANGE);
  
  // =========================
  // CONFIGURARE DFPLAYER
  // =========================
  dfPlayerSerial.begin(9600);
  
  Serial.println("Inițializare DFPlayer...");
  if (!dfPlayer.begin(dfPlayerSerial)) {
    Serial.println("Eroare la inițializarea DFPlayer!");
    Serial.println("Verifică conexiunile și cardul SD!");
    while(true) {
      delay(0);
    }
  }
  Serial.println("DFPlayer inițializat cu succes!");
  
  // Configurare volum (0-30)
  dfPlayer.volume(20);
  
  Serial.println("Sistem inițializat!");
  Serial.println("Apropie-te la mai puțin de 30cm pentru a activa...");
  
  delay(1000);
}

void loop() {
  // Măsurăm distanța doar la fiecare 5 secunde
  unsigned long currentTime = millis();
  if (currentTime - lastMeasurementTime >= MEASUREMENT_INTERVAL) {
    distance = measureDistance();
    lastMeasurementTime = currentTime;
    
    Serial.print("Măsurare nouă - Distanța: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    // Verificăm schimbarea stării
    bool currentState = (distance < THRESHOLD_DISTANCE && distance > 0);
    
    if (currentState != lastState) {
      if (currentState) {
        // Obiect detectat aproape
        activateProximityMode();
      } else {
        // Obiectul s-a îndepărtat
        deactivateProximityMode();
      }
      lastState = currentState;
    }
  }
  
  // Actualizăm poziția servomotoarelor (mișcare lentă)
  updateServoPositions();
  
  // Afișare informații pentru debugging (mai rar)
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime >= 2000) { // La fiecare 2 secunde
    Serial.print("Status - Ultima distanță: ");
    Serial.print(distance);
    Serial.print(" cm | Stare: ");
    Serial.print(lastState ? "APROAPE" : "DEPARTE");
    Serial.print(" | Servo1: ");
    Serial.print(currentAngle1);
    Serial.print("° | Servo2: ");
    Serial.print(currentAngle2);
    Serial.print("° | Timp până la următoarea măsurare: ");
    Serial.print((MEASUREMENT_INTERVAL - (currentTime - lastMeasurementTime)) / 1000);
    Serial.println("s");
    lastDebugTime = currentTime;
  }
  
  delay(20); // Delay mai mic pentru mișcare mai fluidă
}

// =========================
// FUNCȚII PENTRU MIȘCARE LENTĂ SERVOMOTOARE
// =========================
void updateServoPositions() {
  bool moved = false;
  
  // Actualizăm servo1
  if (currentAngle1 != targetAngle1) {
    if (currentAngle1 < targetAngle1) {
      currentAngle1 += min(SERVO_SPEED, targetAngle1 - currentAngle1);
    } else {
      currentAngle1 -= min(SERVO_SPEED, currentAngle1 - targetAngle1);
    }
    servo1.write(currentAngle1);
    moved = true;
  }
  
  // Actualizăm servo2
  if (currentAngle2 != targetAngle2) {
    if (currentAngle2 < targetAngle2) {
      currentAngle2 += min(SERVO_SPEED, targetAngle2 - currentAngle2);
    } else {
      currentAngle2 -= min(SERVO_SPEED, currentAngle2 - targetAngle2);
    }
    servo2.write(currentAngle2);
    moved = true;
  }
  
  // Dacă s-a mișcat cel puțin un servo, așteptăm
  if (moved) {
    delay(SERVO_DELAY);
  }
}

void moveServosToSlow(int angle1, int angle2) {
  Serial.print("Setăm servomotoarele la: ");
  Serial.print(angle1);
  Serial.print("° și ");
  Serial.print(angle2);
  Serial.println("°");
  
  // Setăm pozițiile țintă
  targetAngle1 = constrain(angle1, 0, 180);
  targetAngle2 = constrain(angle2, 0, 180);
}

// Funcție pentru a mișca ambele servomotoare la același unghi
void moveServosToSlow(int angle) {
  moveServosToSlow(angle, angle);
}

// Funcție pentru a aștepta ca servomotoarele să ajungă la poziție
void waitForServos() {
  while (currentAngle1 != targetAngle1 || currentAngle2 != targetAngle2) {
    updateServoPositions();
    delay(1);
  }
}

// =========================
// FUNCȚIE ÎNTRERUPERE
// =========================
void echoInterrupt() {
  // Debounce pentru întrerupere
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime < DEBOUNCE_TIME) {
    return;
  }
  lastInterruptTime = currentTime;
  
  // Setăm flag-ul pentru detecție rapidă
  proximityDetected = true;
}

// =========================
// FUNCȚII GPIO - MĂSURAREA DISTANȚEI
// =========================
float measureDistance() {
  // Trimitem puls trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Măsurăm durata pulsului echo
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout 30ms
  
  // Calculăm distanța în cm
  float distance = (duration * 0.034) / 2;
  
  // Returnăm 0 dacă nu am primit răspuns valid
  return (duration == 0) ? 0 : distance;
}

// =========================
// FUNCȚII PWM - CONTROLUL SERVOMOTOARELOR (ACTUALIZATE)
// =========================
void activateProximityMode() {
  Serial.println("ACTIVARE: Obiect detectat aproape!");
  
  // PWM: Rotim servomotoarele la 45 de grade (lent)
  moveServosToSlow(45);
  
  // Redăm primul fișier audio
  dfPlayer.play(1); // Redă 001.mp3
  
  Serial.println("Redare: 001.mp3");
}

void deactivateProximityMode() {
  Serial.println("DEZACTIVARE: Obiectul s-a îndepărtat!");
  
  // PWM: Rotim servomotoarele la 0 grade (lent)
  moveServosToSlow(0);
  
  // Redăm al doilea fișier audio
  dfPlayer.play(2); // Redă 002.mp3
  
  Serial.println("Redare: 002.mp3");
}

// =========================
// FUNCȚII UTILITARE
// =========================
void printSystemInfo() {
  Serial.println("=========================");
  Serial.println("INFORMAȚII SISTEM:");
  Serial.println("- Senzor ultrasonic: Pin 2 (TRIG), Pin 3 (ECHO)");
  Serial.println("- Servomotor 1: Pin 4 (PWM)");
  Serial.println("- Servomotor 2: Pin 5 (PWM)");
  Serial.println("- DFPlayer: Pin 6 (RX), Pin 7 (TX)");
  Serial.print("- Prag detecție: ");
  Serial.print(THRESHOLD_DISTANCE);
  Serial.println(" cm");
  Serial.print("- Interval măsurare: ");
  Serial.print(MEASUREMENT_INTERVAL / 1000);
  Serial.println(" secunde");
  Serial.print("- Viteza servo: ");
  Serial.print(SERVO_SPEED);
  Serial.println("°/pas");
  Serial.print("- Delay servo: ");
  Serial.print(SERVO_DELAY);
  Serial.println("ms");
  Serial.println("=========================");
}