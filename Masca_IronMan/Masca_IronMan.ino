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
  // Măsurăm distanța periodic
  distance = measureDistance();
  
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
  
  // Afișare informații pentru debugging
  if (millis() % 500 == 0) {
    Serial.print("Distanța: ");
    Serial.print(distance);
    Serial.print(" cm | Stare: ");
    Serial.println(currentState ? "APROAPE" : "DEPARTE");
  }
  
  delay(100);
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
// FUNCȚII PWM - CONTROLUL SERVOMOTOARELOR
// =========================
void moveServosTo(int angle) {
  Serial.print("Mutăm servomotoarele la: ");
  Serial.print(angle);
  Serial.println(" grade");
  
  // Mutăm ambele servomotoare la unghiul specificat
  servo1.write(angle);
  servo2.write(angle);
}

void activateProximityMode() {
  Serial.println("ACTIVARE: Obiect detectat aproape!");
  
  // PWM: Rotim servomotoarele la 90 de grade
  moveServosTo(90);
  
  // Redăm primul fișier audio
  delay(500); // Așteptăm ca servomotoarele să se miște
  dfPlayer.play(1); // Redă 001.mp3
  
  Serial.println("Redare: 001.mp3");
}

void deactivateProximityMode() {
  Serial.println("DEZACTIVARE: Obiectul s-a îndepărtat!");
  
  // PWM: Rotim servomotoarele la -90 de grade (sau 270)
  moveServosTo(270); // Echivalent cu -90 grade
  
  // Redăm al doilea fișier audio
  delay(500); // Așteptăm ca servomotoarele să se miște
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
  Serial.println("- Servomotor 1: Pin 9 (PWM)");
  Serial.println("- Servomotor 2: Pin 10 (PWM)");
  Serial.println("- DFPlayer: Pin 7 (RX), Pin 8 (TX)");
  Serial.print("- Prag detecție: ");
  Serial.print(THRESHOLD_DISTANCE);
  Serial.println(" cm");
  Serial.println("=========================");
}