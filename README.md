# IronMan_Mask

> **Sistem inteligent de detecție proximitate cu feedback multi-modal (servo + audio)**


## Descriere

Sistem Arduino care detectează obiectele în proximitate folosind un senzor ultrasonic HC-SR04 și oferă feedback prin:
- ** Mișcare sincronizată** a două servomotoare 
- ** Redare audio contextuală** prin DFPlayer Mini
- ** Detecție în timp real** cu întreruperi hardware

###  Funcționalități Cheie

-  **Detecție precisă** sub 30cm cu senzor ultrasonic
-  **Control dual servo** sincronizat (+90°/-90°)
-  **Audio feedback** cu fișiere MP3 personalizabile
-  **Interrupt-driven** pentru performanță optimă
-  **Debounce protection** anti-false triggers
-  **Serial monitoring** pentru debugging


```
 Obiect detectat < 30cm:
    Servomotoare → +90°
    Redare: 001.mp3

 Obiect îndepărtat > 30cm:
    Servomotoare → -90° 
    Redare: 002.mp3
```

---

###  Componente

| Component | Cantitate | Descriere |
|-----------|-----------|-----------|
| Arduino UNO R3 | 1x | Microcontroller principal |
| Senzor Ultrasonic HC-SR04 | 1x | Detecție distanță |
| Servomotor SG90 | 2x | Actuatori PWM |
| DFPlayer Mini | 1x | Player audio MP3 |
| Boxă 3W | 1x | Output audio |
| Card MicroSD | 1x | Stocare fișiere audio |
| Breadboard | 1x | Prototipare conexiuni |
| Cabluri jumper | ~15x | Interconectări |

### Cerințe Alimentare
- **Arduino**: 5V via USB sau adaptor 7-12V
- **Servomotoare**: 5V (max 2A pentru ambele)
- **DFPlayer**: 3.3-5V
- **Recomandat**: Adaptor extern 5V/2A pentru stabilitate

---

###  Senzor Ultrasonic HC-SR04
```
HC-SR04    →    Arduino UNO
VCC        →    5V
GND        →    GND  
TRIG       →    Pin 2 (GPIO)
ECHO       →    Pin 3 (GPIO + INT1)
```

###  Servomotoare  
```
Servo 1    →    Arduino UNO
VCC (Roșu) →    5V
GND (Negru)→    GND
PWM (Galben)→   Pin 4

Servo 2    →    Arduino UNO  
VCC (Roșu) →    5V
GND (Negru)→    GND
PWM (Galben)→   Pin 5
```

### DFPlayer Mini + Boxă
```
DFPlayer   →    Arduino UNO
VCC        →    5V
GND        →    GND
RX         →    Pin 6
TX         →    Pin 7
SPK1       →    Boxă (+)
SPK2       →    Boxă (-)
```

---




### 2️ Instalarea Bibliotecilor

În Arduino IDE:
```
Tools → Manage Libraries → Căută și instalează:
```
- `DFRobotDFPlayerMini` by DFRobot
- `Servo` (pre-instalată)






### Funcții Principale

| Funcție | Descriere | Complexitate |
|---------|-----------|--------------|
| `measureDistance()` | Măsurare ultrasonică | O(1) |
| `moveServosTo(angle)` | Control PWM servomotoare | O(1) |
| `echoInterrupt()` | ISR pentru ECHO pin | O(1) |
| `activateProximityMode()` | Acțiune apropiere | O(1) |
| `deactivateProximityMode()` | Acțiune îndepărtare | O(1) |

---




## 👨‍💻 Autor

**[Robert]**
- 🐙 GitHub: [@username](https://github.com/Robert326)



<div align="center">


</div>
