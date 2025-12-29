
#  Vehicle Tilt/Roll Monitor | Système de Surveillance d'Inclinaison Véhicule

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/Platform-STM32-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)

*Real-time vehicle tilt monitoring system with Electronic Stability Control (ESC) simulation*

*Système de surveillance en temps réel de l'inclinaison véhicule avec simulation du contrôle électronique de stabilité (ESC)*

---

## 🇬🇧 English

###  Description

A real-time embedded system that monitors vehicle roll angle using an IMU sensor and simulates Electronic Stability Control (ESC) intervention. The system features a 4-level warning system with progressive motor speed reduction, visual indicators, and audible alerts to prevent vehicle rollover.

This project demonstrates:
- IMU sensor integration and calibration
- Real-time angle calculation from accelerometer data
- Multi-level state machine with hysteresis
- PWM motor control with dynamic speed adjustment
- Multi-peripheral coordination (I2C, UART, GPIO, Timers)

###  Features

- **Real-time Roll Angle Detection** using MPU6500 6-axis IMU
- **4-Level Warning System** with hysteresis to prevent oscillation
  - 🟢 STABLE: Normal operation (< 17°)
  - 🟡 WARNING: Caution required (17-32°)
  - 🔴 DANGER: Critical tilt (32-47°)
  - 🚨 ROLLOVER: Vehicle rolling (> 47°)
- **Progressive Motor Intervention**
  - 100% → 70% → 40% → 0% based on tilt severity
- **Multi-Sensory Feedback**
  - Color-coded LED indicators
  - Variable frequency buzzer (slow/fast/continuous)
  - Real-time OLED display
- **Automatic Sensor Calibration** (1000-sample averaging)
- **Printf Debug Output** via UART

###  Hardware Components

| Component | Model | Purpose |
|-----------|-------|---------|
| Microcontroller | STM32F401RE Nucleo | Main controller |
| IMU Sensor | MPU6500 (6-axis) | Roll angle detection |
| Display | SSD1306 OLED (128x64) | Real-time data visualization |
| Motor Driver | DC motor control |
| LEDs | Green, Yellow, Red | Visual state indicators |
| Buzzer | Active Buzzer | Audible warnings |

###  Pin Configuration
```
STM32F401RE Pinout:
├── I2C1 (MPU6500 + OLED)
│   ├── PB8 → SCL
│   └── PB9 → SDA
├── UART2 (Debug)
│   ├── PA2 → TX
│   └── PA3 → RX
├── TIM1_CH1 (PWM)
│   └── PA8 → Motor PWM
├── GPIO Outputs
│   ├── PB3 → Green LED
│   ├── PB4 → Yellow LED
│   ├── PB5 → Red LED
│   ├── PB6 → Buzzer
│   └── PA8 → Motor Control
```

###  State Machine Logic
```
         ┌──────────┐
         │  STABLE  │ < 17°
         │ Green LED│ Motor: 100%
         └─────┬────┘
               │ > 17°
         ┌─────▼────┐
         │ WARNING  │ 17-32°
         │Yellow LED│ Motor: 70%, Slow beep
         └─────┬────┘
               │ > 32°
         ┌─────▼────┐
         │  DANGER  │ 32-47°
         │  Red LED │ Motor: 40%, Fast beep
         └─────┬────┘
               │ > 47°
         ┌─────▼────┐
         │ ROLLOVER │ > 47°
         │Red Blink │ Motor: STOP, Alarm
         └──────────┘

Note: 2° hysteresis prevents state oscillation
```

### Quick Start

1. **Hardware Setup**
   - Connect components according to pin configuration
   - Ensure MPU6500 is mounted flat and stable

2. **Software**
   - Open project in STM32CubeIDE
   - Configure I2C1 (100kHz), UART2 (115200), TIM1 (PWM)
   - Build and flash

3. **Calibration**
   - Keep sensor still during startup
   - Wait for "Calibration complete" message
   - System ready after 20 seconds

4. **Operation**
   - Monitor serial output for roll angles
   - Check OLED for real-time state
   - Tilt breadboard to test state transitions

###  Project Structure
```
Vehicle-Tilt-Roll-Monitor/
├── src/
│   ├── main.c              # Main application logic
│   └── mpu6500.c          # MPU6500 driver implementation
├── inc/
│   └── mpu6500.h          # MPU6500 driver header
├── drivers/
│   └── ssd1306/           # OLED display driver
├── docs/
│   └── demo.mp4           # Demonstration video
└── README.md
```

###  Demo Video

[Watch Video](Demo_video/Demo.mp4)
## Challenges & Solutions

### Challenge 1: Sensor Drift and Noise
**Problem:** Raw accelerometer readings showed ±100 units of noise even when stationary, causing unstable angle calculations.

**Solution:**
- Implemented 1000-sample calibration routine with offset compensation
- Used 32-bit accumulators to prevent overflow during averaging
- Added 2ms delay between samples to ensure fresh data

### Challenge 2: State Oscillation at Thresholds
**Problem:** When roll angle hovered around thresholds (e.g., 15°), the system rapidly switched between states, causing LED/buzzer flicker.

**Solution:**
- Implemented hysteresis with 2° gap (e.g., 17° to enter WARNING, 13° to return to STABLE)
- State machine remembers previous state to make intelligent transitions
- Eliminated flickering and created smooth, professional behavior


### Challenge 3: Printf Float Formatting
**Problem:** Printf showed garbage characters (`Â°`) instead of float values due to missing float support in newlib-nano.

**Solution:**
- Enabled "Use float with printf from newlib-nano" in MCU settings
- Added linker flag for float support
- Float values now display correctly in serial terminal




---
###  Key Technical Details

**MPU6500 Driver:**
- Custom I2C driver built from scratch
- 14-byte burst read for efficiency
- Automatic offset calibration
- Roll angle calculation using `atan2(acc_y, acc_z)`

**State Machine:**
- Hysteresis implementation (2° gap)
- Prevents rapid state switching at thresholds
- Smooth transitions between warning levels

**PWM Motor Control:**
- Timer: TIM1, Frequency: 1kHz
- Prescaler: 83, Period: 999
- Dynamic duty cycle: 999/699/399/0

### 📈 Performance

- **Update Rate:** 10 Hz
- **Angle Accuracy:** ±0.5°
- **State Transition Time:** < 100ms
- **Calibration Time:** 20 second


##  Future Improvements

- **Gyroscope Integration**: Implement complementary filter combining accelerometer and gyroscope for drift-free angle tracking
-  **CAN Bus Communication**: Interface with other vehicle ECUs to demonstrate automotive networking
-  **FreeRTOS Integration**: Multi-task architecture with separate priorities for sensor, display, and control
-  **Data Logging**: Save tilt events and statistics to flash memory
-  **Predictive Algorithm**: Calculate roll rate (°/s) to detect rollover earlier

##  Skills Developed
### Technical Skills
 **I2C Communication**: Custom MPU6500 driver built from scratch  
 **Sensor Calibration**: 1000-sample averaging with offset compensation  
**State Machine Design**: 4-state system with hysteresis anti-oscillation  
 **Real-Time Systems**: Non-blocking timing with multi-peripheral coordination  
 **PWM Motor Control**: Dynamic duty cycle adjustment (100% → 0%)  

### Engineering Practices
 **Modular Design**: Separated driver code from application logic  
**Problem Solving**: Overcame sensor noise, state oscillation, and timing challenges  

### Automotive Context
 **Safety-Critical Systems**: Progressive intervention and multi-level warnings  
 **ESC Concepts**: Roll angle monitoring and stability control simulation  


---

## 🇫🇷 Français

###  Description

Système embarqué temps réel qui surveille l'angle de roulis du véhicule à l'aide d'un capteur IMU et simule l'intervention du contrôle électronique de stabilité (ESC). Le système dispose d'un système d'alerte à 4 niveaux avec réduction progressive de la vitesse du moteur, des indicateurs visuels et des alertes sonores pour prévenir le renversement du véhicule.

Ce projet démontre :
- Intégration et calibration de capteur IMU
- Calcul d'angle en temps réel à partir de données d'accéléromètre
- Machine à états multi-niveaux avec hystérésis
- Contrôle moteur PWM avec ajustement dynamique de vitesse
- Coordination multi-périphérique (I2C, UART, GPIO, Timers)

###  Fonctionnalités

- **Détection d'angle de roulis en temps réel** avec MPU6500 IMU 6 axes
- **Système d'alerte à 4 niveaux** avec hystérésis anti-oscillation
  - 🟢 STABLE : Fonctionnement normal (< 17°)
  - 🟡 AVERTISSEMENT : Attention requise (17-32°)
  - 🔴 DANGER : Inclinaison critique (32-47°)
  - 🚨 RENVERSEMENT : Véhicule en train de se renverser (> 47°)
- **Intervention moteur progressive**
  - 100% → 70% → 40% → 0% selon la gravité de l'inclinaison
- **Retour multi-sensoriel**
  - Indicateurs LED codés par couleur
  - Buzzer à fréquence variable (lent/rapide/continu)
  - Affichage OLED en temps réel
- **Calibration automatique du capteur** (moyenne sur 1000 échantillons)
- **Sortie de débogage Printf** via UART

###  Vidéo de Démonstration

[Watch Video](Demo_video/Demo.mp4)

###  Détails Techniques Clés

**Pilote MPU6500 :**
- Pilote I2C personnalisé développé from scratch
- Lecture en rafale de 14 octets pour l'efficacité
- Calibration automatique des offsets
- Calcul d'angle de roulis avec `atan2(acc_y, acc_z)`

**Machine à États :**
- Implémentation d'hystérésis (écart de 2°)
- Empêche les changements d'état rapides aux seuils
- Transitions fluides entre les niveaux d'alerte

---

##  License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

##  Author

**Wassim Kessaissia**

- GitHub: [@wassimkessaissia](https://github.com/wassimkessaissia)
- LinkedIn: (https://www.linkedin.com/in/wassim-kessaissia-6aa0472b7/)
- Email: wassimkessaissia8@gmail.com

##  Acknowledgments

- STM32 HAL Library
- MPU6500 Datasheet - InvenSense
- SSD1306 OLED Driver

---

**⭐ If you found this project helpful, please consider giving it a star!**
