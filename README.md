# ModLab

## A modular station for your electronic workbench

The ModLab is designed to be housed in a 19" Rack with 3U.  
It is the improved version of the (failed) Arduino Workstation I built a long time ago.

But now It will be bigger, better and way too complicated.

---

[GitHub](https://github.com/Synthron/ModLab)
[Gitlab](https://gitlab.synthron.de/Synthron/ModLab)

---

**Work in Progress**

---

# To-Dos

## Repo

- [X] Rewrite documentation in english
  - [ ] Write better introduction
  - [ ] explain what exactly the ModLab is
  - [X] update changes for TFT
  - [X] Explain TFT Protocol
  - [ ] Attach Schematics
- [X] Cleanup Repo

---

## Hardware

- [X] Redesign modules with STM32
  - [x] Master Bus Controller with F767
    - [X] V1
      - [X] Review
      - [X] Ordered
      - [X] Assembled
    - [ ] V2
      - [ ] Change LCD connector
      - [ ] omit external SRAM
      - [ ] include header for Reset
  - [x] Diode/LED-Tester
    - [X] V1
    - [X] V2
  - [X] Variable Power Supply
    - [X] V1
    - [ ] V2
      - [X] Fix Relais Footprint
      - [X] Feedback Dividers for correct level
      - [ ] Review
      - [ ] Ordered
      - [ ] Assembled
  - [x] Symmetric Power Supply
    - [X] V1
    - [ ] V2
      - [X] fix negative channel
      - [X] fix CC LEDs
      - [ ] Review
      - [ ] Ordered
      - [ ] Assembled
  - [x] Waveform generator
  - [ ] DSO-138 clone?
  - [X] Electronic Load
  - [ ] Logic Tester
  - [ ] Continuity tester
  - [ ] Speaker Module
  - [x] Termination boards and Bus Extenders

---

## Software

- General
  - [X] Define CAN Protocol
    - [X] Define Command Structure
    - [X] Define Data Structure
- Master Bus Controller
  - [ ] STM32 code
    - [X] USB for Debug
    - [X] LCD
      - [X] Get Nextion Display working
        - [X] UI
        - [X] µC Interface
    - [ ] Busses
      - [X] CAN
      - [ ] SPI - No Usecase so far
      - [ ] I2C
      - [ ] DAC - No Usecase so far
      - [ ] RS485 - No Usecase so far
  - [ ] ESP32 code
    - [ ] comm with STM
    - [X] OTA
    - [ ] basic web interface
- Diode Tester code
  - [X] CAN
  - [X] Testing Sequence
  - [X] ADC
- SymPSU
  - [X] PWM Signals
  - [X] ADC
  - [X] CAN-Interface
- Power SMPS
  - [X] PWM Signals
  - [X] ADC
  - [X] CAN-Interface
- Waveform Generator
  - [X] PWM Signals
  - [X] AD9833
  - [X] CAN-Interface
- Power Monitor
  - [ ] Voltage Measurement
  - [ ] Current Measurement
  - [ ] I2C communication

---

ModLab Lib Tracker:

1. SymPSU
2. Diode-Tester
3. SMPS
4. Load
5. F-Gen
