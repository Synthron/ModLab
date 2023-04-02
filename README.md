# ModLab

## A modular station for your electronic workbench

The ModLab is designed to be housed in a 19" Rack with 3U.  
It is the improved version of the (failed) Arduino Workstation I built a long time ago.

But now It will be bigger, better and way too complicated.

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
- [X] Cleanup Repo

---

## Hardware

- [X] Redesign modules with STM32
  - [x] Master Bus Controller with F767
    - [X] Review
    - [X] Ordered
    - [X] Assembled
  - [x] Diode/LED-Tester
    - [X] V1
      - [X] Review
      - [X] Ordered
      - [X] Assembled
    - [X] V2
      - [X] Review
      - [X] Ordered
      - [X] Assembled
  - [X] Variable Power Supply
    - [X] Review
    - [X] Ordered
    - [X] Assembled
  - [x] Symmetric Power Supply
    - [X] Review
    - [X] Ordered
    - [X] Assembled
  - [x] Waveform generator
    - [x] Review
    - [X] Ordered
    - [X] Assembled
  - [ ] DSO-138 clone?
  - [ ] Electronic Load
    - [X] Review
    - [X] Ordered
    - [ ] Assembled
  - [ ] Logic Tester
  - [ ] Continuity tester
  - [ ] Speaker Module
  - [x] Termination boards and Bus Extenders
    - [x] Review
    - [X] Ordered
    - [X] Assembled

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
  - [ ] Testing Sequence
  - [X] ADC
- SymPSU
  - [X] PWM Signals
  - [X] ADC
  - [X] CAN-Interface
- Power SMPS
  - [ ] PWM Signals
  - [ ] ADC
  - [ ] CAN-Interface
- Waveform Generator
  - [ ] PWM Signals
  - [ ] AD9833
  - [ ] CAN-Interface
- Power Monitor
  - [ ] Voltage Measurement
  - [ ] Current Measurement
  - [ ] I2C communication