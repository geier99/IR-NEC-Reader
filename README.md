# IR-NEC-Reader
IR-NEC code protocol reader - Tool for readintg out the IR-NEC code from remote controlls 

The software supports the ATMEL ATMega44/88/168 mcu

The software use the internal rc-oscilator ( 8MHz)  with an internal osc-calibration by reding out the calibration byte from eeprom address 0x0000. So don't forget to to write the oscal-byte by flashing to the eeprom adress 0x0000.

