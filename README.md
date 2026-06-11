This repository contains files with code developed for the Pinal Year Project entitled "Designing a a Signal Acquisition Platform for Monitoring of
Patients with Traumatic Brain Injury" as a part of the award of MEng Biomedical Engineering from Imperial College London. 

File contents:
- logger.m is a MATLAB file containing code to receive data from all channels from the Arduino Due and store it in a CSV and Excel files;
- calibration_summary.m is a MATLAB file that uses the staored data to derive amperometric electrode calibration features and provide new sensitivity factor;
- Arduino_ADS is a C++ file that runs the Arduino. It allows for communication with ADS124S08 and ADS1299 to receive channel specific data, runs the amperometric electrode interrogation and sends the data to MATLAB. This code is still being validated during hardware testing. 
- Arduino_ADS_simulation was the code for data transmission and amperometric electrode interrogation used for pre-hardware implementation testing. It uses a simulated ADC data to check logic of the interrogation routine and data transmission. 
