# DEEED-Open-Source-Flightcomputer

The DEEED Flighcomputer is an easy to Build open source Flighcomputer/Altimeter for Model Rockets.
It is Designed around commonly used Breakout Boards for easy assembly and use. I designed it with the Intent do be a good first PCB-Projekt for myself and every Hobbiist.
The Progarmmin side of the Project was mostly done by Claude Ai as i myself can not realy programm and this exeelerated this Part extremly 
I recomend you to write your own code or improve the existing Code and design if you can or want. I Hope you have a great experience 

Technical Overwiew:
As an MCU and IMU aswell as Blouthoothbeacon for easier recovery it uses the XIAO nRF 52840 sense.
For the BArometer i chose the BMP 280 (the six pin Breakoutboard version)
For a stable 5Volt Power supply the Pololu 4941 
For Datalogging the Adafruit 4682 SD Card BRaekout Board ist used 
The Buzzer is a standart 5 Volt Buzzer found in most electronic startetkits 
The two Mosfets are N-Channel IRLZ44N, they are used for the Buzzer and the Pyrochannel.
The Board is 24mm wide and 106mm tall you can fit the Flightcomputer in an 3

Code Overview:
The Current Software is DEEEDSoft V...
It can..

