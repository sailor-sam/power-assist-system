EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Sensor_Pressure:MPXAZ6115A U1
U 1 1 6065426E
P 4850 2650
F 0 "U1" H 4421 2696 50  0000 R CNN
F 1 "HELM PRESSURE SENSOR" H 4421 2605 50  0000 R CNN
F 2 "IML parts:MPXV7007" H 4350 2300 50  0001 C CNN
F 3 "http://www.nxp.com/files/sensors/doc/data_sheet/MPXA6115A.pdf" H 4850 3250 50  0001 C CNN
	1    4850 2650
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Pressure:MPXAZ6115A U5
U 1 1 6065518A
P 4850 3900
F 0 "U5" H 4421 3946 50  0000 R CNN
F 1 "WINCH PRESSURE SENSOR" H 4421 3855 50  0000 R CNN
F 2 "IML parts:MPXV7007" H 4350 3550 50  0001 C CNN
F 3 "http://www.nxp.com/files/sensors/doc/data_sheet/MPXA6115A.pdf" H 4850 4500 50  0001 C CNN
	1    4850 3900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 60657F58
P 7250 3700
F 0 "J2" H 7330 3692 50  0000 L CNN
F 1 "NO FUNCTION" H 7330 3601 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 7250 3700 50  0001 C CNN
F 3 "~" H 7250 3700 50  0001 C CNN
	1    7250 3700
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 606583C1
P 7250 4200
F 0 "J1" H 7330 4192 50  0000 L CNN
F 1 "PIN HEADER" H 7330 4101 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 7250 4200 50  0001 C CNN
F 3 "~" H 7250 4200 50  0001 C CNN
	1    7250 4200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x08 J4
U 1 1 60658885
P 7250 2200
F 0 "J4" H 7330 2192 50  0000 L CNN
F 1 "ARDUINO POWER" H 7330 2101 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 7250 2200 50  0001 C CNN
F 3 "~" H 7250 2200 50  0001 C CNN
	1    7250 2200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J3
U 1 1 606592ED
P 7250 3000
F 0 "J3" H 7330 2992 50  0000 L CNN
F 1 "ARDUINO ANALOG" H 7330 2901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 7250 3000 50  0001 C CNN
F 3 "~" H 7250 3000 50  0001 C CNN
	1    7250 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 60659A45
P 6350 3350
F 0 "C1" H 6465 3396 50  0000 L CNN
F 1 "47uF 50V" H 6465 3305 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D8.0mm_W2.5mm_P5.00mm" H 6388 3200 50  0001 C CNN
F 3 "~" H 6350 3350 50  0001 C CNN
	1    6350 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 4400 7000 4400
Wire Wire Line
	5250 4400 5250 3900
Wire Wire Line
	7050 4100 6900 4100
Wire Wire Line
	5350 4100 5350 2650
Wire Wire Line
	5350 2650 5250 2650
Wire Wire Line
	7050 4300 6800 4300
Wire Wire Line
	4850 4300 4850 4200
Wire Wire Line
	4850 2950 4850 3350
Wire Wire Line
	4850 3350 5400 3350
Wire Wire Line
	5400 3350 5400 4300
Connection ~ 5400 4300
Wire Wire Line
	5400 4300 4850 4300
Wire Wire Line
	7050 4200 5450 4200
Wire Wire Line
	5450 4200 5450 3500
Wire Wire Line
	5450 3500 4850 3500
Wire Wire Line
	4850 3500 4850 3600
Wire Wire Line
	5450 3500 5450 2250
Wire Wire Line
	5450 2250 4850 2250
Wire Wire Line
	4850 2250 4850 2350
Connection ~ 5450 3500
Wire Wire Line
	6200 2250 5450 2250
Connection ~ 5450 2250
Connection ~ 6200 4300
Wire Wire Line
	6200 4300 5400 4300
Wire Wire Line
	5600 2650 5350 2650
Connection ~ 5350 2650
Connection ~ 5250 3900
Wire Wire Line
	5600 3400 5500 3400
Wire Wire Line
	5500 3400 5500 3900
Wire Wire Line
	5500 3900 5250 3900
Wire Wire Line
	6800 4300 6800 2500
Wire Wire Line
	6800 2500 7050 2500
Connection ~ 6800 4300
Wire Wire Line
	6800 4300 6200 4300
Wire Wire Line
	6800 2500 6800 2400
Wire Wire Line
	6800 2400 7050 2400
Connection ~ 6800 2500
Wire Wire Line
	6200 2250 6800 2250
Wire Wire Line
	6800 2250 6800 2300
Wire Wire Line
	6800 2300 7050 2300
Connection ~ 6200 2250
Wire Wire Line
	6900 4100 6900 3000
Wire Wire Line
	6900 3000 7050 3000
Connection ~ 6900 4100
Wire Wire Line
	6900 4100 5350 4100
Wire Wire Line
	7000 4400 7000 3100
Wire Wire Line
	7000 3100 7050 3100
Connection ~ 7000 4400
Wire Wire Line
	7000 4400 5250 4400
$Comp
L Device:R_Network_Dividers_x02_SIP RN4
U 1 1 6068061E
P 5900 3400
F 0 "RN4" V 5483 3400 50  0000 C CNN
F 1 "100K DIVIDER" V 5574 3400 50  0000 C CNN
F 2 "Resistor_THT:R_Array_SIP4" V 6175 3400 50  0001 C CNN
F 3 "http://www.vishay.com/docs/31509/csc.pdf" H 5900 3400 50  0001 C CNN
	1    5900 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	5600 2650 5600 3200
Wire Wire Line
	6200 3500 6200 4300
Wire Wire Line
	6200 2250 6200 3200
Wire Wire Line
	6350 3200 6200 3200
Connection ~ 6200 3200
Wire Wire Line
	6350 3500 6200 3500
Connection ~ 6200 3500
Text Notes 10600 7650 0    79   ~ 0
A
Text Notes 8300 7650 0    79   ~ 0
MARCH 2021
$EndSCHEMATC
