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
Text Notes 8150 7650 0    59   ~ 12
2023-04-15
Text Notes 10600 7650 0    59   ~ 12
C
Text Notes 7350 7500 0    59   ~ 12
Power Assist System - Relay PCB
$Comp
L power:GNDREF #PWR0112
U 1 1 5EDC5EFA
P 4950 5850
F 0 "#PWR0112" H 4950 5600 50  0001 C CNN
F 1 "GNDREF" H 4955 5677 30  0000 C CNN
F 2 "" H 4950 5850 50  0001 C CNN
F 3 "" H 4950 5850 50  0001 C CNN
	1    4950 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:D_TVS_ALT D2
U 1 1 5EDF865D
P 3750 5250
F 0 "D2" V 3704 5329 50  0000 L CNN
F 1 "SA15CA" V 3795 5329 50  0000 L CNN
F 2 "Diode_THT:D_T-1_P12.70mm_Horizontal" H 3750 5250 50  0001 C CNN
F 3 "~" H 3750 5250 50  0001 C CNN
	1    3750 5250
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C1
U 1 1 5EDFA22E
P 3200 5250
F 0 "C1" H 3315 5296 50  0000 L CNN
F 1 ".1uF POLY" H 3315 5205 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D8.0mm_W2.5mm_P5.00mm" H 3238 5100 50  0001 C CNN
F 3 "~" H 3200 5250 50  0001 C CNN
	1    3200 5250
	-1   0    0    1   
$EndComp
Wire Wire Line
	3200 5400 3200 5700
Wire Wire Line
	3750 5400 3750 5700
Wire Wire Line
	3750 5700 3200 5700
Connection ~ 3200 5700
$Comp
L Switch:SW_DPDT_x2 SW1
U 1 1 5ED31F61
P 5100 3050
F 0 "SW1" H 5100 3335 50  0000 C CNN
F 1 "SW_DPDT_x2" H 5100 3244 50  0000 C CNN
F 2 "IML parts:DPDT_SW" H 5100 3050 50  0001 C CNN
F 3 "~" H 5100 3050 50  0001 C CNN
	1    5100 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5ED35DDA
P 8100 1650
F 0 "D1" H 8093 1866 50  0000 C CNN
F 1 "LED" H 8093 1775 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" H 8100 1650 50  0001 C CNN
F 3 "~" H 8100 1650 50  0001 C CNN
	1    8100 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5ED36825
P 8400 1650
F 0 "R1" V 8607 1650 50  0000 C CNN
F 1 "470R" V 8516 1650 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8330 1650 50  0001 C CNN
F 3 "~" H 8400 1650 50  0001 C CNN
	1    8400 1650
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J1
U 1 1 5ED3A48C
P 9400 3950
F 0 "J1" H 9318 3725 50  0000 C CNN
F 1 "+12V" H 9318 3816 50  0000 C CNN
F 2 "IML parts:250spadeconn" H 9400 3950 50  0001 C CNN
F 3 "~" H 9400 3950 50  0001 C CNN
	1    9400 3950
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J2
U 1 1 5ED3AAC2
P 9400 3400
F 0 "J2" H 9318 3175 50  0000 C CNN
F 1 "GND" H 9318 3266 50  0000 C CNN
F 2 "IML parts:250spadeconn" H 9400 3400 50  0001 C CNN
F 3 "~" H 9400 3400 50  0001 C CNN
	1    9400 3400
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J3
U 1 1 5ED3B239
P 9400 2800
F 0 "J3" H 9318 2575 50  0000 C CNN
F 1 "GRN" H 9318 2666 50  0000 C CNN
F 2 "IML parts:250spadeconn" H 9400 2800 50  0001 C CNN
F 3 "~" H 9400 2800 50  0001 C CNN
	1    9400 2800
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J4
U 1 1 5ED3B8E2
P 9400 2250
F 0 "J4" H 9318 2025 50  0000 C CNN
F 1 "WHT" H 9318 2116 50  0000 C CNN
F 2 "IML parts:250spadeconn" H 9400 2250 50  0001 C CNN
F 3 "~" H 9400 2250 50  0001 C CNN
	1    9400 2250
	1    0    0    1   
$EndComp
$Comp
L Device:Polyfuse F1
U 1 1 5ED3FBFB
P 8050 4000
F 0 "F1" V 7825 4000 50  0000 C CNN
F 1 "Polyfuse" V 7916 4000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 8100 3800 50  0001 L CNN
F 3 "~" H 8050 4000 50  0001 C CNN
	1    8050 4000
	0    1    1    0   
$EndComp
$Comp
L power:+12V #PWR0101
U 1 1 5ED4AF6B
P 9750 4200
F 0 "#PWR0101" H 9750 4050 50  0001 C CNN
F 1 "+12V" H 9765 4373 50  0000 C CNN
F 2 "" H 9750 4200 50  0001 C CNN
F 3 "" H 9750 4200 50  0001 C CNN
	1    9750 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0102
U 1 1 5ED4B5C0
P 9750 3550
F 0 "#PWR0102" H 9750 3300 50  0001 C CNN
F 1 "GNDREF" H 9755 3377 50  0000 C CNN
F 2 "" H 9750 3550 50  0001 C CNN
F 3 "" H 9750 3550 50  0001 C CNN
	1    9750 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 4050 7900 4000
Wire Wire Line
	9200 3950 9150 3950
Wire Wire Line
	8200 3950 8200 4000
Wire Wire Line
	9200 3400 9150 3400
Wire Wire Line
	6050 3400 6050 3850
Wire Wire Line
	6050 3850 4650 3850
Wire Wire Line
	4650 3850 4650 3600
Wire Wire Line
	4650 3600 4900 3600
Wire Wire Line
	4650 3600 4650 3050
Wire Wire Line
	4650 3050 4900 3050
Connection ~ 4650 3600
Wire Wire Line
	5300 3500 5500 3500
Wire Wire Line
	5500 3500 5500 4850
Wire Wire Line
	5500 3500 5500 2950
Wire Wire Line
	5500 2950 5300 2950
Connection ~ 5500 3500
Wire Wire Line
	5300 3150 5400 3150
Wire Wire Line
	5400 3150 5400 3700
Wire Wire Line
	5400 3700 5300 3700
Wire Wire Line
	5400 3700 5400 4500
Wire Wire Line
	5400 4500 4750 4500
Connection ~ 5400 3700
Wire Wire Line
	5500 4850 8750 4850
Wire Wire Line
	8750 4850 8750 2800
Wire Wire Line
	8750 2800 9200 2800
Wire Wire Line
	5400 4500 9000 4500
Wire Wire Line
	9000 4500 9000 2250
Wire Wire Line
	9000 2250 9200 2250
Connection ~ 5400 4500
Wire Wire Line
	9750 4200 9150 4200
Wire Wire Line
	9150 4200 9150 3950
Connection ~ 9150 3950
Wire Wire Line
	9150 3950 8400 3950
Wire Wire Line
	9750 3550 9150 3550
Wire Wire Line
	9150 3550 9150 3400
Connection ~ 9150 3400
Wire Wire Line
	9150 3400 6050 3400
$Comp
L Connector_Generic:Conn_01x01 J7
U 1 1 5ED62514
P 2350 5500
F 0 "J7" H 2268 5717 50  0000 C CNN
F 1 "A" H 2268 5626 50  0000 C CNN
F 2 "IML parts:250spadeconn" H 2350 5500 50  0001 C CNN
F 3 "~" H 2350 5500 50  0001 C CNN
	1    2350 5500
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J8
U 1 1 5ED6299A
P 2350 5050
F 0 "J8" H 2268 5267 50  0000 C CNN
F 1 "B" H 2268 5176 50  0000 C CNN
F 2 "IML parts:250spadeconn" H 2350 5050 50  0001 C CNN
F 3 "~" H 2350 5050 50  0001 C CNN
	1    2350 5050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2800 4750 2800 5050
Wire Wire Line
	2800 5050 2550 5050
Wire Wire Line
	3200 5700 2800 5700
Wire Wire Line
	2800 5700 2800 5500
Wire Wire Line
	2800 5500 2550 5500
$Comp
L Switch:SW_DPDT_x2 SW1B1
U 1 1 5ED72C89
P 5100 3600
F 0 "SW1B1" H 5100 3885 50  0000 C CNN
F 1 "SW_DPDT_x2" H 5100 3794 50  0000 C CNN
F 2 "IML parts:DPDT_SW" H 5100 3600 50  0001 C CNN
F 3 "~" H 5100 3600 50  0001 C CNN
	1    5100 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 4000 8200 4100
Wire Wire Line
	8200 4100 7900 4100
Wire Wire Line
	7900 4100 7900 4050
Connection ~ 8200 4000
Connection ~ 7900 4050
Wire Wire Line
	4950 5750 4950 5800
Wire Wire Line
	4950 5800 3750 5800
Wire Wire Line
	3750 5800 3750 5700
Connection ~ 3750 5700
Wire Wire Line
	4950 4750 4950 4700
Wire Wire Line
	4950 4700 3750 4700
Wire Wire Line
	3750 4700 3750 4750
Wire Wire Line
	4850 5750 4850 5850
Wire Wire Line
	5050 4750 5050 4650
Wire Wire Line
	5050 4650 5300 4650
Wire Wire Line
	5150 4750 5150 4600
Wire Wire Line
	4550 4600 4550 5750
Wire Wire Line
	4550 5750 4750 5750
Wire Wire Line
	4750 4750 4750 4500
Wire Wire Line
	5150 5750 5500 5750
Wire Wire Line
	5500 4850 5500 5750
Connection ~ 5500 4850
Wire Wire Line
	5300 4650 5300 5850
Wire Wire Line
	4850 5850 4950 5850
Connection ~ 4950 5850
Wire Wire Line
	4950 5850 5300 5850
Wire Wire Line
	2800 4750 3200 4750
Wire Wire Line
	3200 5100 3200 4750
Connection ~ 3200 4750
Wire Wire Line
	3200 4750 3750 4750
Wire Wire Line
	3750 5100 3750 4750
Connection ~ 3750 4750
$Comp
L power:GNDREF #PWR0103
U 1 1 607FF7CE
P 4950 2500
F 0 "#PWR0103" H 4950 2250 50  0001 C CNN
F 1 "GNDREF" H 4955 2327 30  0000 C CNN
F 2 "" H 4950 2500 50  0001 C CNN
F 3 "" H 4950 2500 50  0001 C CNN
	1    4950 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:D_TVS_ALT D3
U 1 1 607FF9A8
P 3750 1900
F 0 "D3" V 3704 1979 50  0000 L CNN
F 1 "SA15CA" V 3795 1979 50  0000 L CNN
F 2 "Diode_THT:D_T-1_P12.70mm_Horizontal" H 3750 1900 50  0001 C CNN
F 3 "~" H 3750 1900 50  0001 C CNN
	1    3750 1900
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C2
U 1 1 607FF9B2
P 3200 1900
F 0 "C2" H 3315 1946 50  0000 L CNN
F 1 ".1uF POLY" H 3315 1855 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D8.0mm_W2.5mm_P5.00mm" H 3238 1750 50  0001 C CNN
F 3 "~" H 3200 1900 50  0001 C CNN
	1    3200 1900
	-1   0    0    1   
$EndComp
Wire Wire Line
	3200 2050 3200 2350
Wire Wire Line
	3750 2050 3750 2350
Wire Wire Line
	3750 2350 3200 2350
Connection ~ 3200 2350
$Comp
L Connector_Generic:Conn_01x01 J9
U 1 1 607FF9C6
P 2350 2150
F 0 "J9" H 2268 2367 50  0000 C CNN
F 1 "A" H 2268 2276 50  0000 C CNN
F 2 "IML parts:250spadeconn" H 2350 2150 50  0001 C CNN
F 3 "~" H 2350 2150 50  0001 C CNN
	1    2350 2150
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J10
U 1 1 607FF9D0
P 2350 1700
F 0 "J10" H 2268 1917 50  0000 C CNN
F 1 "B" H 2268 1826 50  0000 C CNN
F 2 "IML parts:250spadeconn" H 2350 1700 50  0001 C CNN
F 3 "~" H 2350 1700 50  0001 C CNN
	1    2350 1700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2800 1400 2800 1700
Wire Wire Line
	2800 1700 2550 1700
Wire Wire Line
	3200 2350 2800 2350
Wire Wire Line
	2800 2350 2800 2150
Wire Wire Line
	2800 2150 2550 2150
$Comp
L V23086C2001A403:V23086C2001A403 K2
U 1 1 607FF9DF
P 4950 1900
F 0 "K2" V 4904 2280 50  0000 L CNN
F 1 "V23086 relay" V 4995 2280 50  0000 L CNN
F 2 "V23086C2001A403:RELAY_V23086C2001A403" H 4900 1500 50  0001 C CNN
F 3 "https://www.fujitsu.com/downloads/MICRO/fcai/relays/ftr-f1.pdf" H 4950 1900 50  0001 C CNN
	1    4950 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 2400 4950 2450
Wire Wire Line
	4950 2450 3750 2450
Wire Wire Line
	3750 2450 3750 2350
Connection ~ 3750 2350
Wire Wire Line
	4950 1400 4950 1350
Wire Wire Line
	4950 1350 3750 1350
Wire Wire Line
	3750 1350 3750 1400
Wire Wire Line
	4850 2400 4850 2500
Wire Wire Line
	5050 1300 5300 1300
Wire Wire Line
	5150 1400 5150 1250
Wire Wire Line
	4550 1250 4550 2400
Wire Wire Line
	4550 2400 4750 2400
Wire Wire Line
	4750 1400 4750 1150
Wire Wire Line
	5300 1300 5300 2500
Wire Wire Line
	4850 2500 4950 2500
Connection ~ 4950 2500
Wire Wire Line
	4950 2500 5300 2500
Wire Wire Line
	2800 1400 3200 1400
Wire Wire Line
	3200 1750 3200 1400
Connection ~ 3200 1400
Wire Wire Line
	3200 1400 3750 1400
Wire Wire Line
	3750 1750 3750 1400
Connection ~ 3750 1400
Wire Wire Line
	5300 2500 6050 2500
Wire Wire Line
	6050 2500 6050 3400
Connection ~ 5300 2500
Connection ~ 6050 3400
Wire Wire Line
	5150 1250 7150 1250
Wire Wire Line
	7150 1250 7150 4050
Connection ~ 5150 1250
$Comp
L Connector_Generic:Conn_01x01 J6
U 1 1 6080B71E
P 9400 1150
F 0 "J6" H 9318 925 50  0000 C CNN
F 1 "ORG" H 9318 1016 50  0000 C CNN
F 2 "IML parts:250spadeconn" H 9400 1150 50  0001 C CNN
F 3 "~" H 9400 1150 50  0001 C CNN
	1    9400 1150
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J5
U 1 1 5ED3BF67
P 9400 1650
F 0 "J5" H 9318 1425 50  0000 C CNN
F 1 "YEL" H 9318 1516 50  0000 C CNN
F 2 "IML parts:250spadeconn" H 9400 1650 50  0001 C CNN
F 3 "~" H 9400 1650 50  0001 C CNN
	1    9400 1650
	1    0    0    1   
$EndComp
Wire Wire Line
	4750 1150 8700 1150
Wire Wire Line
	5150 2400 8850 2400
Wire Wire Line
	8850 2400 8850 1150
Wire Wire Line
	8850 1150 9200 1150
Wire Wire Line
	7150 4050 7900 4050
Wire Wire Line
	8400 3950 8400 2150
Wire Wire Line
	8400 2150 7800 2150
Wire Wire Line
	7800 2150 7800 1650
Wire Wire Line
	7800 1650 7950 1650
Connection ~ 8400 3950
Wire Wire Line
	8400 3950 8200 3950
$Comp
L V23086C2001A403:V23086C2001A403 K1
U 1 1 5ED3CEEA
P 4950 5250
F 0 "K1" V 4904 5630 50  0000 L CNN
F 1 "V23086 relay" V 4995 5630 50  0000 L CNN
F 2 "V23086C2001A403:RELAY_V23086C2001A403" H 4900 4850 50  0001 C CNN
F 3 "https://www.fujitsu.com/downloads/MICRO/fcai/relays/ftr-f1.pdf" H 4950 5250 50  0001 C CNN
	1    4950 5250
	0    1    1    0   
$EndComp
Wire Wire Line
	4550 1250 5150 1250
Wire Wire Line
	6850 1400 6850 2150
Wire Wire Line
	6850 2150 7800 2150
Connection ~ 7800 2150
Wire Wire Line
	5050 1400 5050 1300
Wire Wire Line
	8400 4750 8400 3950
Wire Wire Line
	5050 5750 5050 5800
Wire Wire Line
	5050 5800 8400 5800
Wire Wire Line
	8400 5800 8400 4750
Connection ~ 8400 4750
Wire Wire Line
	8700 1650 9200 1650
Wire Wire Line
	8550 1650 8700 1650
Connection ~ 8700 1650
Wire Wire Line
	8700 1150 8700 1650
Wire Wire Line
	5050 2400 5050 2450
Wire Wire Line
	5050 2450 6850 2450
Wire Wire Line
	6850 2450 6850 2150
Connection ~ 6850 2150
Wire Wire Line
	4850 1400 6850 1400
Wire Wire Line
	4850 4750 8400 4750
Wire Wire Line
	4550 4600 5150 4600
Wire Wire Line
	5150 4600 7150 4600
Wire Wire Line
	7150 4600 7150 4050
Connection ~ 5150 4600
Connection ~ 7150 4050
$EndSCHEMATC
