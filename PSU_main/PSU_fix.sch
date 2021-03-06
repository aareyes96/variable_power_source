EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "PSU mod Main Board"
Date ""
Rev "1.1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
NoConn ~ 7050 1100
NoConn ~ 7050 1200
NoConn ~ 7050 1300
NoConn ~ 7050 1400
NoConn ~ 7050 1800
NoConn ~ 7050 1900
NoConn ~ 7050 2300
NoConn ~ 7050 2400
NoConn ~ 7050 2500
NoConn ~ 8600 3000
NoConn ~ 8600 2900
NoConn ~ 8600 2800
NoConn ~ 8600 2700
NoConn ~ 8600 2400
NoConn ~ 8600 2300
NoConn ~ 8600 1400
Wire Wire Line
	8600 1200 8600 1150
$Comp
L power:GND #PWR012
U 1 1 6094EC37
P 8750 1150
F 0 "#PWR012" H 8750 900 50  0001 C CNN
F 1 "GND" V 8755 1022 50  0001 R CNN
F 2 "" H 8750 1150 50  0001 C CNN
F 3 "" H 8750 1150 50  0001 C CNN
	1    8750 1150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8600 1150 8750 1150
Connection ~ 8600 1150
Wire Wire Line
	8600 1150 8600 1100
Text GLabel 8750 1900 2    50   Output ~ 0
A7
Text GLabel 8750 2000 2    50   Output ~ 0
A6
Wire Wire Line
	8600 2000 8750 2000
Wire Wire Line
	8600 1900 8750 1900
$Comp
L Connector_Generic:Conn_01x10 J10
U 1 1 6094FE77
P 10000 1900
F 0 "J10" H 9918 1175 50  0000 C CNN
F 1 "LCD 16x2" H 9918 1266 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical" H 10000 1900 50  0001 C CNN
F 3 "~" H 10000 1900 50  0001 C CNN
	1    10000 1900
	1    0    0    1   
$EndComp
Wire Wire Line
	8600 2100 9000 2100
Wire Wire Line
	9000 2100 9000 1900
$Comp
L Device:R_POT RV1
U 1 1 6095618B
P 9600 2750
F 0 "RV1" H 9530 2796 50  0000 R CNN
F 1 "10K" H 9530 2705 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Runtron_RM-065_Vertical" H 9600 2750 50  0001 C CNN
F 3 "~" H 9600 2750 50  0001 C CNN
	1    9600 2750
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 609595D5
P 9800 2400
F 0 "#PWR015" H 9800 2150 50  0001 C CNN
F 1 "GND" H 9805 2227 50  0001 C CNN
F 2 "" H 9800 2400 50  0001 C CNN
F 3 "" H 9800 2400 50  0001 C CNN
	1    9800 2400
	1    0    0    -1  
$EndComp
Text GLabel 9200 2650 3    50   Input ~ 0
+5V
Wire Wire Line
	9200 2500 9200 2650
Connection ~ 9600 2500
$Comp
L power:GND #PWR014
U 1 1 6095AD84
P 9600 2950
F 0 "#PWR014" H 9600 2700 50  0001 C CNN
F 1 "GND" H 9605 2777 50  0001 C CNN
F 2 "" H 9600 2950 50  0001 C CNN
F 3 "" H 9600 2950 50  0001 C CNN
	1    9600 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 2950 9600 2900
Wire Wire Line
	9400 2750 9400 2100
Wire Wire Line
	9400 2100 9800 2100
$Comp
L Device:R R19
U 1 1 6095795C
P 9200 2350
F 0 "R19" H 9200 2100 50  0000 R CNN
F 1 "100" H 9200 2200 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9130 2350 50  0001 C CNN
F 3 "~" H 9200 2350 50  0001 C CNN
	1    9200 2350
	-1   0    0    1   
$EndComp
Wire Wire Line
	9200 2200 9200 1400
Text Notes 10250 1450 2    50   ~ 0
LED
Text Notes 10200 1550 2    50   ~ 0
D7
Text Notes 10200 1650 2    50   ~ 0
D6
Text Notes 10200 1750 2    50   ~ 0
D5
Text Notes 10200 1850 2    50   ~ 0
D4
Text Notes 10150 1950 2    50   ~ 0
E
Text Notes 10200 2050 2    50   ~ 0
RS
Text Notes 10200 2150 2    50   ~ 0
VO
Text Notes 10250 2350 2    50   ~ 0
GND
Text Notes 10200 2250 2    50   ~ 0
5V
Text GLabel 8750 2500 2    50   Input ~ 0
A1
Text GLabel 8750 2600 2    50   Input ~ 0
A0
Wire Wire Line
	8600 2500 8750 2500
Wire Wire Line
	8600 2600 8750 2600
Text GLabel 6800 2800 0    50   Input ~ 0
+5V
Wire Wire Line
	6800 2800 7050 2800
Text GLabel 6800 2200 0    50   Output ~ 0
B4
Text GLabel 6800 1700 0    50   Output ~ 0
A10
Wire Wire Line
	6800 2200 7050 2200
Wire Wire Line
	6800 1700 7050 1700
Wire Wire Line
	8600 2200 9100 2200
Wire Wire Line
	9100 2200 9100 2000
Wire Wire Line
	9200 1400 9800 1400
Wire Wire Line
	8600 1500 9800 1500
Wire Wire Line
	8600 1600 9800 1600
Wire Wire Line
	8600 1700 9800 1700
Wire Wire Line
	8600 1800 9800 1800
Wire Wire Line
	9000 1900 9800 1900
Wire Wire Line
	9100 2000 9800 2000
$Comp
L Device:R R5
U 1 1 609838B6
P 2050 1750
F 0 "R5" H 2120 1796 50  0000 L CNN
F 1 "2k" H 2120 1705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1980 1750 50  0001 C CNN
F 3 "~" H 2050 1750 50  0001 C CNN
	1    2050 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 609840DF
P 3150 1650
F 0 "R14" V 3357 1650 50  0000 C CNN
F 1 "220" V 3266 1650 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3080 1650 50  0001 C CNN
F 3 "~" H 3150 1650 50  0001 C CNN
	1    3150 1650
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R9
U 1 1 60984834
P 2600 1950
F 0 "R9" V 2700 1950 50  0000 C CNN
F 1 "10K" V 2500 1950 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 2530 1950 50  0001 C CNN
F 3 "~" H 2600 1950 50  0001 C CNN
	1    2600 1950
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 60984C14
P 1800 1550
F 0 "R3" V 1593 1550 50  0000 C CNN
F 1 "22k" V 1684 1550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 1730 1550 50  0001 C CNN
F 3 "~" H 1800 1550 50  0001 C CNN
	1    1800 1550
	0    1    1    0   
$EndComp
$Comp
L Device:CP C3
U 1 1 609886F8
P 2950 2100
F 0 "C3" H 3068 2146 50  0000 L CNN
F 1 "10u" H 3068 2055 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D4.0mm_P2.00mm" H 2988 1950 50  0001 C CNN
F 3 "~" H 2950 2100 50  0001 C CNN
	1    2950 2100
	1    0    0    -1  
$EndComp
Text GLabel 3400 1650 2    50   Output ~ 0
A0
Wire Wire Line
	3400 1650 3300 1650
Wire Wire Line
	3000 1650 2950 1650
Wire Wire Line
	2950 1950 2950 1650
Connection ~ 2950 1650
Wire Wire Line
	2950 1650 2900 1650
Wire Wire Line
	2750 1950 2950 1950
Connection ~ 2950 1950
Wire Wire Line
	2450 1950 2250 1950
Wire Wire Line
	2250 1950 2250 1750
Wire Wire Line
	2250 1750 2300 1750
Wire Wire Line
	2300 1550 2050 1550
Wire Wire Line
	2050 1600 2050 1550
Connection ~ 2050 1550
Wire Wire Line
	2050 1550 1950 1550
$Comp
L power:GND #PWR02
U 1 1 60999E22
P 2050 2000
F 0 "#PWR02" H 2050 1750 50  0001 C CNN
F 1 "GND" H 2055 1827 50  0001 C CNN
F 2 "" H 2050 2000 50  0001 C CNN
F 3 "" H 2050 2000 50  0001 C CNN
	1    2050 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 6099C6DC
P 2950 2350
F 0 "#PWR06" H 2950 2100 50  0001 C CNN
F 1 "GND" H 2955 2177 50  0001 C CNN
F 2 "" H 2950 2350 50  0001 C CNN
F 3 "" H 2950 2350 50  0001 C CNN
	1    2950 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 2350 2950 2250
$Comp
L Device:R R6
U 1 1 609A2019
P 2250 3350
F 0 "R6" H 2320 3396 50  0000 L CNN
F 1 "10K" H 2320 3305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 2180 3350 50  0001 C CNN
F 3 "~" H 2250 3350 50  0001 C CNN
	1    2250 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 609A201F
P 3150 2900
F 0 "R15" V 3357 2900 50  0000 C CNN
F 1 "220" V 3266 2900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3080 2900 50  0001 C CNN
F 3 "~" H 3150 2900 50  0001 C CNN
	1    3150 2900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R10
U 1 1 609A2025
P 2600 3200
F 0 "R10" V 2700 3200 50  0000 C CNN
F 1 "20K" V 2500 3200 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 2530 3200 50  0001 C CNN
F 3 "~" H 2600 3200 50  0001 C CNN
	1    2600 3200
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 609A202B
P 1950 2800
F 0 "R4" V 1743 2800 50  0000 C CNN
F 1 "2K" V 1834 2800 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 1880 2800 50  0001 C CNN
F 3 "~" H 1950 2800 50  0001 C CNN
	1    1950 2800
	0    1    1    0   
$EndComp
$Comp
L Device:CP C4
U 1 1 609A2031
P 2950 3350
F 0 "C4" H 3068 3396 50  0000 L CNN
F 1 "10u" H 3068 3305 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D4.0mm_P2.00mm" H 2988 3200 50  0001 C CNN
F 3 "~" H 2950 3350 50  0001 C CNN
	1    2950 3350
	1    0    0    -1  
$EndComp
Text GLabel 3400 2900 2    50   Output ~ 0
A1
Wire Wire Line
	3400 2900 3300 2900
Wire Wire Line
	3000 2900 2950 2900
Wire Wire Line
	2950 3200 2950 2900
Connection ~ 2950 2900
Wire Wire Line
	2950 2900 2900 2900
Wire Wire Line
	2750 3200 2950 3200
Connection ~ 2950 3200
Wire Wire Line
	2450 3200 2250 3200
Wire Wire Line
	2250 3200 2250 3000
Wire Wire Line
	2250 3000 2300 3000
$Comp
L Connector_Generic:Conn_01x01 J1
U 1 1 609A2046
P 1100 2800
F 0 "J1" H 1018 3017 50  0000 C CNN
F 1 "RSHUNT" H 1018 2926 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 1100 2800 50  0001 C CNN
F 3 "~" H 1100 2800 50  0001 C CNN
	1    1100 2800
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 609A204D
P 2250 3600
F 0 "#PWR03" H 2250 3350 50  0001 C CNN
F 1 "GND" H 2255 3427 50  0001 C CNN
F 2 "" H 2250 3600 50  0001 C CNN
F 3 "" H 2250 3600 50  0001 C CNN
	1    2250 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 3500 2250 3600
$Comp
L power:GND #PWR07
U 1 1 609A2054
P 2950 3600
F 0 "#PWR07" H 2950 3350 50  0001 C CNN
F 1 "GND" H 2955 3427 50  0001 C CNN
F 2 "" H 2950 3600 50  0001 C CNN
F 3 "" H 2950 3600 50  0001 C CNN
	1    2950 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 3600 2950 3500
Connection ~ 2250 3200
$Comp
L Device:R R1
U 1 1 609AF05F
P 1750 3350
F 0 "R1" V 1543 3350 50  0000 C CNN
F 1 "2K" V 1634 3350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 1680 3350 50  0001 C CNN
F 3 "~" H 1750 3350 50  0001 C CNN
	1    1750 3350
	-1   0    0    1   
$EndComp
Wire Wire Line
	1750 3200 1750 2800
Connection ~ 1750 2800
Wire Wire Line
	1750 2800 1800 2800
Wire Wire Line
	1300 2800 1750 2800
$Comp
L Device:R R7
U 1 1 609C6DB8
P 1600 5850
F 0 "R7" H 1670 5896 50  0000 L CNN
F 1 "5.1K" H 1670 5805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 1530 5850 50  0001 C CNN
F 3 "~" H 1600 5850 50  0001 C CNN
	1    1600 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 609C6DBE
P 2500 5400
F 0 "R16" V 2707 5400 50  0000 C CNN
F 1 "2K" V 2616 5400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2430 5400 50  0001 C CNN
F 3 "~" H 2500 5400 50  0001 C CNN
	1    2500 5400
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R11
U 1 1 609C6DC4
P 1950 5700
F 0 "R11" V 2050 5700 50  0000 C CNN
F 1 "2.7K" V 1850 5700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 1880 5700 50  0001 C CNN
F 3 "~" H 1950 5700 50  0001 C CNN
	1    1950 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 609C6DCA
P 1100 5300
F 0 "R2" V 893 5300 50  0000 C CNN
F 1 "2K" V 984 5300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 1030 5300 50  0001 C CNN
F 3 "~" H 1100 5300 50  0001 C CNN
	1    1100 5300
	0    1    1    0   
$EndComp
Text GLabel 950  5300 0    50   Input ~ 0
A6
Wire Wire Line
	2350 5400 2300 5400
Wire Wire Line
	2300 5700 2300 5400
Connection ~ 2300 5400
Wire Wire Line
	2300 5400 2250 5400
Wire Wire Line
	2100 5700 2300 5700
Wire Wire Line
	1800 5700 1600 5700
Wire Wire Line
	1600 5700 1600 5500
Wire Wire Line
	1600 5500 1650 5500
$Comp
L power:GND #PWR04
U 1 1 609C6DE7
P 1600 6100
F 0 "#PWR04" H 1600 5850 50  0001 C CNN
F 1 "GND" H 1605 5927 50  0001 C CNN
F 2 "" H 1600 6100 50  0001 C CNN
F 3 "" H 1600 6100 50  0001 C CNN
	1    1600 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 6000 1600 6100
$Comp
L power:GND #PWR01
U 1 1 609C6DEE
P 1300 6100
F 0 "#PWR01" H 1300 5850 50  0001 C CNN
F 1 "GND" H 1305 5927 50  0001 C CNN
F 2 "" H 1300 6100 50  0001 C CNN
F 3 "" H 1300 6100 50  0001 C CNN
	1    1300 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 6100 1300 6000
Connection ~ 1600 5700
$Comp
L Device:CP C1
U 1 1 609C6DD0
P 1300 5850
F 0 "C1" H 1050 5900 50  0000 L CNN
F 1 "10u" H 1050 5800 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D4.0mm_P2.00mm" H 1338 5700 50  0001 C CNN
F 3 "~" H 1300 5850 50  0001 C CNN
	1    1300 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 5300 1250 5300
Connection ~ 1300 5300
$Comp
L Device:R R12
U 1 1 609F38A2
P 2650 6800
F 0 "R12" H 2720 6846 50  0000 L CNN
F 1 "220" H 2720 6755 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 2580 6800 50  0001 C CNN
F 3 "~" H 2650 6800 50  0001 C CNN
	1    2650 6800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 609F41D0
P 2650 7200
F 0 "R13" H 2720 7246 50  0000 L CNN
F 1 "150" H 2720 7155 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 2580 7200 50  0001 C CNN
F 3 "~" H 2650 7200 50  0001 C CNN
	1    2650 7200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 609F4635
P 2100 6550
F 0 "R8" V 1893 6550 50  0000 C CNN
F 1 "2K" V 1984 6550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2030 6550 50  0001 C CNN
F 3 "~" H 2100 6550 50  0001 C CNN
	1    2100 6550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 609F5DFB
P 2300 7400
F 0 "#PWR05" H 2300 7150 50  0001 C CNN
F 1 "GND" H 2305 7227 50  0001 C CNN
F 2 "" H 2300 7400 50  0001 C CNN
F 3 "" H 2300 7400 50  0001 C CNN
	1    2300 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 7400 2300 7300
$Comp
L Device:CP C2
U 1 1 609F5E02
P 2300 7150
F 0 "C2" H 2418 7196 50  0000 L CNN
F 1 "10u" H 2418 7105 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D4.0mm_P2.00mm" H 2338 7000 50  0001 C CNN
F 3 "~" H 2300 7150 50  0001 C CNN
	1    2300 7150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 609FF279
P 2650 7400
F 0 "#PWR08" H 2650 7150 50  0001 C CNN
F 1 "GND" H 2655 7227 50  0001 C CNN
F 2 "" H 2650 7400 50  0001 C CNN
F 3 "" H 2650 7400 50  0001 C CNN
	1    2650 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 7050 2650 6950
Wire Wire Line
	2650 7400 2650 7350
Wire Wire Line
	2250 6550 2300 6550
Wire Wire Line
	2650 6650 2650 6550
Wire Wire Line
	2300 7000 2300 6550
Connection ~ 2300 6550
Wire Wire Line
	2300 6550 2650 6550
Text GLabel 1700 6550 0    50   Input ~ 0
A7
Wire Wire Line
	1700 6550 1950 6550
$Comp
L Transistor_BJT:BD139 Q1
U 1 1 60A19711
P 9700 4400
F 0 "Q1" H 9892 4446 50  0000 L CNN
F 1 "BD139" H 9892 4355 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-126-3_Vertical" H 9900 4325 50  0001 L CIN
F 3 "http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/DATASHEET/CD00001225.pdf" H 9700 4400 50  0001 L CNN
	1    9700 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 60A1AAD1
P 9800 4750
F 0 "#PWR011" H 9800 4500 50  0001 C CNN
F 1 "GND" H 9805 4577 50  0001 C CNN
F 2 "" H 9800 4750 50  0001 C CNN
F 3 "" H 9800 4750 50  0001 C CNN
	1    9800 4750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J7
U 1 1 60A1E06A
P 10000 3900
F 0 "J7" H 10080 3892 50  0000 L CNN
F 1 "PSU FAN" H 10080 3801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 10000 3900 50  0001 C CNN
F 3 "~" H 10000 3900 50  0001 C CNN
	1    10000 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 4750 9800 4600
Wire Wire Line
	9800 4200 9800 4000
Text GLabel 9650 3900 0    50   Input ~ 0
+12V
Wire Wire Line
	9650 3900 9800 3900
$Comp
L Device:R R17
U 1 1 60A2A291
P 9200 4400
F 0 "R17" V 8993 4400 50  0000 C CNN
F 1 "1K" V 9084 4400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9130 4400 50  0001 C CNN
F 3 "~" H 9200 4400 50  0001 C CNN
	1    9200 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	9350 4400 9500 4400
Text GLabel 8850 4400 0    50   Input ~ 0
A10
Wire Wire Line
	8850 4400 9050 4400
$Comp
L Connector_Generic:Conn_01x02 J9
U 1 1 60A33B15
P 9950 5600
F 0 "J9" H 10030 5592 50  0000 L CNN
F 1 "SW_STBY" H 10030 5501 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 9950 5600 50  0001 C CNN
F 3 "~" H 9950 5600 50  0001 C CNN
	1    9950 5600
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J8
U 1 1 60A3485F
P 9950 5250
F 0 "J8" H 10030 5292 50  0000 L CNN
F 1 "PS_ON" H 10030 5201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 9950 5250 50  0001 C CNN
F 3 "~" H 9950 5250 50  0001 C CNN
	1    9950 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R18
U 1 1 60A352C1
P 9250 5600
F 0 "R18" V 9043 5600 50  0000 C CNN
F 1 "2K" V 9134 5600 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9180 5600 50  0001 C CNN
F 3 "~" H 9250 5600 50  0001 C CNN
	1    9250 5600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR013
U 1 1 60A35963
P 9750 5850
F 0 "#PWR013" H 9750 5600 50  0001 C CNN
F 1 "GND" H 9755 5677 50  0001 C CNN
F 2 "" H 9750 5850 50  0001 C CNN
F 3 "" H 9750 5850 50  0001 C CNN
	1    9750 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 5700 9750 5850
Wire Wire Line
	9750 5600 9750 5250
Wire Wire Line
	9750 5600 9400 5600
Connection ~ 9750 5600
Text GLabel 8900 5600 0    50   Input ~ 0
B4
Wire Wire Line
	8900 5600 9100 5600
$Comp
L Device:R R21
U 1 1 60A4677E
P 2600 4550
F 0 "R21" H 2670 4596 50  0000 L CNN
F 1 "2K" H 2670 4505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 2530 4550 50  0001 C CNN
F 3 "~" H 2600 4550 50  0001 C CNN
	1    2600 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R20
U 1 1 60A47343
P 2300 4300
F 0 "R20" V 2093 4300 50  0000 C CNN
F 1 "10K" V 2184 4300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 2230 4300 50  0001 C CNN
F 3 "~" H 2300 4300 50  0001 C CNN
	1    2300 4300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR019
U 1 1 60A47B11
P 2600 4850
F 0 "#PWR019" H 2600 4600 50  0001 C CNN
F 1 "GND" H 2605 4677 50  0001 C CNN
F 2 "" H 2600 4850 50  0001 C CNN
F 3 "" H 2600 4850 50  0001 C CNN
	1    2600 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 4850 2600 4700
Wire Wire Line
	2600 4400 2600 4300
Wire Wire Line
	2600 4300 2450 4300
Wire Wire Line
	1950 4300 2050 4300
$Comp
L power:GND #PWR016
U 1 1 60A59FEE
P 1950 4700
F 0 "#PWR016" H 1950 4450 50  0001 C CNN
F 1 "GND" H 1955 4527 50  0001 C CNN
F 2 "" H 1950 4700 50  0001 C CNN
F 3 "" H 1950 4700 50  0001 C CNN
	1    1950 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C7
U 1 1 60A5FC25
P 7500 5550
F 0 "C7" H 7618 5596 50  0000 L CNN
F 1 "4700u" H 7550 5400 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D12.5mm_P5.00mm" H 7538 5400 50  0001 C CNN
F 3 "~" H 7500 5550 50  0001 C CNN
	1    7500 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 60A60D8C
P 7250 5550
F 0 "C5" H 7100 5300 50  0000 L CNN
F 1 "100n" H 7000 5400 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.4mm_W2.1mm_P2.50mm" H 7288 5400 50  0001 C CNN
F 3 "~" H 7250 5550 50  0001 C CNN
	1    7250 5550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 60A6115B
P 7500 5800
F 0 "#PWR018" H 7500 5550 50  0001 C CNN
F 1 "GND" H 7505 5627 50  0001 C CNN
F 2 "" H 7500 5800 50  0001 C CNN
F 3 "" H 7500 5800 50  0001 C CNN
	1    7500 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 60A61BFE
P 7250 5800
F 0 "#PWR017" H 7250 5550 50  0001 C CNN
F 1 "GND" H 7255 5627 50  0001 C CNN
F 2 "" H 7250 5800 50  0001 C CNN
F 3 "" H 7250 5800 50  0001 C CNN
	1    7250 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 60A62418
P 6800 5800
F 0 "#PWR020" H 6800 5550 50  0001 C CNN
F 1 "GND" H 6805 5627 50  0001 C CNN
F 2 "" H 6800 5800 50  0001 C CNN
F 3 "" H 6800 5800 50  0001 C CNN
	1    6800 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 5800 7500 5700
Wire Wire Line
	7850 5350 7500 5350
Wire Wire Line
	7500 5400 7500 5350
Connection ~ 7500 5350
Text GLabel 7850 5350 2    50   Output ~ 0
+5V
$Comp
L Connector_Generic:Conn_01x03 J14
U 1 1 60A7EF59
P 5700 4250
F 0 "J14" H 5800 4100 50  0000 L CNN
F 1 "12_SIMETRICOS" H 5600 4000 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 5700 4250 50  0001 C CNN
F 3 "~" H 5700 4250 50  0001 C CNN
	1    5700 4250
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Schottky D1
U 1 1 60A7FF3A
P 6150 4450
F 0 "D1" H 6300 4500 50  0000 C CNN
F 1 "D_Schottky" H 6150 4600 50  0000 C CNN
F 2 "Diode_THT:D_DO-15_P10.16mm_Horizontal" H 6150 4450 50  0001 C CNN
F 3 "~" H 6150 4450 50  0001 C CNN
	1    6150 4450
	1    0    0    1   
$EndComp
$Comp
L Device:CP C8
U 1 1 60A818CF
P 6600 4800
F 0 "C8" H 6450 4900 50  0000 C CNN
F 1 "100u" H 6400 5000 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D7.5mm_P2.50mm" H 6638 4650 50  0001 C CNN
F 3 "~" H 6600 4800 50  0001 C CNN
	1    6600 4800
	-1   0    0    1   
$EndComp
$Comp
L Device:CP C6
U 1 1 60A8296E
P 6450 5500
F 0 "C6" H 6200 5500 50  0000 L CNN
F 1 "100u" H 6200 5400 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D7.5mm_P2.50mm" H 6488 5350 50  0001 C CNN
F 3 "~" H 6450 5500 50  0001 C CNN
	1    6450 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 60A833AE
P 6050 4250
F 0 "#PWR021" H 6050 4000 50  0001 C CNN
F 1 "GND" H 6200 4250 50  0001 C CNN
F 2 "" H 6050 4250 50  0001 C CNN
F 3 "" H 6050 4250 50  0001 C CNN
	1    6050 4250
	1    0    0    -1  
$EndComp
Text GLabel 5900 4150 2    50   Input ~ 0
+12V
$Comp
L Connector_Generic:Conn_01x08 J6
U 1 1 60AB6777
P 5600 1800
F 0 "J6" H 5518 2317 50  0000 C CNN
F 1 "OLED" H 5518 2226 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 5600 1800 50  0001 C CNN
F 3 "~" H 5600 1800 50  0001 C CNN
	1    5600 1800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6150 2700 7050 2700
Wire Wire Line
	6250 2600 7050 2600
Wire Wire Line
	5800 1900 6350 1900
Wire Wire Line
	6350 1900 6350 2100
Wire Wire Line
	6350 2100 7050 2100
Wire Wire Line
	6350 1500 7050 1500
Wire Wire Line
	5800 1800 6350 1800
Wire Wire Line
	6350 1500 6350 1800
Wire Wire Line
	5800 1700 6500 1700
Wire Wire Line
	6500 1700 6500 2000
Wire Wire Line
	6500 2000 7050 2000
Wire Wire Line
	5800 1600 6700 1600
$Comp
L power:GND #PWR09
U 1 1 60AF5172
P 5950 1500
F 0 "#PWR09" H 5950 1250 50  0001 C CNN
F 1 "GND" V 5955 1372 50  0001 R CNN
F 2 "" H 5950 1500 50  0001 C CNN
F 3 "" H 5950 1500 50  0001 C CNN
	1    5950 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5800 1500 5950 1500
Text Notes 5800 1600 0    50   ~ 0
A 
Text Notes 5800 1700 0    50   ~ 0
SW_ENTER
Text Notes 5800 1800 0    50   ~ 0
B
Text Notes 5800 1900 0    50   ~ 0
LED
Text Notes 5800 2100 0    50   ~ 0
SCL
Text Notes 5800 2000 0    50   ~ 0
SDA
$Comp
L Amplifier_Operational:LM324 U1
U 1 1 6096FCEB
P 2600 1650
F 0 "U1" H 2600 2017 50  0000 C CNN
F 1 "LM324" H 2600 1926 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 2550 1750 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 2650 1850 50  0001 C CNN
	1    2600 1650
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U1
U 2 1 609720AC
P 2600 2900
F 0 "U1" H 2600 3267 50  0000 C CNN
F 1 "LM324" H 2600 3176 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 2550 3000 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 2650 3100 50  0001 C CNN
	2    2600 2900
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U1
U 3 1 60973877
P 1950 5400
F 0 "U1" H 1950 5767 50  0000 C CNN
F 1 "LM324" H 1950 5676 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 1900 5500 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 2000 5600 50  0001 C CNN
	3    1950 5400
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U1
U 5 1 6097856F
P 5600 4900
F 0 "U1" H 5250 4950 50  0000 L CNN
F 1 "LM324" H 5200 4850 50  0000 L CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 5550 5000 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 5650 5100 50  0001 C CNN
	5    5600 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 609D0D06
P 6450 5800
F 0 "#PWR024" H 6450 5550 50  0001 C CNN
F 1 "GND" H 6500 5650 50  0001 R CNN
F 2 "" H 6450 5800 50  0001 C CNN
F 3 "" H 6450 5800 50  0001 C CNN
	1    6450 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 2500 9600 2600
Wire Wire Line
	9400 2750 9450 2750
$Comp
L Device:C C9
U 1 1 609FBC4B
P 6350 1300
F 0 "C9" H 6465 1346 50  0000 L CNN
F 1 "100n" H 6400 1200 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.4mm_W2.1mm_P2.50mm" H 6388 1150 50  0001 C CNN
F 3 "~" H 6350 1300 50  0001 C CNN
	1    6350 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 609FCFF4
P 6700 1300
F 0 "C11" H 6815 1346 50  0000 L CNN
F 1 "100n" H 6750 1200 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.4mm_W2.1mm_P2.50mm" H 6738 1150 50  0001 C CNN
F 3 "~" H 6700 1300 50  0001 C CNN
	1    6700 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 609FE2F1
P 6500 2300
F 0 "C10" H 6350 2400 50  0000 L CNN
F 1 "100n" H 6615 2255 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.4mm_W2.1mm_P2.50mm" H 6538 2150 50  0001 C CNN
F 3 "~" H 6500 2300 50  0001 C CNN
	1    6500 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 2150 6500 2000
Connection ~ 6500 2000
Wire Wire Line
	6350 1500 6350 1450
Connection ~ 6350 1500
Wire Wire Line
	6700 1600 6700 1450
Connection ~ 6700 1600
Wire Wire Line
	6700 1600 7050 1600
Wire Wire Line
	6700 1150 6350 1150
Connection ~ 6350 1150
Wire Wire Line
	6350 1150 6150 1150
$Comp
L power:GND #PWR022
U 1 1 60A26DCE
P 6150 1200
F 0 "#PWR022" H 6150 950 50  0001 C CNN
F 1 "GND" H 6155 1027 50  0001 C CNN
F 2 "" H 6150 1200 50  0001 C CNN
F 3 "" H 6150 1200 50  0001 C CNN
	1    6150 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 1200 6150 1150
$Comp
L power:GND #PWR023
U 1 1 60A2F2AC
P 6500 2750
F 0 "#PWR023" H 6500 2500 50  0001 C CNN
F 1 "GND" H 6500 2600 50  0001 C CNN
F 2 "" H 6500 2750 50  0001 C CNN
F 3 "" H 6500 2750 50  0001 C CNN
	1    6500 2750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J11
U 1 1 60A9335E
P 1750 4300
F 0 "J11" H 1830 4292 50  0000 L CNN
F 1 "Conn_01x02" H 1830 4201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1750 4300 50  0001 C CNN
F 3 "~" H 1750 4300 50  0001 C CNN
	1    1750 4300
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1950 4400 1950 4700
Text Notes 1650 4200 0    50   ~ 0
V out
Text Notes 9850 2800 0    50   ~ 0
BRIGHT LCD
Text GLabel 1950 4000 0    50   Output ~ 0
Vout
Wire Wire Line
	1950 4000 2050 4000
Wire Wire Line
	2050 4000 2050 4300
Connection ~ 2050 4300
Wire Wire Line
	2050 4300 2150 4300
Text GLabel 1500 1550 0    50   Input ~ 0
Vout
Wire Wire Line
	1500 1550 1650 1550
Wire Wire Line
	1300 5300 1300 5700
Wire Wire Line
	6500 2450 6500 2750
Wire Wire Line
	2050 1900 2050 2000
Wire Wire Line
	2100 2800 2300 2800
Wire Wire Line
	1300 5300 1650 5300
Wire Wire Line
	9200 2500 9600 2500
Wire Wire Line
	9600 2200 9800 2200
Wire Wire Line
	9600 2200 9600 2500
Wire Wire Line
	9800 2300 9800 2400
Connection ~ 9200 2500
Wire Wire Line
	6250 2600 6250 2100
Wire Wire Line
	6250 2100 5800 2100
Wire Wire Line
	6150 2700 6150 2000
Wire Wire Line
	6150 2000 5800 2000
$Comp
L YAAJ_STM32:BluePill_1 BP1
U 1 1 60A1D59F
P 7250 1050
F 0 "BP1" H 7825 1225 50  0000 C CNN
F 1 "BluePill_1" H 7825 1134 50  0000 C CNN
F 2 "YAAJ_BluePill:YAAJ_BluePill_1" H 8500 -900 50  0001 C CNN
F 3 "" H 8500 -900 50  0001 C CNN
	1    7250 1050
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:L7805 U2
U 1 1 60D6DB8B
P 6800 5350
F 0 "U2" H 6800 5592 50  0000 C CNN
F 1 "L7805" H 6800 5501 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-252-2" H 6825 5200 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 6800 5300 50  0001 C CNN
	1    6800 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 5650 6800 5800
Text GLabel 6350 5350 0    50   Input ~ 0
+12V
Wire Wire Line
	6350 5350 6450 5350
Wire Wire Line
	6500 5350 6450 5350
Connection ~ 6450 5350
Wire Wire Line
	6450 5650 6450 5800
$Comp
L power:GND #PWR026
U 1 1 60DBF38A
P 6600 4950
F 0 "#PWR026" H 6600 4700 50  0001 C CNN
F 1 "GND" H 6605 4777 50  0001 C CNN
F 2 "" H 6600 4950 50  0001 C CNN
F 3 "" H 6600 4950 50  0001 C CNN
	1    6600 4950
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BD138 Q2
U 1 1 60DC1977
P 7150 4500
F 0 "Q2" H 7341 4546 50  0000 L CNN
F 1 "BD138" H 7341 4455 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-126-3_Vertical" H 7350 4425 50  0001 L CIN
F 3 "http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/DATASHEET/CD00001225.pdf" H 7150 4500 50  0001 L CNN
	1    7150 4500
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 60DC96E4
P 7500 4950
F 0 "#PWR028" H 7500 4700 50  0001 C CNN
F 1 "GND" H 7505 4777 50  0001 C CNN
F 2 "" H 7500 4950 50  0001 C CNN
F 3 "" H 7500 4950 50  0001 C CNN
	1    7500 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R27
U 1 1 60DD25C4
P 7750 4200
F 0 "R27" H 7809 4246 50  0000 L CNN
F 1 "2.2k" H 7809 4155 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 7750 4200 50  0001 C CNN
F 3 "~" H 7750 4200 50  0001 C CNN
	1    7750 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R28
U 1 1 60DDA036
P 7750 4700
F 0 "R28" H 7809 4746 50  0000 L CNN
F 1 "8.2k" H 7809 4655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 7750 4700 50  0001 C CNN
F 3 "~" H 7750 4700 50  0001 C CNN
	1    7750 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R26
U 1 1 60E0B868
P 7500 4700
F 0 "R26" H 7300 4650 50  0000 L CNN
F 1 "2.2k" H 7300 4550 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 7500 4700 50  0001 C CNN
F 3 "~" H 7500 4700 50  0001 C CNN
	1    7500 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R22
U 1 1 60E392B6
P 6750 4050
F 0 "R22" V 6750 3800 50  0000 C CNN
F 1 "4.7K" V 6750 4050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 6680 4050 50  0001 C CNN
F 3 "~" H 6750 4050 50  0001 C CNN
	1    6750 4050
	0    1    1    0   
$EndComp
$Comp
L Device:R R23
U 1 1 60E3A65A
P 6750 4150
F 0 "R23" V 6750 3900 50  0000 C CNN
F 1 "4.7K" V 6750 4150 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 6680 4150 50  0001 C CNN
F 3 "~" H 6750 4150 50  0001 C CNN
	1    6750 4150
	0    1    1    0   
$EndComp
$Comp
L Device:R R24
U 1 1 60E3ADE3
P 6750 4250
F 0 "R24" V 6750 4000 50  0000 C CNN
F 1 "4.7K" V 6750 4250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 6680 4250 50  0001 C CNN
F 3 "~" H 6750 4250 50  0001 C CNN
	1    6750 4250
	0    1    1    0   
$EndComp
$Comp
L Device:R R25
U 1 1 60E3AFC7
P 6750 4350
F 0 "R25" V 6750 4100 50  0000 C CNN
F 1 "4.7K" V 6750 4350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 6680 4350 50  0001 C CNN
F 3 "~" H 6750 4350 50  0001 C CNN
	1    6750 4350
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 4050 7050 4300
Wire Wire Line
	6900 4050 6950 4050
Connection ~ 7050 4050
Wire Wire Line
	6900 4150 6950 4150
Wire Wire Line
	6950 4150 6950 4050
Connection ~ 6950 4050
Wire Wire Line
	6950 4050 7050 4050
Wire Wire Line
	6900 4250 6950 4250
Wire Wire Line
	6950 4250 6950 4150
Connection ~ 6950 4150
Wire Wire Line
	6900 4350 6950 4350
Wire Wire Line
	6950 4350 6950 4250
Connection ~ 6950 4250
Wire Wire Line
	6600 4050 6600 4150
Wire Wire Line
	6600 4150 6600 4250
Connection ~ 6600 4150
Wire Wire Line
	6600 4250 6600 4350
Connection ~ 6600 4250
Wire Wire Line
	6050 4250 5900 4250
Wire Wire Line
	6600 4350 6600 4450
Connection ~ 6600 4350
Wire Wire Line
	5900 4350 6000 4350
Wire Wire Line
	6000 4350 6000 4450
Text GLabel 7850 4050 2    50   Output ~ 0
-12V
Text GLabel 5500 5200 2    50   Input ~ 0
-12V
Text GLabel 5500 4600 2    50   Input ~ 0
+12V
Text Label 3550 5800 0    50   ~ 0
PIN_1_TL494
Text Label 3550 5900 0    50   ~ 0
PIN_2_TL494
Text Label 3500 6000 0    50   ~ 0
PIN_15_TL494
Text Label 3500 5700 0    50   ~ 0
PIN_16_TL494
Wire Notes Line
	5150 3850 5150 6100
Text Notes 5150 3850 0    197  ~ 0
FUENTE
Wire Notes Line
	4550 1150 650  1150
Wire Notes Line
	650  1150 650  7700
Wire Notes Line
	650  7700 4550 7700
Wire Notes Line
	4550 1150 4550 7700
Text Notes 650  1150 0    197  ~ 0
Acondicionador
Wire Wire Line
	7050 4050 7500 4050
$Comp
L power:GND #PWR029
U 1 1 60EBC61B
P 7750 4950
F 0 "#PWR029" H 7750 4700 50  0001 C CNN
F 1 "GND" H 7755 4777 50  0001 C CNN
F 2 "" H 7750 4950 50  0001 C CNN
F 3 "" H 7750 4950 50  0001 C CNN
	1    7750 4950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 60EC4ED8
P 7050 4950
F 0 "#PWR027" H 7050 4700 50  0001 C CNN
F 1 "GND" H 7055 4777 50  0001 C CNN
F 2 "" H 7050 4950 50  0001 C CNN
F 3 "" H 7050 4950 50  0001 C CNN
	1    7050 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 4950 7500 4800
Wire Wire Line
	7750 4950 7750 4800
Wire Wire Line
	7500 4600 7500 4500
Wire Wire Line
	7350 4500 7500 4500
Connection ~ 7500 4500
Wire Wire Line
	7500 4500 7500 4400
Wire Wire Line
	7050 4950 7050 4700
Wire Wire Line
	7600 4300 7750 4300
Wire Wire Line
	7750 4600 7750 4300
Connection ~ 7750 4300
Wire Wire Line
	7500 4200 7500 4050
Connection ~ 7500 4050
Wire Wire Line
	7750 4050 7750 4100
Wire Wire Line
	7500 4050 7750 4050
Wire Wire Line
	7750 4050 7850 4050
Connection ~ 7750 4050
Wire Notes Line
	8200 3850 8200 6100
Wire Notes Line
	5150 6100 8200 6100
Wire Notes Line
	5150 3850 8200 3850
Wire Wire Line
	7100 5350 7250 5350
Wire Wire Line
	7250 5400 7250 5350
Connection ~ 7250 5350
Wire Wire Line
	7250 5350 7500 5350
Wire Wire Line
	7250 5700 7250 5800
Wire Wire Line
	6300 4450 6600 4450
Wire Wire Line
	6600 4450 6600 4650
Connection ~ 6600 4450
Wire Wire Line
	1750 3800 1750 3500
$Comp
L Reference_Voltage:TL431LP U3
U 1 1 60F4759E
P 7500 4300
F 0 "U3" V 7350 4250 50  0000 R CNN
F 1 "TL431LP" V 7450 4250 50  0000 R CNN
F 2 "Package_TO_SOT_THT:TO-92_Wide" H 7500 4150 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tl431.pdf" H 7500 4300 50  0001 C CIN
	1    7500 4300
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 60EE0828
P 4200 5800
F 0 "J2" H 4280 5792 50  0000 L CNN
F 1 "TL494" H 4280 5701 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4200 5800 50  0001 C CNN
F 3 "~" H 4200 5800 50  0001 C CNN
	1    4200 5800
	1    0    0    -1  
$EndComp
Wire Notes Line
	9100 1100 9100 3150
Wire Notes Line
	9100 3150 10400 3150
Wire Notes Line
	10400 3150 10400 1100
Wire Notes Line
	10400 1100 9100 1100
Text Notes 9700 3150 0    50   ~ 0
FRONT PANEL LCD
Wire Notes Line
	5150 1050 5150 2450
Wire Notes Line
	5150 2450 7000 2450
Wire Notes Line
	7000 2450 7000 1050
Wire Notes Line
	7000 1050 5150 1050
Text Notes 5200 2400 0    50   ~ 0
FRONT PANEL OLED
Wire Wire Line
	4000 6000 2800 6000
Wire Wire Line
	2800 6000 2800 6550
Wire Wire Line
	2800 6550 2650 6550
Connection ~ 2650 6550
Wire Wire Line
	2650 5400 2800 5400
Wire Wire Line
	2800 5400 2800 5900
Wire Wire Line
	2800 5900 4000 5900
Wire Wire Line
	4000 5800 3050 5800
Wire Wire Line
	3050 5800 3050 4300
Wire Wire Line
	3050 4300 2600 4300
Connection ~ 2600 4300
Wire Wire Line
	3400 5700 3400 3800
Wire Wire Line
	3400 3800 1750 3800
Wire Notes Line
	4300 6150 4300 6950
Wire Notes Line
	4300 6950 3250 6950
Wire Notes Line
	3250 6950 3250 6150
Wire Notes Line
	3250 6150 4300 6150
Wire Wire Line
	3400 5700 4000 5700
Text Notes 3500 6650 0    50   ~ 0
PIN 1- 1IN +\nPIN 2- 1IN -\nPIN 15- 2IN -\nPIN 16- 2IN +
Text Notes 4000 6900 0    50   ~ 0
TL494
Wire Wire Line
	6900 2900 7050 2900
$Comp
L power:GND #PWR010
U 1 1 6095FFE4
P 6900 3350
F 0 "#PWR010" H 6900 3100 50  0001 C CNN
F 1 "GND" H 6905 3177 50  0001 C CNN
F 2 "" H 6900 3350 50  0001 C CNN
F 3 "" H 6900 3350 50  0001 C CNN
	1    6900 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 3000 6700 3050
$Comp
L Device:CP C12
U 1 1 610919F1
P 6700 3200
F 0 "C12" H 6450 3200 50  0000 L CNN
F 1 "100u" H 6450 3100 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D7.5mm_P2.50mm" H 6738 3050 50  0001 C CNN
F 3 "~" H 6700 3200 50  0001 C CNN
	1    6700 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 610913C3
P 6700 3350
F 0 "#PWR025" H 6700 3100 50  0001 C CNN
F 1 "GND" H 6750 3200 50  0001 R CNN
F 2 "" H 6700 3350 50  0001 C CNN
F 3 "" H 6700 3350 50  0001 C CNN
	1    6700 3350
	1    0    0    -1  
$EndComp
NoConn ~ 8600 1300
Wire Wire Line
	6900 2900 6900 3350
Wire Wire Line
	7050 3000 6700 3000
Connection ~ 6700 3000
Wire Wire Line
	6000 3000 6000 2200
Wire Wire Line
	5800 2200 6000 2200
Wire Wire Line
	6000 3000 6700 3000
$EndSCHEMATC
