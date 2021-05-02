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
L Amplifier_Operational:LM324 U1
U 1 1 608CA6C2
P 2650 3250
F 0 "U1" H 2650 3617 50  0000 C CNN
F 1 "LM324" H 2650 3526 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 2600 3350 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 2700 3450 50  0001 C CNN
	1    2650 3250
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U1
U 2 1 608CA944
P 2650 1000
F 0 "U1" H 2650 1367 50  0000 C CNN
F 1 "LM324" H 2650 1276 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 2600 1100 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 2700 1200 50  0001 C CNN
	2    2650 1000
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U1
U 3 1 608CBB9C
P 2650 2150
F 0 "U1" H 2650 2517 50  0000 C CNN
F 1 "LM324" H 2650 2426 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 2600 2250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 2700 2350 50  0001 C CNN
	3    2650 2150
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U1
U 5 1 608CDD40
P 10100 1050
F 0 "U1" H 10058 1096 50  0000 L CNN
F 1 "LM324" H 10058 1005 50  0000 L CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 10050 1150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 10150 1250 50  0001 C CNN
	5    10100 1050
	1    0    0    -1  
$EndComp
$Comp
L bluepill:BluePill_1 BP1
U 1 1 608CF23F
P 7200 4350
F 0 "BP1" H 7775 4525 50  0000 C CNN
F 1 "BluePill_1" H 7775 4434 50  0000 C CNN
F 2 "bluepill:YAAJ_BluePill" H 8450 2400 50  0001 C CNN
F 3 "" H 8450 2400 50  0001 C CNN
	1    7200 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 608D2CED
P 1750 3150
F 0 "R3" V 1543 3150 50  0000 C CNN
F 1 "2K" V 1634 3150 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 1680 3150 50  0001 C CNN
F 3 "~" H 1750 3150 50  0001 C CNN
	1    1750 3150
	0    1    1    0   
$EndComp
$Comp
L Device:C C8
U 1 1 608D328F
P 9800 1900
F 0 "C8" H 9915 1946 50  0000 L CNN
F 1 "100n" H 9915 1855 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 9838 1750 50  0001 C CNN
F 3 "~" H 9800 1900 50  0001 C CNN
	1    9800 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 608D395C
P 2100 3350
F 0 "C1" H 1900 3400 50  0000 L CNN
F 1 "10u" H 1900 3250 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D4.0mm_P2.00mm" H 2138 3200 50  0001 C CNN
F 3 "~" H 2100 3350 50  0001 C CNN
	1    2100 3350
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BD139 Q1
U 1 1 608D41B7
P 5200 2850
F 0 "Q1" H 5392 2896 50  0000 L CNN
F 1 "BD139" H 5392 2805 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-126-3_Vertical" H 5400 2775 50  0001 L CIN
F 3 "http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/DATASHEET/CD00001225.pdf" H 5200 2850 50  0001 L CNN
	1    5200 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 608DF765
P 2350 3900
F 0 "R6" H 2420 3946 50  0000 L CNN
F 1 "5.1K" H 2420 3855 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 2280 3900 50  0001 C CNN
F 3 "~" H 2350 3900 50  0001 C CNN
	1    2350 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 608E06EF
P 2550 3700
F 0 "R7" V 2343 3700 50  0000 C CNN
F 1 "2.7K" V 2434 3700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 2480 3700 50  0001 C CNN
F 3 "~" H 2550 3700 50  0001 C CNN
	1    2550 3700
	0    1    1    0   
$EndComp
$Comp
L Device:R R12
U 1 1 608E1B88
P 3400 3250
F 0 "R12" V 3193 3250 50  0000 C CNN
F 1 "2K" V 3284 3250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 3330 3250 50  0001 C CNN
F 3 "~" H 3400 3250 50  0001 C CNN
	1    3400 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	2100 3200 2100 3150
Connection ~ 2100 3150
Wire Wire Line
	2100 3150 2350 3150
Wire Wire Line
	2350 3350 2350 3700
Wire Wire Line
	2350 3700 2400 3700
Wire Wire Line
	2350 3750 2350 3700
Connection ~ 2350 3700
Wire Wire Line
	2950 3250 3000 3250
Wire Wire Line
	2700 3700 3000 3700
Wire Wire Line
	3000 3700 3000 3250
Connection ~ 3000 3250
$Comp
L power:GND #PWR04
U 1 1 608E78E8
P 2350 4100
F 0 "#PWR04" H 2350 3850 50  0001 C CNN
F 1 "GND" H 2355 3927 50  0000 C CNN
F 2 "" H 2350 4100 50  0001 C CNN
F 3 "" H 2350 4100 50  0001 C CNN
	1    2350 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 608E82DA
P 2100 3550
F 0 "#PWR03" H 2100 3300 50  0001 C CNN
F 1 "GND" H 2105 3377 50  0000 C CNN
F 2 "" H 2100 3550 50  0001 C CNN
F 3 "" H 2100 3550 50  0001 C CNN
	1    2100 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 3550 2100 3500
Wire Wire Line
	2350 4050 2350 4100
$Comp
L Device:R R9
U 1 1 608EB7F5
P 2650 2500
F 0 "R9" V 2500 2650 50  0000 C CNN
F 1 "20K" V 2534 2500 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 2580 2500 50  0001 C CNN
F 3 "~" H 2650 2500 50  0001 C CNN
	1    2650 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 2150 3150 2150
Wire Wire Line
	3150 2150 3150 2500
Wire Wire Line
	3150 2500 2800 2500
$Comp
L Device:R R8
U 1 1 608ECBD1
P 2600 1350
F 0 "R8" V 2450 1500 50  0000 C CNN
F 1 "10K" V 2484 1350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 2530 1350 50  0001 C CNN
F 3 "~" H 2600 1350 50  0001 C CNN
	1    2600 1350
	0    1    1    0   
$EndComp
Wire Wire Line
	2450 1350 2250 1350
Wire Wire Line
	2250 1350 2250 1100
Wire Wire Line
	2250 1100 2350 1100
Wire Wire Line
	2950 1000 3050 1000
Wire Wire Line
	3050 1000 3050 1350
Wire Wire Line
	3050 1350 2750 1350
$Comp
L Device:R R1
U 1 1 608EDE0F
P 1750 900
F 0 "R1" V 1600 1050 50  0000 C CNN
F 1 "22K" V 1634 900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 1680 900 50  0001 C CNN
F 3 "~" H 1750 900 50  0001 C CNN
	1    1750 900 
	0    1    1    0   
$EndComp
Wire Wire Line
	1900 900  2000 900 
Wire Wire Line
	2000 950  2000 900 
$Comp
L power:GND #PWR01
U 1 1 608EF484
P 2000 1300
F 0 "#PWR01" H 2000 1050 50  0001 C CNN
F 1 "GND" H 2005 1127 50  0000 C CNN
F 2 "" H 2000 1300 50  0001 C CNN
F 3 "" H 2000 1300 50  0001 C CNN
	1    2000 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 1300 2000 1250
$Comp
L Device:R R11
U 1 1 608F0B2B
P 3400 2150
F 0 "R11" V 3500 2150 50  0000 C CNN
F 1 "220" V 3284 2150 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 3330 2150 50  0001 C CNN
F 3 "~" H 3400 2150 50  0001 C CNN
	1    3400 2150
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 608EE34C
P 2000 1100
F 0 "R4" V 1850 1250 50  0000 C CNN
F 1 "2K" V 1884 1100 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 1930 1100 50  0001 C CNN
F 3 "~" H 2000 1100 50  0001 C CNN
	1    2000 1100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R10
U 1 1 608F1936
P 3400 1000
F 0 "R10" V 3500 1000 50  0000 C CNN
F 1 "220" V 3284 1000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 3330 1000 50  0001 C CNN
F 3 "~" H 3400 1000 50  0001 C CNN
	1    3400 1000
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 608F311D
P 1750 2050
F 0 "R2" V 1850 2050 50  0000 C CNN
F 1 "2K" V 1634 2050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 1680 2050 50  0001 C CNN
F 3 "~" H 1750 2050 50  0001 C CNN
	1    1750 2050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3250 1000 3050 1000
Connection ~ 3050 1000
Wire Wire Line
	3250 2150 3150 2150
Connection ~ 3150 2150
Wire Wire Line
	3000 3250 3250 3250
Wire Wire Line
	2350 900  2000 900 
Connection ~ 2000 900 
Wire Wire Line
	1900 3150 2100 3150
Wire Wire Line
	1900 2050 2350 2050
$Comp
L Device:R R15
U 1 1 60901B68
P 5300 1200
F 0 "R15" H 5230 1154 50  0000 R CNN
F 1 "220" H 5230 1245 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 5230 1200 50  0001 C CNN
F 3 "~" H 5300 1200 50  0001 C CNN
	1    5300 1200
	-1   0    0    1   
$EndComp
$Comp
L Device:R R16
U 1 1 6090244F
P 5300 1650
F 0 "R16" H 5230 1604 50  0000 R CNN
F 1 "150" H 5230 1695 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 5230 1650 50  0001 C CNN
F 3 "~" H 5300 1650 50  0001 C CNN
	1    5300 1650
	-1   0    0    1   
$EndComp
$Comp
L Device:R R14
U 1 1 60902789
P 4900 1000
F 0 "R14" V 5107 1000 50  0000 C CNN
F 1 "2K" V 5016 1000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 4830 1000 50  0001 C CNN
F 3 "~" H 4900 1000 50  0001 C CNN
	1    4900 1000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5300 1350 5300 1500
Wire Wire Line
	5050 1000 5100 1000
Wire Wire Line
	5300 1000 5300 1050
$Comp
L power:GND #PWR08
U 1 1 609047F2
P 5300 1900
F 0 "#PWR08" H 5300 1650 50  0001 C CNN
F 1 "GND" H 5305 1727 50  0000 C CNN
F 2 "" H 5300 1900 50  0001 C CNN
F 3 "" H 5300 1900 50  0001 C CNN
	1    5300 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 1800 5300 1900
$Comp
L Device:CP C4
U 1 1 60906965
P 5100 1250
F 0 "C4" H 4900 1300 50  0000 L CNN
F 1 "10u" H 4900 1150 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D4.0mm_P2.00mm" H 5138 1100 50  0001 C CNN
F 3 "~" H 5100 1250 50  0001 C CNN
	1    5100 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 6090696C
P 5100 1450
F 0 "#PWR07" H 5100 1200 50  0001 C CNN
F 1 "GND" H 5105 1277 50  0000 C CNN
F 2 "" H 5100 1450 50  0001 C CNN
F 3 "" H 5100 1450 50  0001 C CNN
	1    5100 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 1450 5100 1400
Wire Wire Line
	5100 1000 5100 1100
Connection ~ 5100 1000
Wire Wire Line
	5100 1000 5300 1000
$Comp
L power:GND #PWR09
U 1 1 6090886B
P 5300 3150
F 0 "#PWR09" H 5300 2900 50  0001 C CNN
F 1 "GND" H 5305 2977 50  0000 C CNN
F 2 "" H 5300 3150 50  0001 C CNN
F 3 "" H 5300 3150 50  0001 C CNN
	1    5300 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 3150 5300 3050
$Comp
L Device:R R13
U 1 1 6090978E
P 4700 2850
F 0 "R13" V 4907 2850 50  0000 C CNN
F 1 "1K" V 4816 2850 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 4630 2850 50  0001 C CNN
F 3 "~" H 4700 2850 50  0001 C CNN
	1    4700 2850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4850 2850 5000 2850
$Comp
L Connector_Generic:Conn_01x03 J8
U 1 1 6091290B
P 10550 1050
F 0 "J8" H 10630 1092 50  0000 L CNN
F 1 "12V_Simetrico" H 10630 1001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 10550 1050 50  0001 C CNN
F 3 "~" H 10550 1050 50  0001 C CNN
	1    10550 1050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 60913376
P 5500 2400
F 0 "J4" H 5580 2392 50  0000 L CNN
F 1 "FAN" H 5580 2301 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5500 2400 50  0001 C CNN
F 3 "~" H 5500 2400 50  0001 C CNN
	1    5500 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10350 750  10000 750 
Wire Wire Line
	10350 950  10350 750 
$Comp
L power:GND #PWR018
U 1 1 6091BD02
P 9750 1050
F 0 "#PWR018" H 9750 800 50  0001 C CNN
F 1 "GND" H 9800 1100 50  0000 C CNN
F 2 "" H 9750 1050 50  0001 C CNN
F 3 "" H 9750 1050 50  0001 C CNN
	1    9750 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 1050 10350 1050
Text Label 10100 750  0    50   ~ 0
+12V
Text Label 10000 1400 0    50   ~ 0
-12V
$Comp
L Connector_Generic:Conn_01x01 J1
U 1 1 6091E778
P 1150 900
F 0 "J1" H 1068 675 50  0000 C CNN
F 1 "Vout" H 1068 766 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 1150 900 50  0001 C CNN
F 3 "~" H 1150 900 50  0001 C CNN
	1    1150 900 
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J2
U 1 1 6091F1AB
P 1150 2050
F 0 "J2" H 1068 1825 50  0000 C CNN
F 1 "Rshunt" H 1068 1916 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 1150 2050 50  0001 C CNN
F 3 "~" H 1150 2050 50  0001 C CNN
	1    1150 2050
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J3
U 1 1 609229A5
P 3850 3250
F 0 "J3" H 3930 3292 50  0000 L CNN
F 1 "Pin_2_TL494" H 3930 3201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3850 3250 50  0001 C CNN
F 3 "~" H 3850 3250 50  0001 C CNN
	1    3850 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 2050 1350 2050
Wire Wire Line
	1350 900  1600 900 
Wire Wire Line
	3650 3250 3550 3250
Text GLabel 1200 3150 0    50   Input ~ 0
A6
Wire Wire Line
	1200 3150 1600 3150
Text GLabel 3650 1000 2    50   Input ~ 0
A0
Wire Wire Line
	3550 1000 3650 1000
Text GLabel 3650 2150 2    50   Input ~ 0
A1
Wire Wire Line
	3650 2150 3550 2150
$Comp
L Device:R R5
U 1 1 6093AE21
P 2000 2450
F 0 "R5" H 2070 2496 50  0000 L CNN
F 1 "10K" H 2070 2405 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 1930 2450 50  0001 C CNN
F 3 "~" H 2000 2450 50  0001 C CNN
	1    2000 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 2300 2000 2250
$Comp
L power:GND #PWR02
U 1 1 6093CD04
P 2000 2650
F 0 "#PWR02" H 2000 2400 50  0001 C CNN
F 1 "GND" H 2005 2477 50  0000 C CNN
F 2 "" H 2000 2650 50  0001 C CNN
F 3 "" H 2000 2650 50  0001 C CNN
	1    2000 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 2600 2000 2650
$Comp
L Device:CP C2
U 1 1 6093EE46
P 3050 1500
F 0 "C2" H 2850 1550 50  0000 L CNN
F 1 "10u" H 2850 1400 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D4.0mm_P2.00mm" H 3088 1350 50  0001 C CNN
F 3 "~" H 3050 1500 50  0001 C CNN
	1    3050 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 6093EE4C
P 3050 1700
F 0 "#PWR05" H 3050 1450 50  0001 C CNN
F 1 "GND" H 3055 1527 50  0000 C CNN
F 2 "" H 3050 1700 50  0001 C CNN
F 3 "" H 3050 1700 50  0001 C CNN
	1    3050 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 1700 3050 1650
$Comp
L Device:CP C3
U 1 1 60940E36
P 3150 2650
F 0 "C3" H 2950 2700 50  0000 L CNN
F 1 "10u" H 2950 2550 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D4.0mm_P2.00mm" H 3188 2500 50  0001 C CNN
F 3 "~" H 3150 2650 50  0001 C CNN
	1    3150 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 60940E3C
P 3150 2850
F 0 "#PWR06" H 3150 2600 50  0001 C CNN
F 1 "GND" H 3155 2677 50  0000 C CNN
F 2 "" H 3150 2850 50  0001 C CNN
F 3 "" H 3150 2850 50  0001 C CNN
	1    3150 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 2850 3150 2800
Connection ~ 3150 2500
Connection ~ 3050 1350
Text GLabel 4600 1000 0    50   Input ~ 0
A7
Wire Wire Line
	4600 1000 4750 1000
$Comp
L Connector_Generic:Conn_01x01 J5
U 1 1 6095612C
P 5550 1000
F 0 "J5" H 5630 1042 50  0000 L CNN
F 1 "Pin_15_TL494" H 5630 951 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 5550 1000 50  0001 C CNN
F 3 "~" H 5550 1000 50  0001 C CNN
	1    5550 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 1000 5300 1000
Connection ~ 5300 1000
Wire Wire Line
	5300 2500 5300 2650
Text GLabel 5150 2400 0    50   Input ~ 0
+12V
Text GLabel 9250 750  0    50   Input ~ 0
+12V
Connection ~ 10000 750 
Text GLabel 4450 2850 0    50   Input ~ 0
A10
Wire Wire Line
	4450 2850 4550 2850
$Comp
L Connector_Generic:Conn_01x02 J9
U 1 1 60970D78
P 10550 1700
F 0 "J9" H 10630 1692 50  0000 L CNN
F 1 "5V" H 10630 1601 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 10550 1700 50  0001 C CNN
F 3 "~" H 10550 1700 50  0001 C CNN
	1    10550 1700
	1    0    0    -1  
$EndComp
Text GLabel 9500 1700 0    50   Input ~ 0
+5V
$Comp
L power:GND #PWR019
U 1 1 6097156B
P 10150 1850
F 0 "#PWR019" H 10150 1600 50  0001 C CNN
F 1 "GND" H 10155 1677 50  0000 C CNN
F 2 "" H 10150 1850 50  0001 C CNN
F 3 "" H 10150 1850 50  0001 C CNN
	1    10150 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 1850 10150 1800
Wire Wire Line
	10150 1800 10350 1800
$Comp
L Device:CP C5
U 1 1 609757B4
P 9400 950
F 0 "C5" H 9200 1000 50  0000 L CNN
F 1 "100u" H 9200 850 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P2.50mm" H 9438 800 50  0001 C CNN
F 3 "~" H 9400 950 50  0001 C CNN
	1    9400 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 800  9400 750 
$Comp
L power:GND #PWR016
U 1 1 609757BB
P 9300 1400
F 0 "#PWR016" H 9300 1150 50  0001 C CNN
F 1 "GND" V 9305 1272 50  0000 R CNN
F 2 "" H 9300 1400 50  0001 C CNN
F 3 "" H 9300 1400 50  0001 C CNN
	1    9300 1400
	0    1    1    0   
$EndComp
$Comp
L Device:CP C6
U 1 1 60976F85
P 9600 1900
F 0 "C6" H 9400 1950 50  0000 L CNN
F 1 "100u" H 9400 1800 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P2.50mm" H 9638 1750 50  0001 C CNN
F 3 "~" H 9600 1900 50  0001 C CNN
	1    9600 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 1750 9600 1700
$Comp
L power:GND #PWR017
U 1 1 60976F8C
P 9600 2100
F 0 "#PWR017" H 9600 1850 50  0001 C CNN
F 1 "GND" H 9605 1927 50  0000 C CNN
F 2 "" H 9600 2100 50  0001 C CNN
F 3 "" H 9600 2100 50  0001 C CNN
	1    9600 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 2100 9600 2050
Wire Wire Line
	9250 750  9400 750 
Connection ~ 9400 750 
Wire Wire Line
	9800 2050 9600 2050
Connection ~ 9600 2050
Wire Wire Line
	9500 1700 9600 1700
Wire Wire Line
	9600 1700 9800 1700
Connection ~ 9600 1700
Wire Wire Line
	9800 1750 9800 1700
Connection ~ 9800 1700
Wire Wire Line
	9800 1700 10350 1700
Text GLabel 6400 1100 0    50   Input ~ 0
B4
Text GLabel 6250 2550 0    50   Input ~ 0
B3
$Comp
L Connector_Generic:Conn_01x02 J7
U 1 1 6099096A
P 7050 1100
F 0 "J7" H 7130 1092 50  0000 L CNN
F 1 "SWITCH_STBY" H 7130 1001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 7050 1100 50  0001 C CNN
F 3 "~" H 7050 1100 50  0001 C CNN
	1    7050 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 1100 6800 1100
$Comp
L power:GND #PWR011
U 1 1 6099308E
P 6750 1200
F 0 "#PWR011" H 6750 950 50  0001 C CNN
F 1 "GND" H 6755 1027 50  0000 C CNN
F 2 "" H 6750 1200 50  0001 C CNN
F 3 "" H 6750 1200 50  0001 C CNN
	1    6750 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 1200 6850 1200
$Comp
L Connector_Generic:Conn_01x01 J6
U 1 1 609953D3
P 7050 850
F 0 "J6" H 7130 892 50  0000 L CNN
F 1 "PS_ON" H 7130 801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 7050 850 50  0001 C CNN
F 3 "~" H 7050 850 50  0001 C CNN
	1    7050 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 850  6800 850 
Wire Wire Line
	6800 850  6800 1100
$Comp
L Device:R R18
U 1 1 60997A7D
P 6600 1100
F 0 "R18" V 6807 1100 50  0000 C CNN
F 1 "2K" V 6716 1100 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 6530 1100 50  0001 C CNN
F 3 "~" H 6600 1100 50  0001 C CNN
	1    6600 1100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6400 1100 6450 1100
Wire Wire Line
	6750 1100 6800 1100
Connection ~ 6800 1100
$Comp
L Device:LED D1
U 1 1 609A06CC
P 6800 2750
F 0 "D1" V 6839 2632 50  0000 R CNN
F 1 "LED" V 6748 2632 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 6800 2750 50  0001 C CNN
F 3 "~" H 6800 2750 50  0001 C CNN
	1    6800 2750
	0    -1   -1   0   
$EndComp
$Comp
L Device:Rotary_Encoder_Switch SW1
U 1 1 609A117E
P 7700 2400
F 0 "SW1" H 7700 2767 50  0000 C CNN
F 1 "Rotary_Encoder_Switch" H 7700 2676 50  0000 C CNN
F 2 "Rotary_Encoder:RotaryEncoder_Alps_EC11E-Switch_Vertical_H20mm_CircularMountingHoles" H 7550 2560 50  0001 C CNN
F 3 "~" H 7700 2660 50  0001 C CNN
	1    7700 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R19
U 1 1 609A4136
P 6550 2550
F 0 "R19" V 6350 2650 50  0000 C CNN
F 1 "470" V 6450 2600 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 6480 2550 50  0001 C CNN
F 3 "~" H 6550 2550 50  0001 C CNN
	1    6550 2550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6700 2550 6800 2550
Wire Wire Line
	6800 2550 6800 2600
Wire Wire Line
	6800 2950 6800 2900
Wire Wire Line
	8100 2500 8000 2500
$Comp
L Device:R R20
U 1 1 609B0963
P 7100 2150
F 0 "R20" H 7030 2104 50  0000 R CNN
F 1 "10K" H 7030 2195 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 7030 2150 50  0001 C CNN
F 3 "~" H 7100 2150 50  0001 C CNN
	1    7100 2150
	-1   0    0    1   
$EndComp
Wire Wire Line
	7100 2300 7250 2300
$Comp
L Device:R R17
U 1 1 609B9D6F
P 7000 2350
F 0 "R17" H 6800 2400 50  0000 L CNN
F 1 "10K" H 6800 2300 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 6930 2350 50  0001 C CNN
F 3 "~" H 7000 2350 50  0001 C CNN
	1    7000 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 2500 7350 2500
Text GLabel 5750 1750 0    50   Input ~ 0
A8
Text GLabel 5750 1650 0    50   Input ~ 0
A9
Text GLabel 5750 1850 0    50   Input ~ 0
A15
$Comp
L Andy:SSD1306 Brd1
U 1 1 609D934F
P 8100 3400
F 0 "Brd1" V 8146 3122 50  0000 R CNN
F 1 "SSD1306" V 8055 3122 50  0000 R CNN
F 2 "andy:128x64OLED" H 8100 3650 50  0001 C CNN
F 3 "" H 8100 3650 50  0001 C CNN
	1    8100 3400
	0    -1   -1   0   
$EndComp
Text GLabel 7000 3250 0    50   Input ~ 0
B9
Text GLabel 7000 3350 0    50   Input ~ 0
B8
Text GLabel 7000 3450 0    50   Input ~ 0
+5V
$Comp
L power:GND #PWR014
U 1 1 609E3E9E
P 7000 3550
F 0 "#PWR014" H 7000 3300 50  0001 C CNN
F 1 "GND" H 7005 3377 50  0000 C CNN
F 2 "" H 7000 3550 50  0001 C CNN
F 3 "" H 7000 3550 50  0001 C CNN
	1    7000 3550
	1    0    0    -1  
$EndComp
Text Label 2950 3250 0    50   ~ 0
PIN_1
Text Label 2350 3550 0    50   ~ 0
PIN_2
Text Label 2200 3150 0    50   ~ 0
PIN_3
Text Label 3000 2150 0    50   ~ 0
PIN_8
Text Label 2100 2050 0    50   ~ 0
PIN_10
Text Label 2100 2250 0    50   ~ 0
PIN_9
Text Label 2250 1300 0    50   ~ 0
PIN_6
Text Label 2100 900  0    50   ~ 0
PIN_5
Text Label 3000 1000 0    50   ~ 0
PIN_7
Text GLabel 8750 5300 2    50   Input ~ 0
A6
Wire Wire Line
	8750 5300 8550 5300
Text GLabel 8750 5800 2    50   Input ~ 0
A1
Wire Wire Line
	8550 5800 8750 5800
Text GLabel 8750 5900 2    50   Input ~ 0
A0
Wire Wire Line
	8750 5900 8550 5900
Text GLabel 6850 5000 0    50   Input ~ 0
A10
Wire Wire Line
	6850 5000 7000 5000
Text GLabel 8750 5200 2    50   Input ~ 0
A7
Wire Wire Line
	8750 5200 8550 5200
Text GLabel 6850 5500 0    50   Input ~ 0
B4
Wire Wire Line
	6850 5500 7000 5500
Text GLabel 6850 5400 0    50   Input ~ 0
B3
Wire Wire Line
	6850 5400 7000 5400
Text GLabel 6850 4800 0    50   Input ~ 0
A8
Text GLabel 6850 4900 0    50   Input ~ 0
A9
Text GLabel 6850 5300 0    50   Input ~ 0
A15
Wire Wire Line
	6850 5300 7000 5300
Wire Wire Line
	6850 4900 7000 4900
Wire Wire Line
	6850 4800 7000 4800
Text GLabel 6850 5900 0    50   Input ~ 0
B8
Wire Wire Line
	6850 5900 7000 5900
Text GLabel 6850 6000 0    50   Input ~ 0
B9
Wire Wire Line
	6850 6000 7000 6000
Text GLabel 6850 6100 0    50   Input ~ 0
+5V
Wire Wire Line
	6850 6100 7000 6100
$Comp
L power:GND #PWR010
U 1 1 60A68A47
P 6800 6200
F 0 "#PWR010" H 6800 5950 50  0001 C CNN
F 1 "GND" H 6805 6027 50  0000 C CNN
F 2 "" H 6800 6200 50  0001 C CNN
F 3 "" H 6800 6200 50  0001 C CNN
	1    6800 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 6200 7000 6200
Text GLabel 8750 4600 2    50   Input ~ 0
3V3
Wire Wire Line
	8750 4600 8550 4600
Text GLabel 5750 1950 0    50   Input ~ 0
3V3
Wire Wire Line
	9400 750  10000 750 
$Comp
L Device:D_Schottky D2
U 1 1 60AC6EC5
P 10200 1400
F 0 "D2" H 10200 1183 50  0000 C CNN
F 1 "D_Schottky" H 10200 1274 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 10200 1400 50  0001 C CNN
F 3 "~" H 10200 1400 50  0001 C CNN
	1    10200 1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	10350 1400 10350 1150
Wire Wire Line
	10050 1400 10000 1400
Wire Wire Line
	10000 1400 10000 1350
$Comp
L Device:CP C7
U 1 1 60AD7E8C
P 9700 1400
F 0 "C7" H 9500 1450 50  0000 L CNN
F 1 "100u" H 9500 1300 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P2.50mm" H 9738 1250 50  0001 C CNN
F 3 "~" H 9700 1400 50  0001 C CNN
	1    9700 1400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9400 1400 9300 1400
Wire Wire Line
	9400 1100 9400 1400
Wire Wire Line
	9850 1400 10000 1400
Connection ~ 10000 1400
Wire Wire Line
	9550 1400 9400 1400
Connection ~ 9400 1400
Wire Wire Line
	2250 2500 2250 2250
Wire Wire Line
	2000 2250 2250 2250
Wire Wire Line
	2250 2500 2350 2500
Wire Wire Line
	2350 2250 2350 2500
Connection ~ 2350 2500
Wire Wire Line
	2350 2500 2500 2500
$Comp
L Device:R R23
U 1 1 60C34E3B
P 9950 2800
F 0 "R23" H 9880 2754 50  0000 R CNN
F 1 "2K" H 9880 2845 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 9880 2800 50  0001 C CNN
F 3 "~" H 9950 2800 50  0001 C CNN
	1    9950 2800
	-1   0    0    1   
$EndComp
$Comp
L Device:R R21
U 1 1 60C34E47
P 9550 2600
F 0 "R21" V 9757 2600 50  0000 C CNN
F 1 "10K" V 9666 2600 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 9480 2600 50  0001 C CNN
F 3 "~" H 9550 2600 50  0001 C CNN
	1    9550 2600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9950 2600 9950 2650
$Comp
L power:GND #PWR020
U 1 1 60C34E50
P 9950 3050
F 0 "#PWR020" H 9950 2800 50  0001 C CNN
F 1 "GND" H 9955 2877 50  0000 C CNN
F 2 "" H 9950 3050 50  0001 C CNN
F 3 "" H 9950 3050 50  0001 C CNN
	1    9950 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 2600 9400 2600
$Comp
L Connector_Generic:Conn_01x01 J11
U 1 1 60C34E69
P 10200 2600
F 0 "J11" H 10280 2642 50  0000 L CNN
F 1 "Pin_1_TL494" H 10280 2551 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 10200 2600 50  0001 C CNN
F 3 "~" H 10200 2600 50  0001 C CNN
	1    10200 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 2600 9950 2600
Connection ~ 9950 2600
$Comp
L Device:R R22
U 1 1 60C3ACC5
P 9700 3400
F 0 "R22" V 9907 3400 50  0000 C CNN
F 1 "2K" V 9816 3400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 9630 3400 50  0001 C CNN
F 3 "~" H 9700 3400 50  0001 C CNN
	1    9700 3400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9400 3400 9550 3400
$Comp
L Connector_Generic:Conn_01x01 J10
U 1 1 60C3ACE7
P 10100 3400
F 0 "J10" H 10180 3442 50  0000 L CNN
F 1 "Pin_16_TL494" H 10180 3351 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 10100 3400 50  0001 C CNN
F 3 "~" H 10100 3400 50  0001 C CNN
	1    10100 3400
	1    0    0    -1  
$EndComp
Text Label 1400 2050 0    50   ~ 0
Rshunt
Text Label 9400 3400 0    50   ~ 0
Rshunt
Wire Wire Line
	9850 3400 9900 3400
Wire Wire Line
	9700 2600 9950 2600
Wire Wire Line
	9950 2950 9950 3050
Text Label 1400 900  0    50   ~ 0
Vout
Text Label 9250 2600 0    50   ~ 0
Vout
$Comp
L Connector_Generic:Conn_01x05 J12
U 1 1 60CD1CA5
P 6300 1850
F 0 "J12" H 5950 2150 50  0000 L CNN
F 1 "Conn_01x05" H 6150 1550 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 6300 1850 50  0001 C CNN
F 3 "~" H 6300 1850 50  0001 C CNN
	1    6300 1850
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J13
U 1 1 60CD35D9
P 6450 1850
F 0 "J13" H 6550 2250 50  0000 C CNN
F 1 "Conn_01x05" H 6550 2150 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 6450 1850 50  0001 C CNN
F 3 "~" H 6450 1850 50  0001 C CNN
	1    6450 1850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7100 2000 7000 2000
Wire Wire Line
	7000 2000 7000 2200
Wire Wire Line
	6650 1950 7000 1950
Wire Wire Line
	7000 1950 7000 2000
Connection ~ 7000 2000
$Comp
L power:GND #PWR013
U 1 1 60D05CCC
P 6000 2050
F 0 "#PWR013" H 6000 1800 50  0001 C CNN
F 1 "GND" H 6005 1877 50  0000 C CNN
F 2 "" H 6000 2050 50  0001 C CNN
F 3 "" H 6000 2050 50  0001 C CNN
	1    6000 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 2050 6100 2050
Wire Wire Line
	6650 1650 7250 1650
Wire Wire Line
	7250 1650 7250 2300
Connection ~ 7250 2300
Wire Wire Line
	7250 2300 7400 2300
Wire Wire Line
	6650 1750 7350 1750
Wire Wire Line
	7350 1750 7350 2500
Connection ~ 7350 2500
Wire Wire Line
	7350 2500 7400 2500
Wire Wire Line
	5150 2400 5300 2400
Wire Wire Line
	6650 1850 8050 1850
Wire Wire Line
	8050 1850 8050 2300
Wire Wire Line
	8050 2300 8000 2300
$Comp
L Connector_Generic:Conn_01x01 J17
U 1 1 60D5D4CD
P 6600 2400
F 0 "J17" H 6300 2450 50  0000 L CNN
F 1 "LED_connector" H 6300 2550 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6600 2400 50  0001 C CNN
F 3 "~" H 6600 2400 50  0001 C CNN
	1    6600 2400
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J16
U 1 1 60D5F618
P 6050 2400
F 0 "J16" H 6200 2400 50  0000 C CNN
F 1 "LED_connector" H 6100 2650 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6050 2400 50  0001 C CNN
F 3 "~" H 6050 2400 50  0001 C CNN
	1    6050 2400
	-1   0    0    1   
$EndComp
Wire Wire Line
	6250 2400 6250 2550
Wire Wire Line
	6400 2400 6400 2550
$Comp
L Connector_Generic:Conn_01x04 J14
U 1 1 60D77E32
P 7300 3350
F 0 "J14" H 7200 2900 50  0000 L CNN
F 1 "Conn_01x04" H 7200 3000 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 7300 3350 50  0001 C CNN
F 3 "~" H 7300 3350 50  0001 C CNN
	1    7300 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J15
U 1 1 60D78516
P 7450 3350
F 0 "J15" H 7368 3667 50  0000 C CNN
F 1 "Conn_01x04" H 7368 3576 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 7450 3350 50  0001 C CNN
F 3 "~" H 7450 3350 50  0001 C CNN
	1    7450 3350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7100 3250 7000 3250
Wire Wire Line
	7000 3350 7100 3350
Wire Wire Line
	7100 3450 7000 3450
Wire Wire Line
	7000 3550 7100 3550
Wire Wire Line
	7650 3550 7750 3550
Wire Wire Line
	7750 3450 7650 3450
Wire Wire Line
	7650 3350 7750 3350
Wire Wire Line
	7750 3250 7650 3250
Wire Wire Line
	7100 2400 7100 2550
Wire Wire Line
	7100 2550 6850 2550
Wire Wire Line
	6850 2550 6850 2050
Wire Wire Line
	7100 2400 7400 2400
Wire Wire Line
	6650 2050 6850 2050
Wire Wire Line
	8100 2650 7100 2650
Wire Wire Line
	7100 2650 7100 2550
Wire Wire Line
	8100 2500 8100 2650
Connection ~ 7100 2550
Wire Wire Line
	6800 2950 7100 2950
Wire Wire Line
	7100 2950 7100 2650
Connection ~ 7100 2650
$Comp
L Device:C C9
U 1 1 60E535B9
P 5900 2000
F 0 "C9" H 5700 1950 50  0000 L CNN
F 1 "100n" H 5650 1850 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 5938 1850 50  0001 C CNN
F 3 "~" H 5900 2000 50  0001 C CNN
	1    5900 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 1950 6100 1950
Connection ~ 6100 1950
Wire Wire Line
	6100 1950 6150 1950
Wire Wire Line
	5750 1850 5900 1850
Connection ~ 5900 1850
Wire Wire Line
	5900 1850 6100 1850
Wire Wire Line
	5750 1750 6100 1750
Wire Wire Line
	5750 1650 6100 1650
$Comp
L power:GND #PWR012
U 1 1 60E7C880
P 5900 2200
F 0 "#PWR012" H 5900 1950 50  0001 C CNN
F 1 "GND" H 5800 2100 50  0000 C CNN
F 2 "" H 5900 2200 50  0001 C CNN
F 3 "" H 5900 2200 50  0001 C CNN
	1    5900 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2200 5900 2150
$Comp
L power:GND #PWR0101
U 1 1 60E8C118
P 8650 4450
F 0 "#PWR0101" H 8650 4200 50  0001 C CNN
F 1 "GND" V 8655 4322 50  0000 R CNN
F 2 "" H 8650 4450 50  0001 C CNN
F 3 "" H 8650 4450 50  0001 C CNN
	1    8650 4450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8650 4450 8550 4450
Wire Wire Line
	8550 4450 8550 4400
Wire Wire Line
	8550 4500 8550 4450
Connection ~ 8550 4450
Text Label 7200 2650 0    50   ~ 0
GND_DISPLAY
$EndSCHEMATC