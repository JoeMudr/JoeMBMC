EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "JoeM-BMS"
Date "2021-04-18"
Rev "1.0"
Comp "JoeMTec"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR0101
U 1 1 60722527
P 4350 850
F 0 "#PWR0101" H 4350 600 50  0001 C CNN
F 1 "GND" H 4355 677 50  0000 C CNN
F 2 "" H 4350 850 50  0001 C CNN
F 3 "" H 4350 850 50  0001 C CNN
	1    4350 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 850  4350 850 
$Comp
L Interface_CAN_LIN:SN65HVD230 U2
U 1 1 60723409
P 3600 6850
F 0 "U2" H 3800 6500 50  0000 C CNN
F 1 "SN65HVD230" H 3950 7100 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3600 6350 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn65hvd230.pdf" H 3500 7250 50  0001 C CNN
F 4 "C12084" H 3600 6850 50  0001 C CNN "LCSC"
	1    3600 6850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4500 2450 4200 2450
$Comp
L Device:R R1
U 1 1 6073875D
P 4100 7200
F 0 "R1" H 4170 7246 50  0000 L CNN
F 1 "10k" H 4170 7155 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4030 7200 50  0001 C CNN
F 3 "~" H 4100 7200 50  0001 C CNN
F 4 "C17414" H 4100 7200 50  0001 C CNN "LCSC"
	1    4100 7200
	1    0    0    1   
$EndComp
Wire Wire Line
	4100 7050 4000 7050
Wire Wire Line
	6500 3250 6750 3250
$Comp
L Regulator_Switching:R-78E5.0-0.5 U3
U 1 1 6072BF95
P 7950 950
F 0 "U3" H 7950 1192 50  0000 C CNN
F 1 "DCDC 5V" H 7950 1101 50  0000 C CNN
F 2 "Converter_DCDC:Converter_DCDC_muRata_OKI-78SR_Vertical" H 8000 700 50  0001 L CIN
F 3 "https://www.recom-power.com/pdf/Innoline/R-78Exx-0.5.pdf" H 7950 950 50  0001 C CNN
F 4 "C115916" H 7950 950 50  0001 C CNN "LCSC"
	1    7950 950 
	-1   0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0102
U 1 1 6072E6A3
P 8250 950
F 0 "#PWR0102" H 8250 800 50  0001 C CNN
F 1 "+12V" V 8265 1078 50  0000 L CNN
F 2 "" H 8250 950 50  0001 C CNN
F 3 "" H 8250 950 50  0001 C CNN
	1    8250 950 
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_02x05_Top_Bottom J5
U 1 1 60732DDB
P 1500 6900
F 0 "J5" H 1550 7317 50  0000 C CNN
F 1 "IN_COMM" H 1550 7226 50  0000 C CNN
F 2 "Connector_Molex:Molex_Mini-Fit_Jr_5566-10A_2x05_P4.20mm_Vertical" H 1500 6900 50  0001 C CNN
F 3 "~" H 1500 6900 50  0001 C CNN
F 4 "C169060" H 1500 6900 50  0001 C CNN "LCSC"
	1    1500 6900
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 6073DECD
P 4200 2450
F 0 "#PWR0103" H 4200 2200 50  0001 C CNN
F 1 "GND" H 4205 2277 50  0000 C CNN
F 2 "" H 4200 2450 50  0001 C CNN
F 3 "" H 4200 2450 50  0001 C CNN
	1    4200 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 2350 6750 2350
$Comp
L power:GND #PWR0104
U 1 1 6073ECB6
P 6750 2350
F 0 "#PWR0104" H 6750 2100 50  0001 C CNN
F 1 "GND" H 6755 2177 50  0000 C CNN
F 2 "" H 6750 2350 50  0001 C CNN
F 3 "" H 6750 2350 50  0001 C CNN
	1    6750 2350
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 6073F031
P 6750 3250
F 0 "#PWR0105" H 6750 3000 50  0001 C CNN
F 1 "GND" H 6755 3077 50  0000 C CNN
F 2 "" H 6750 3250 50  0001 C CNN
F 3 "" H 6750 3250 50  0001 C CNN
	1    6750 3250
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 60741C63
P 7950 1250
F 0 "#PWR0106" H 7950 1000 50  0001 C CNN
F 1 "GND" H 7955 1077 50  0000 C CNN
F 2 "" H 7950 1250 50  0001 C CNN
F 3 "" H 7950 1250 50  0001 C CNN
	1    7950 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 607441FC
P 1200 7100
F 0 "#PWR0107" H 1200 6850 50  0001 C CNN
F 1 "GND" H 1205 6927 50  0000 C CNN
F 2 "" H 1200 7100 50  0001 C CNN
F 3 "" H 1200 7100 50  0001 C CNN
	1    1200 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 7350 3600 7250
$Comp
L power:GND #PWR0108
U 1 1 60748A38
P 3600 7350
F 0 "#PWR0108" H 3600 7100 50  0001 C CNN
F 1 "GND" H 3605 7177 50  0000 C CNN
F 2 "" H 3600 7350 50  0001 C CNN
F 3 "" H 3600 7350 50  0001 C CNN
	1    3600 7350
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0109
U 1 1 60749C58
P 6500 3350
F 0 "#PWR0109" H 6500 3200 50  0001 C CNN
F 1 "+3.3V" V 6515 3478 50  0000 L CNN
F 2 "" H 6500 3350 50  0001 C CNN
F 3 "" H 6500 3350 50  0001 C CNN
	1    6500 3350
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0110
U 1 1 6074B1A1
P 3600 6250
F 0 "#PWR0110" H 3600 6100 50  0001 C CNN
F 1 "+3.3V" H 3615 6423 50  0000 C CNN
F 2 "" H 3600 6250 50  0001 C CNN
F 3 "" H 3600 6250 50  0001 C CNN
	1    3600 6250
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0112
U 1 1 6075075E
P 7650 950
F 0 "#PWR0112" H 7650 800 50  0001 C CNN
F 1 "+5V" V 7665 1078 50  0000 L CNN
F 2 "" H 7650 950 50  0001 C CNN
F 3 "" H 7650 950 50  0001 C CNN
	1    7650 950 
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0113
U 1 1 60754C66
P 6500 3150
F 0 "#PWR0113" H 6500 3000 50  0001 C CNN
F 1 "+5V" V 6515 3278 50  0000 L CNN
F 2 "" H 6500 3150 50  0001 C CNN
F 3 "" H 6500 3150 50  0001 C CNN
	1    6500 3150
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0114
U 1 1 607563BE
P 1700 7100
F 0 "#PWR0114" H 1700 6950 50  0001 C CNN
F 1 "+5V" V 1715 7228 50  0000 L CNN
F 2 "" H 1700 7100 50  0001 C CNN
F 3 "" H 1700 7100 50  0001 C CNN
	1    1700 7100
	0    1    1    0   
$EndComp
Text GLabel 1700 6700 2    50   Input ~ 0
J5_IN1
Text GLabel 1700 6800 2    50   Input ~ 0
J5_IN2
Text GLabel 1200 6700 0    50   Input ~ 0
J5_IN3
Text GLabel 1200 6800 0    50   Input ~ 0
J5_IN4
Text GLabel 1200 6900 0    50   Input ~ 0
J5_RX2
Text GLabel 4500 1850 0    50   Input ~ 0
J5_RX2
Text GLabel 1700 6900 2    50   Input ~ 0
J5_TX2
Text GLabel 4500 1950 0    50   Input ~ 0
J5_TX2
Text GLabel 4500 3150 0    50   Input ~ 0
Teensy_IN1
Text GLabel 4500 3050 0    50   Input ~ 0
Teensy_IN2
Text GLabel 4500 3350 0    50   Input ~ 0
Teensy_IN4
Text GLabel 4500 3250 0    50   Input ~ 0
Teensy_IN3
$Comp
L power:+5V #PWR0127
U 1 1 6080EC7A
P 10450 4550
F 0 "#PWR0127" H 10450 4400 50  0001 C CNN
F 1 "+5V" H 10465 4723 50  0000 C CNN
F 2 "" H 10450 4550 50  0001 C CNN
F 3 "" H 10450 4550 50  0001 C CNN
	1    10450 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 6082811A
P 10450 4700
F 0 "C3" H 10335 4654 50  0000 R CNN
F 1 "10uF" H 10335 4745 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 10488 4550 50  0001 C CNN
F 3 "~" H 10450 4700 50  0001 C CNN
F 4 "C15850" H 10450 4700 50  0001 C CNN "LCSC"
	1    10450 4700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 60828DA8
P 10450 4850
F 0 "#PWR0128" H 10450 4600 50  0001 C CNN
F 1 "GND" H 10455 4677 50  0000 C CNN
F 2 "" H 10450 4850 50  0001 C CNN
F 3 "" H 10450 4850 50  0001 C CNN
	1    10450 4850
	1    0    0    -1  
$EndComp
Text GLabel 1700 7000 2    50   Input ~ 0
J5_CANH
Text GLabel 1200 7000 0    50   Input ~ 0
J5_CANL
Text GLabel 3100 6950 0    50   Input ~ 0
J5_CANL
Text GLabel 3100 6850 0    50   Input ~ 0
J5_CANH
Text GLabel 4500 1250 0    50   Input ~ 0
Teensy_TX
Text GLabel 4500 1350 0    50   Input ~ 0
Teensy_RX
Text GLabel 4000 6750 2    50   Input ~ 0
Teensy_TX
Text GLabel 4000 6850 2    50   Input ~ 0
Teensy_RX
Text GLabel 6500 3550 2    50   Input ~ 0
Teensy_OUT5
Text GLabel 6500 3450 2    50   Input ~ 0
Teensy_OUT6
Text GLabel 4500 3550 0    50   Input ~ 0
Teensy_OUT4
Text GLabel 4500 3450 0    50   Input ~ 0
Teensy_OUT3
Text GLabel 4500 2050 0    50   Input ~ 0
Teensy_OUT1
Text GLabel 4500 2150 0    50   Input ~ 0
Teensy_OUT2
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J1
U 1 1 6088FEB9
P 1400 4350
F 0 "J1" H 1450 4567 50  0000 C CNN
F 1 "Tesla_BMS" H 1450 4476 50  0000 C CNN
F 2 "Connector_Molex:Molex_Mini-Fit_Jr_5566-04A_2x02_P4.20mm_Vertical" H 1400 4350 50  0001 C CNN
F 3 "~" H 1400 4350 50  0001 C CNN
F 4 "C29973" H 1400 4350 50  0001 C CNN "LCSC"
	1    1400 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 60894063
P 1200 4350
F 0 "#PWR0131" H 1200 4100 50  0001 C CNN
F 1 "GND" H 1205 4177 50  0000 C CNN
F 2 "" H 1200 4350 50  0001 C CNN
F 3 "" H 1200 4350 50  0001 C CNN
	1    1200 4350
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0132
U 1 1 608947A5
P 1700 4350
F 0 "#PWR0132" H 1700 4200 50  0001 C CNN
F 1 "+5V" V 1715 4478 50  0000 L CNN
F 2 "" H 1700 4350 50  0001 C CNN
F 3 "" H 1700 4350 50  0001 C CNN
	1    1700 4350
	0    1    1    0   
$EndComp
Text GLabel 4500 1650 0    50   Input ~ 0
Teensy_RX3
Text GLabel 4500 1750 0    50   Input ~ 0
Teensy_TX3
Text GLabel 1200 4450 0    50   Input ~ 0
Teensy_RX3
Text GLabel 1700 4450 2    50   Input ~ 0
Teensy_TX3
Text GLabel 4500 1450 0    50   Input ~ 0
Teensy_OUT7
Text GLabel 4500 1550 0    50   Input ~ 0
Teensy_OUT8
Text GLabel 2150 2150 2    50   Input ~ 0
Teensy_OUT7
$Comp
L Device:R R13
U 1 1 608A396F
P 2000 2150
F 0 "R13" V 1900 2150 50  0000 C CNN
F 1 "220" V 2000 2150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1930 2150 50  0001 C CNN
F 3 "~" H 2000 2150 50  0001 C CNN
F 4 "C17557" V 2000 2150 50  0001 C CNN "LCSC"
	1    2000 2150
	0    1    1    0   
$EndComp
$Comp
L Device:R R12
U 1 1 608A5061
P 1550 2150
F 0 "R12" V 1450 2150 50  0000 C CNN
F 1 "10k" V 1550 2150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1480 2150 50  0001 C CNN
F 3 "~" H 1550 2150 50  0001 C CNN
F 4 "C17414" V 1550 2150 50  0001 C CNN "LCSC"
	1    1550 2150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0133
U 1 1 608A67E8
P 1400 2150
F 0 "#PWR0133" H 1400 1900 50  0001 C CNN
F 1 "GND" H 1405 1977 50  0000 C CNN
F 2 "" H 1400 2150 50  0001 C CNN
F 3 "" H 1400 2150 50  0001 C CNN
	1    1400 2150
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0134
U 1 1 608DC48B
P 1700 1300
F 0 "#PWR0134" H 1700 1050 50  0001 C CNN
F 1 "GND" H 1705 1127 50  0000 C CNN
F 2 "" H 1700 1300 50  0001 C CNN
F 3 "" H 1700 1300 50  0001 C CNN
	1    1700 1300
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0135
U 1 1 608DCB0C
P 1700 1600
F 0 "#PWR0135" H 1700 1350 50  0001 C CNN
F 1 "GND" H 1705 1427 50  0000 C CNN
F 2 "" H 1700 1600 50  0001 C CNN
F 3 "" H 1700 1600 50  0001 C CNN
	1    1700 1600
	0    1    -1   0   
$EndComp
$Comp
L Transistor-FET:IRF9956 Q3
U 1 1 608DF60B
P 2000 1550
F 0 "Q3" H 2000 1950 50  0000 C CNN
F 1 "BSO615NG" H 2000 1200 50  0000 C CNN
F 2 "JoeMBMS:SO-8_IRF956" H 2000 1450 50  0001 C CNN
F 3 "" H 2000 1450 50  0001 C CNN
F 4 "C693398" H 2000 1550 50  0001 C CNN "LCSC"
	1    2000 1550
	1    0    0    -1  
$EndComp
Text GLabel 2150 1050 2    50   Input ~ 0
Teensy_OUT8
$Comp
L Device:R R11
U 1 1 608E24B5
P 2000 1050
F 0 "R11" V 1900 1050 50  0000 C CNN
F 1 "220" V 2000 1050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1930 1050 50  0001 C CNN
F 3 "~" H 2000 1050 50  0001 C CNN
F 4 "C17557" V 2000 1050 50  0001 C CNN "LCSC"
	1    2000 1050
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 608E2BC2
P 1550 1050
F 0 "R10" V 1450 1050 50  0000 C CNN
F 1 "10k" V 1550 1050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1480 1050 50  0001 C CNN
F 3 "~" H 1550 1050 50  0001 C CNN
F 4 "C17414" V 1550 1050 50  0001 C CNN "LCSC"
	1    1550 1050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0136
U 1 1 608E3956
P 1400 1050
F 0 "#PWR0136" H 1400 800 50  0001 C CNN
F 1 "GND" H 1405 877 50  0000 C CNN
F 2 "" H 1400 1050 50  0001 C CNN
F 3 "" H 1400 1050 50  0001 C CNN
	1    1400 1050
	0    1    -1   0   
$EndComp
Wire Wire Line
	1700 1450 1350 1450
Wire Wire Line
	1350 1750 1700 1750
Wire Wire Line
	1700 1050 1750 1050
Wire Wire Line
	1350 1450 1350 1150
Wire Wire Line
	1350 1150 1750 1150
Wire Wire Line
	1750 1150 1750 1050
Connection ~ 1750 1050
Wire Wire Line
	1750 1050 1850 1050
Wire Wire Line
	1700 2150 1750 2150
Wire Wire Line
	1350 1750 1350 2000
Wire Wire Line
	1350 2000 1750 2000
Wire Wire Line
	1750 2000 1750 2150
Connection ~ 1750 2150
Wire Wire Line
	1750 2150 1850 2150
$Comp
L Connector_Generic:Conn_02x08_Top_Bottom J4
U 1 1 609040B8
P 1500 5800
F 0 "J4" H 1550 6317 50  0000 C CNN
F 1 "OUT_PWR" H 1550 6226 50  0000 C CNN
F 2 "Connector_Molex:Molex_Mini-Fit_Jr_5566-16A_2x08_P4.20mm_Vertical" H 1500 5800 50  0001 C CNN
F 3 "~" H 1500 5800 50  0001 C CNN
F 4 "C98644" H 1500 5800 50  0001 C CNN "LCSC"
	1    1500 5800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2300 1600 2400 1600
Wire Wire Line
	2400 1600 2400 1750
Wire Wire Line
	2400 1750 2300 1750
Wire Wire Line
	2300 1300 2400 1300
Wire Wire Line
	2400 1300 2400 1450
Wire Wire Line
	2400 1450 2300 1450
Text GLabel 2400 1750 2    50   Input ~ 0
J4_OUT7
Text GLabel 2400 1450 2    50   Input ~ 0
J4_OUT8
Text GLabel 1200 6100 0    50   Input ~ 0
J4_OUT8
Text GLabel 1700 6100 2    50   Input ~ 0
J4_OUT7
Text GLabel 2150 3700 2    50   Input ~ 0
Teensy_OUT6
$Comp
L Device:R R17
U 1 1 60947F6B
P 2000 3700
F 0 "R17" V 1900 3700 50  0000 C CNN
F 1 "220" V 2000 3700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1930 3700 50  0001 C CNN
F 3 "~" H 2000 3700 50  0001 C CNN
F 4 "C17557" V 2000 3700 50  0001 C CNN "LCSC"
	1    2000 3700
	0    1    1    0   
$EndComp
$Comp
L Device:R R16
U 1 1 60947F75
P 1550 3700
F 0 "R16" V 1450 3700 50  0000 C CNN
F 1 "10k" V 1550 3700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1480 3700 50  0001 C CNN
F 3 "~" H 1550 3700 50  0001 C CNN
F 4 "C17414" V 1550 3700 50  0001 C CNN "LCSC"
	1    1550 3700
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0137
U 1 1 60947F7F
P 1400 3700
F 0 "#PWR0137" H 1400 3450 50  0001 C CNN
F 1 "GND" H 1405 3527 50  0000 C CNN
F 2 "" H 1400 3700 50  0001 C CNN
F 3 "" H 1400 3700 50  0001 C CNN
	1    1400 3700
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0138
U 1 1 60947F89
P 1700 2850
F 0 "#PWR0138" H 1700 2600 50  0001 C CNN
F 1 "GND" H 1705 2677 50  0000 C CNN
F 2 "" H 1700 2850 50  0001 C CNN
F 3 "" H 1700 2850 50  0001 C CNN
	1    1700 2850
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0139
U 1 1 60947F93
P 1700 3150
F 0 "#PWR0139" H 1700 2900 50  0001 C CNN
F 1 "GND" H 1705 2977 50  0000 C CNN
F 2 "" H 1700 3150 50  0001 C CNN
F 3 "" H 1700 3150 50  0001 C CNN
	1    1700 3150
	0    1    -1   0   
$EndComp
$Comp
L Transistor-FET:IRF9956 Q4
U 1 1 60947F9D
P 2000 3100
F 0 "Q4" H 2000 3500 50  0000 C CNN
F 1 "BSO615NG" H 2000 2750 50  0000 C CNN
F 2 "JoeMBMS:SO-8_IRF956" H 2000 3000 50  0001 C CNN
F 3 "" H 2000 3000 50  0001 C CNN
F 4 "C693398" H 2000 3100 50  0001 C CNN "LCSC"
	1    2000 3100
	1    0    0    -1  
$EndComp
Text GLabel 2150 2600 2    50   Input ~ 0
Teensy_OUT5
$Comp
L Device:R R15
U 1 1 60947FA8
P 2000 2600
F 0 "R15" V 1900 2600 50  0000 C CNN
F 1 "220" V 2000 2600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1930 2600 50  0001 C CNN
F 3 "~" H 2000 2600 50  0001 C CNN
F 4 "C17557" V 2000 2600 50  0001 C CNN "LCSC"
	1    2000 2600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0140
U 1 1 60947FBC
P 1400 2600
F 0 "#PWR0140" H 1400 2350 50  0001 C CNN
F 1 "GND" H 1405 2427 50  0000 C CNN
F 2 "" H 1400 2600 50  0001 C CNN
F 3 "" H 1400 2600 50  0001 C CNN
	1    1400 2600
	0    1    -1   0   
$EndComp
Wire Wire Line
	1700 3000 1350 3000
Wire Wire Line
	1350 3300 1700 3300
Wire Wire Line
	1700 2600 1750 2600
Wire Wire Line
	1350 3000 1350 2700
Wire Wire Line
	1350 2700 1750 2700
Wire Wire Line
	1750 2700 1750 2600
Connection ~ 1750 2600
Wire Wire Line
	1750 2600 1850 2600
Wire Wire Line
	1700 3700 1750 3700
Wire Wire Line
	1350 3300 1350 3550
Wire Wire Line
	1350 3550 1750 3550
Wire Wire Line
	1750 3550 1750 3700
Connection ~ 1750 3700
Wire Wire Line
	1750 3700 1850 3700
Wire Wire Line
	2300 3150 2400 3150
Wire Wire Line
	2400 3150 2400 3300
Wire Wire Line
	2400 3300 2300 3300
Wire Wire Line
	2300 2850 2400 2850
Wire Wire Line
	2400 2850 2400 3000
Wire Wire Line
	2400 3000 2300 3000
Text GLabel 2400 3300 2    50   Input ~ 0
J4_OUT6
Text GLabel 2400 3000 2    50   Input ~ 0
J4_OUT5
Text GLabel 1200 6000 0    50   Input ~ 0
J4_OUT5
Text GLabel 1700 6000 2    50   Input ~ 0
J4_OUT6
$Comp
L Device:C C5
U 1 1 60970CEF
P 9200 1000
F 0 "C5" H 9400 950 50  0000 R CNN
F 1 "100uF" H 9550 1050 50  0000 R CNN
F 2 "Capacitor_SMD:C_Elec_6.3x7.7" H 9238 850 50  0001 C CNN
F 3 "~" H 9200 1000 50  0001 C CNN
F 4 "C127971" H 9200 1000 50  0001 C CNN "LCSC"
	1    9200 1000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0141
U 1 1 60970CF9
P 9200 1150
F 0 "#PWR0141" H 9200 900 50  0001 C CNN
F 1 "GND" H 9205 977 50  0000 C CNN
F 2 "" H 9200 1150 50  0001 C CNN
F 3 "" H 9200 1150 50  0001 C CNN
	1    9200 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0142
U 1 1 60980C0B
P 9200 850
F 0 "#PWR0142" H 9200 700 50  0001 C CNN
F 1 "+12V" H 9215 1023 50  0000 C CNN
F 2 "" H 9200 850 50  0001 C CNN
F 3 "" H 9200 850 50  0001 C CNN
	1    9200 850 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 60989CC4
P 4900 7000
F 0 "C2" H 4785 6954 50  0000 R CNN
F 1 "10uF" H 4785 7045 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4938 6850 50  0001 C CNN
F 3 "~" H 4900 7000 50  0001 C CNN
F 4 "C15850" H 4900 7000 50  0001 C CNN "LCSC"
	1    4900 7000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0143
U 1 1 60989CCE
P 4900 7150
F 0 "#PWR0143" H 4900 6900 50  0001 C CNN
F 1 "GND" H 4905 6977 50  0000 C CNN
F 2 "" H 4900 7150 50  0001 C CNN
F 3 "" H 4900 7150 50  0001 C CNN
	1    4900 7150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0144
U 1 1 6099D4BE
P 1200 6200
F 0 "#PWR0144" H 1200 5950 50  0001 C CNN
F 1 "GND" H 1205 6027 50  0000 C CNN
F 2 "" H 1200 6200 50  0001 C CNN
F 3 "" H 1200 6200 50  0001 C CNN
	1    1200 6200
	-1   0    0    -1  
$EndComp
Connection ~ 3600 7350
Wire Wire Line
	3600 7350 4100 7350
$Comp
L Transistor-FET:QM6015D Q5
U 1 1 609CCC88
P 10550 1000
F 0 "Q5" V 10800 950 50  0000 L CNN
F 1 "P-Chanel Mosfet" V 10900 750 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TO-252-2" H 10750 925 50  0001 L CNN
F 3 "" V 10550 1000 50  0001 L CNN
F 4 "C416264" V 10550 1000 50  0001 C CNN "LCSC"
	1    10550 1000
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR0145
U 1 1 609D0564
P 10750 900
F 0 "#PWR0145" H 10750 750 50  0001 C CNN
F 1 "+12V" H 10765 1073 50  0000 C CNN
F 2 "" H 10750 900 50  0001 C CNN
F 3 "" H 10750 900 50  0001 C CNN
	1    10750 900 
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0146
U 1 1 609D0E06
P 10550 1200
F 0 "#PWR0146" H 10550 950 50  0001 C CNN
F 1 "GND" H 10555 1027 50  0000 C CNN
F 2 "" H 10550 1200 50  0001 C CNN
F 3 "" H 10550 1200 50  0001 C CNN
	1    10550 1200
	1    0    0    -1  
$EndComp
Text GLabel 1700 6200 2    50   Input ~ 0
J4_12V
Text GLabel 10350 900  0    50   Input ~ 0
J4_12V
$Comp
L power:GND #PWR0147
U 1 1 60A0BC85
P 1850 5600
F 0 "#PWR0147" H 1850 5350 50  0001 C CNN
F 1 "GND" H 1855 5427 50  0000 C CNN
F 2 "" H 1850 5600 50  0001 C CNN
F 3 "" H 1850 5600 50  0001 C CNN
	1    1850 5600
	0    -1   1    0   
$EndComp
Wire Wire Line
	1700 5700 1800 5700
Wire Wire Line
	1800 5700 1800 5600
Wire Wire Line
	1800 5600 1850 5600
Wire Wire Line
	1700 5600 1800 5600
Connection ~ 1800 5600
$Comp
L power:+5V #PWR0148
U 1 1 60A1AA8C
P 1700 5500
F 0 "#PWR0148" H 1700 5350 50  0001 C CNN
F 1 "+5V" V 1715 5628 50  0000 L CNN
F 2 "" H 1700 5500 50  0001 C CNN
F 3 "" H 1700 5500 50  0001 C CNN
	1    1700 5500
	0    1    1    0   
$EndComp
$Comp
L teensy:Teensy3.2 U1
U 1 1 60A3CD2D
P 5500 2200
F 0 "U1" H 5500 3837 60  0000 C CNN
F 1 "Teensy3.2" H 5500 3731 60  0000 C CNN
F 2 "Teensy:Teensy30_31_32_LC_Mod" H 5500 1450 60  0001 C CNN
F 3 "" H 5500 1450 60  0000 C CNN
	1    5500 2200
	1    0    0    -1  
$EndComp
Text GLabel 1700 5800 2    50   Input ~ 0
J4_OUT2
Text GLabel 1700 5900 2    50   Input ~ 0
J4_OUT4
$Comp
L power:+12V #PWR0150
U 1 1 60A52F85
P 1200 5500
F 0 "#PWR0150" H 1200 5350 50  0001 C CNN
F 1 "+12V" V 1215 5628 50  0000 L CNN
F 2 "" H 1200 5500 50  0001 C CNN
F 3 "" H 1200 5500 50  0001 C CNN
	1    1200 5500
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR0151
U 1 1 60A56775
P 1200 5600
F 0 "#PWR0151" H 1200 5450 50  0001 C CNN
F 1 "+12V" V 1215 5728 50  0000 L CNN
F 2 "" H 1200 5600 50  0001 C CNN
F 3 "" H 1200 5600 50  0001 C CNN
	1    1200 5600
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR0152
U 1 1 60A56A4D
P 1200 5700
F 0 "#PWR0152" H 1200 5550 50  0001 C CNN
F 1 "+12V" V 1215 5828 50  0000 L CNN
F 2 "" H 1200 5700 50  0001 C CNN
F 3 "" H 1200 5700 50  0001 C CNN
	1    1200 5700
	0    -1   -1   0   
$EndComp
Text GLabel 1200 5800 0    50   Input ~ 0
J4_OUT1
Text GLabel 1200 5900 0    50   Input ~ 0
J4_OUT3
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J2
U 1 1 60A9D01A
P 1400 4850
F 0 "J2" H 1450 5067 50  0000 C CNN
F 1 "Analog_Sens" H 1450 4976 50  0000 C CNN
F 2 "Connector_Molex:Molex_Mini-Fit_Jr_5566-04A_2x02_P4.20mm_Vertical" H 1400 4850 50  0001 C CNN
F 3 "~" H 1400 4850 50  0001 C CNN
F 4 "C29973" H 1400 4850 50  0001 C CNN "LCSC"
	1    1400 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0154
U 1 1 60A9E28E
P 1200 4850
F 0 "#PWR0154" H 1200 4600 50  0001 C CNN
F 1 "GND" H 1205 4677 50  0000 C CNN
F 2 "" H 1200 4850 50  0001 C CNN
F 3 "" H 1200 4850 50  0001 C CNN
	1    1200 4850
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0155
U 1 1 60A9E9CC
P 1700 4850
F 0 "#PWR0155" H 1700 4700 50  0001 C CNN
F 1 "+5V" V 1715 4978 50  0000 L CNN
F 2 "" H 1700 4850 50  0001 C CNN
F 3 "" H 1700 4850 50  0001 C CNN
	1    1700 4850
	0    1    1    0   
$EndComp
$Comp
L Device:R R18
U 1 1 60AA0C3E
P 2200 4950
F 0 "R18" V 2100 4950 50  0000 C CNN
F 1 "10k" V 2200 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2130 4950 50  0001 C CNN
F 3 "~" H 2200 4950 50  0001 C CNN
F 4 "C17414" V 2200 4950 50  0001 C CNN "LCSC"
	1    2200 4950
	0    1    1    0   
$EndComp
$Comp
L Device:R R19
U 1 1 60AA29F3
P 2600 4950
F 0 "R19" V 2500 4950 50  0000 C CNN
F 1 "10k" V 2600 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2530 4950 50  0001 C CNN
F 3 "~" H 2600 4950 50  0001 C CNN
F 4 "C17414" V 2600 4950 50  0001 C CNN "LCSC"
	1    2600 4950
	0    1    1    0   
$EndComp
$Comp
L Device:R R20
U 1 1 60AA2F04
P 2900 4950
F 0 "R20" V 2800 4950 50  0000 C CNN
F 1 "10k" V 2900 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2830 4950 50  0001 C CNN
F 3 "~" H 2900 4950 50  0001 C CNN
F 4 "C17414" V 2900 4950 50  0001 C CNN "LCSC"
	1    2900 4950
	0    1    1    0   
$EndComp
$Comp
L Device:R R21
U 1 1 60AAF92A
P 2200 5150
F 0 "R21" V 2100 5150 50  0000 C CNN
F 1 "10k" V 2200 5150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2130 5150 50  0001 C CNN
F 3 "~" H 2200 5150 50  0001 C CNN
F 4 "C17414" V 2200 5150 50  0001 C CNN "LCSC"
	1    2200 5150
	0    1    1    0   
$EndComp
$Comp
L Device:R R22
U 1 1 60AAF934
P 2600 5150
F 0 "R22" V 2500 5150 50  0000 C CNN
F 1 "10k" V 2600 5150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2530 5150 50  0001 C CNN
F 3 "~" H 2600 5150 50  0001 C CNN
F 4 "C17414" V 2600 5150 50  0001 C CNN "LCSC"
	1    2600 5150
	0    1    1    0   
$EndComp
$Comp
L Device:R R23
U 1 1 60AAF93E
P 2900 5150
F 0 "R23" V 2800 5150 50  0000 C CNN
F 1 "10k" V 2900 5150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2830 5150 50  0001 C CNN
F 3 "~" H 2900 5150 50  0001 C CNN
F 4 "C17414" V 2900 5150 50  0001 C CNN "LCSC"
	1    2900 5150
	0    1    1    0   
$EndComp
Wire Wire Line
	1700 4950 2050 4950
Wire Wire Line
	1200 4950 1150 4950
Wire Wire Line
	1150 4950 1150 5150
Wire Wire Line
	1150 5150 2050 5150
$Comp
L power:GND #PWR0156
U 1 1 60AB9DDE
P 3200 5050
F 0 "#PWR0156" H 3200 4800 50  0001 C CNN
F 1 "GND" H 3205 4877 50  0000 C CNN
F 2 "" H 3200 5050 50  0001 C CNN
F 3 "" H 3200 5050 50  0001 C CNN
	1    3200 5050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3050 4950 3200 4950
Wire Wire Line
	3200 4950 3200 5050
Wire Wire Line
	3200 5050 3200 5150
Wire Wire Line
	3200 5150 3050 5150
Connection ~ 3200 5050
Wire Wire Line
	2350 4950 2400 4950
Wire Wire Line
	2350 5150 2400 5150
Text GLabel 4500 2950 0    50   Input ~ 0
Teensy_ASEN1
Text GLabel 4500 2850 0    50   Input ~ 0
Teensy_ASEN2
Text GLabel 2500 5300 2    50   Input ~ 0
Teensy_ASEN1
Text GLabel 2500 4750 2    50   Input ~ 0
Teensy_ASEN2
Wire Wire Line
	2400 4950 2400 4750
Wire Wire Line
	2400 4750 2500 4750
Connection ~ 2400 4950
Wire Wire Line
	2400 4950 2450 4950
Wire Wire Line
	2400 5150 2400 5300
Wire Wire Line
	2400 5300 2500 5300
Connection ~ 2400 5150
Wire Wire Line
	2400 5150 2450 5150
$Comp
L Device:C C6
U 1 1 60AF185E
P 9650 1000
F 0 "C6" H 9535 954 50  0000 R CNN
F 1 "10uF" H 9535 1045 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 9688 850 50  0001 C CNN
F 3 "~" H 9650 1000 50  0001 C CNN
F 4 "C15850" H 9650 1000 50  0001 C CNN "LCSC"
	1    9650 1000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0157
U 1 1 60AF1868
P 9650 1150
F 0 "#PWR0157" H 9650 900 50  0001 C CNN
F 1 "GND" H 9655 977 50  0000 C CNN
F 2 "" H 9650 1150 50  0001 C CNN
F 3 "" H 9650 1150 50  0001 C CNN
	1    9650 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0158
U 1 1 60B26743
P 4900 6850
F 0 "#PWR0158" H 4900 6700 50  0001 C CNN
F 1 "+5V" H 4915 7023 50  0000 C CNN
F 2 "" H 4900 6850 50  0001 C CNN
F 3 "" H 4900 6850 50  0001 C CNN
	1    4900 6850
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0159
U 1 1 60B347B2
P 9650 850
F 0 "#PWR0159" H 9650 700 50  0001 C CNN
F 1 "+12V" H 9665 1023 50  0000 C CNN
F 2 "" H 9650 850 50  0001 C CNN
F 3 "" H 9650 850 50  0001 C CNN
	1    9650 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 609F204D
P 7450 1750
F 0 "#PWR01" H 7450 1600 50  0001 C CNN
F 1 "+5V" H 7465 1923 50  0000 C CNN
F 2 "" H 7450 1750 50  0001 C CNN
F 3 "" H 7450 1750 50  0001 C CNN
	1    7450 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 609F2057
P 7450 1900
F 0 "C7" H 7335 1854 50  0000 R CNN
F 1 "100uF" H 7335 1945 50  0000 R CNN
F 2 "Capacitor_SMD:C_Elec_6.3x7.7" H 7488 1750 50  0001 C CNN
F 3 "~" H 7450 1900 50  0001 C CNN
F 4 "C127971" H 7450 1900 50  0001 C CNN "LCSC"
	1    7450 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 609F2061
P 7450 2050
F 0 "#PWR02" H 7450 1800 50  0001 C CNN
F 1 "GND" H 7455 1877 50  0000 C CNN
F 2 "" H 7450 2050 50  0001 C CNN
F 3 "" H 7450 2050 50  0001 C CNN
	1    7450 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 60ABB33D
P 8750 1000
F 0 "C8" H 8950 950 50  0000 R CNN
F 1 "100uF" H 9100 1050 50  0000 R CNN
F 2 "Capacitor_SMD:C_Elec_6.3x7.7" H 8788 850 50  0001 C CNN
F 3 "~" H 8750 1000 50  0001 C CNN
F 4 "C127971" H 8750 1000 50  0001 C CNN "LCSC"
	1    8750 1000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 60ABB347
P 8750 1150
F 0 "#PWR04" H 8750 900 50  0001 C CNN
F 1 "GND" H 8755 977 50  0000 C CNN
F 2 "" H 8750 1150 50  0001 C CNN
F 3 "" H 8750 1150 50  0001 C CNN
	1    8750 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR03
U 1 1 60ABB351
P 8750 850
F 0 "#PWR03" H 8750 700 50  0001 C CNN
F 1 "+12V" H 8765 1023 50  0000 C CNN
F 2 "" H 8750 850 50  0001 C CNN
F 3 "" H 8750 850 50  0001 C CNN
	1    8750 850 
	1    0    0    -1  
$EndComp
NoConn ~ 4500 950 
NoConn ~ 4500 1050
NoConn ~ 4500 1150
NoConn ~ 4500 2250
NoConn ~ 4500 2350
NoConn ~ 4500 2550
NoConn ~ 4500 2650
NoConn ~ 4500 2750
NoConn ~ 6500 3050
NoConn ~ 6500 2950
NoConn ~ 6500 2850
NoConn ~ 6500 2750
NoConn ~ 6500 2450
NoConn ~ 6500 2250
NoConn ~ 6500 2150
NoConn ~ 6500 2050
NoConn ~ 6500 1950
NoConn ~ 6500 1850
NoConn ~ 6500 1750
NoConn ~ 6500 1650
NoConn ~ 6500 1550
NoConn ~ 6500 1450
NoConn ~ 6500 1350
NoConn ~ 6500 1250
NoConn ~ 6500 1150
NoConn ~ 6500 1050
NoConn ~ 6500 950 
NoConn ~ 6500 850 
NoConn ~ 4000 6950
$Comp
L Device:R R14
U 1 1 60947FB2
P 1550 2600
F 0 "R14" V 1450 2600 50  0000 C CNN
F 1 "10k" V 1550 2600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1480 2600 50  0001 C CNN
F 3 "~" H 1550 2600 50  0001 C CNN
F 4 "C17414" V 1550 2600 50  0001 C CNN "LCSC"
	1    1550 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	9500 3000 9550 3000
Wire Wire Line
	9500 2800 9500 3000
Wire Wire Line
	9550 2800 9500 2800
Wire Wire Line
	9500 3500 9550 3500
Wire Wire Line
	9500 3300 9500 3500
Wire Wire Line
	9550 3300 9500 3300
Text GLabel 9550 3800 2    50   Input ~ 0
Teensy_IN3
Text GLabel 9550 4300 2    50   Input ~ 0
Teensy_IN4
Text GLabel 9550 2800 2    50   Input ~ 0
Teensy_IN2
Text GLabel 9550 3300 2    50   Input ~ 0
Teensy_IN1
$Comp
L power:GND #PWR0126
U 1 1 607CEDE9
P 9850 4500
F 0 "#PWR0126" H 9850 4250 50  0001 C CNN
F 1 "GND" H 9855 4327 50  0000 C CNN
F 2 "" H 9850 4500 50  0001 C CNN
F 3 "" H 9850 4500 50  0001 C CNN
	1    9850 4500
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 607CE691
P 9850 4000
F 0 "#PWR0125" H 9850 3750 50  0001 C CNN
F 1 "GND" H 9855 3827 50  0000 C CNN
F 2 "" H 9850 4000 50  0001 C CNN
F 3 "" H 9850 4000 50  0001 C CNN
	1    9850 4000
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 607CE0F7
P 9850 3500
F 0 "#PWR0124" H 9850 3250 50  0001 C CNN
F 1 "GND" H 9855 3327 50  0000 C CNN
F 2 "" H 9850 3500 50  0001 C CNN
F 3 "" H 9850 3500 50  0001 C CNN
	1    9850 3500
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0123
U 1 1 607CDA0C
P 9850 3000
F 0 "#PWR0123" H 9850 2750 50  0001 C CNN
F 1 "GND" H 9855 2827 50  0000 C CNN
F 2 "" H 9850 3000 50  0001 C CNN
F 3 "" H 9850 3000 50  0001 C CNN
	1    9850 3000
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0122
U 1 1 607CD427
P 9400 4200
F 0 "#PWR0122" H 9400 4050 50  0001 C CNN
F 1 "+5V" V 9415 4328 50  0000 L CNN
F 2 "" H 9400 4200 50  0001 C CNN
F 3 "" H 9400 4200 50  0001 C CNN
	1    9400 4200
	0    1    -1   0   
$EndComp
$Comp
L power:+5V #PWR0121
U 1 1 607CCF79
P 9400 3700
F 0 "#PWR0121" H 9400 3550 50  0001 C CNN
F 1 "+5V" V 9415 3828 50  0000 L CNN
F 2 "" H 9400 3700 50  0001 C CNN
F 3 "" H 9400 3700 50  0001 C CNN
	1    9400 3700
	0    1    -1   0   
$EndComp
$Comp
L power:+5V #PWR0120
U 1 1 607CC9EB
P 9400 3200
F 0 "#PWR0120" H 9400 3050 50  0001 C CNN
F 1 "+5V" V 9415 3328 50  0000 L CNN
F 2 "" H 9400 3200 50  0001 C CNN
F 3 "" H 9400 3200 50  0001 C CNN
	1    9400 3200
	0    1    -1   0   
$EndComp
$Comp
L power:+5V #PWR0119
U 1 1 607CC397
P 9400 2700
F 0 "#PWR0119" H 9400 2550 50  0001 C CNN
F 1 "+5V" V 9415 2828 50  0000 L CNN
F 2 "" H 9400 2700 50  0001 C CNN
F 3 "" H 9400 2700 50  0001 C CNN
	1    9400 2700
	0    1    -1   0   
$EndComp
$Comp
L Device:R R9
U 1 1 607C2705
P 9700 4500
F 0 "R9" V 9600 4500 50  0000 C CNN
F 1 "10k" V 9700 4500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9630 4500 50  0001 C CNN
F 3 "~" H 9700 4500 50  0001 C CNN
F 4 "C17414" V 9700 4500 50  0001 C CNN "LCSC"
	1    9700 4500
	0    -1   1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 607C2126
P 9700 4000
F 0 "R8" V 9600 4000 50  0000 C CNN
F 1 "10k" V 9700 4000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9630 4000 50  0001 C CNN
F 3 "~" H 9700 4000 50  0001 C CNN
F 4 "C17414" V 9700 4000 50  0001 C CNN "LCSC"
	1    9700 4000
	0    -1   1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 607BB564
P 9700 3000
F 0 "R6" V 9600 3000 50  0000 C CNN
F 1 "10k" V 9700 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9630 3000 50  0001 C CNN
F 3 "~" H 9700 3000 50  0001 C CNN
F 4 "C17414" V 9700 3000 50  0001 C CNN "LCSC"
	1    9700 3000
	0    -1   1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 607BAFD0
P 9700 3500
F 0 "R7" V 9600 3500 50  0000 C CNN
F 1 "10k" V 9700 3500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9630 3500 50  0001 C CNN
F 3 "~" H 9700 3500 50  0001 C CNN
F 4 "C17414" V 9700 3500 50  0001 C CNN "LCSC"
	1    9700 3500
	0    -1   1    0   
$EndComp
Text GLabel 8100 3200 0    50   Input ~ 0
J5_IN1
Text GLabel 8100 2700 0    50   Input ~ 0
J5_IN2
Text GLabel 8100 4200 0    50   Input ~ 0
J5_IN4
Text GLabel 8100 3700 0    50   Input ~ 0
J5_IN3
$Comp
L power:GND #PWR0118
U 1 1 6077DE09
P 8400 4500
F 0 "#PWR0118" H 8400 4250 50  0001 C CNN
F 1 "GND" H 8405 4327 50  0000 C CNN
F 2 "" H 8400 4500 50  0001 C CNN
F 3 "" H 8400 4500 50  0001 C CNN
	1    8400 4500
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 6077D8A0
P 8400 4000
F 0 "#PWR0117" H 8400 3750 50  0001 C CNN
F 1 "GND" H 8405 3827 50  0000 C CNN
F 2 "" H 8400 4000 50  0001 C CNN
F 3 "" H 8400 4000 50  0001 C CNN
	1    8400 4000
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 6077D3F5
P 8400 3500
F 0 "#PWR0116" H 8400 3250 50  0001 C CNN
F 1 "GND" H 8405 3327 50  0000 C CNN
F 2 "" H 8400 3500 50  0001 C CNN
F 3 "" H 8400 3500 50  0001 C CNN
	1    8400 3500
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 6077CD3E
P 8400 3000
F 0 "#PWR0115" H 8400 2750 50  0001 C CNN
F 1 "GND" H 8405 2827 50  0000 C CNN
F 2 "" H 8400 3000 50  0001 C CNN
F 3 "" H 8400 3000 50  0001 C CNN
	1    8400 3000
	0    1    -1   0   
$EndComp
$Comp
L Device:R R5
U 1 1 6077B3EC
P 8250 4200
F 0 "R5" V 8150 4200 50  0000 C CNN
F 1 "4.7k" V 8250 4200 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8180 4200 50  0001 C CNN
F 3 "~" H 8250 4200 50  0001 C CNN
F 4 "C17673" V 8250 4200 50  0001 C CNN "LCSC"
	1    8250 4200
	0    -1   1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 6077ADA9
P 8250 3700
F 0 "R4" V 8150 3700 50  0000 C CNN
F 1 "4.7k" V 8250 3700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8180 3700 50  0001 C CNN
F 3 "~" H 8250 3700 50  0001 C CNN
F 4 "C17673" V 8250 3700 50  0001 C CNN "LCSC"
	1    8250 3700
	0    -1   1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 6077A854
P 8250 3200
F 0 "R3" V 8150 3200 50  0000 C CNN
F 1 "4.7k" V 8250 3200 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8180 3200 50  0001 C CNN
F 3 "~" H 8250 3200 50  0001 C CNN
F 4 "C17673" V 8250 3200 50  0001 C CNN "LCSC"
	1    8250 3200
	0    -1   1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 6077A156
P 8250 2700
F 0 "R2" V 8150 2700 50  0000 C CNN
F 1 "4.7k" V 8250 2700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8180 2700 50  0001 C CNN
F 3 "~" H 8250 2700 50  0001 C CNN
F 4 "C17673" V 8250 2700 50  0001 C CNN "LCSC"
	1    8250 2700
	0    -1   1    0   
$EndComp
$Comp
L 2021-05-20_16-46-13:TLP293-4 U4
U 1 1 60B77729
P 8600 2600
F 0 "U4" H 8900 2787 60  0000 C CNN
F 1 "IS281-4 " H 8900 2681 60  0000 C CNN
F 2 "footprints:TLP293-4" V 8900 1600 60  0001 C CNN
F 3 "" H 8600 2600 60  0000 C CNN
F 4 "C89878" H 8600 2600 50  0001 C CNN "LCSC"
	1    8600 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 3000 9400 3000
Connection ~ 9500 3000
Wire Wire Line
	9500 3500 9400 3500
Connection ~ 9500 3500
Wire Wire Line
	9400 4500 9500 4500
Wire Wire Line
	9500 4500 9500 4300
Wire Wire Line
	9500 4300 9550 4300
Connection ~ 9500 4500
Wire Wire Line
	9500 4500 9550 4500
Wire Wire Line
	9400 4000 9500 4000
Wire Wire Line
	9500 4000 9500 3800
Wire Wire Line
	9500 3800 9550 3800
Connection ~ 9500 4000
Wire Wire Line
	9500 4000 9550 4000
Wire Wire Line
	3200 6850 3100 6850
Wire Wire Line
	3200 6950 3100 6950
$Comp
L Device:R R24
U 1 1 60AB5FA5
P 3000 7400
F 0 "R24" H 3070 7446 50  0000 L CNN
F 1 "120" H 3070 7355 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2930 7400 50  0001 C CNN
F 3 "~" H 3000 7400 50  0001 C CNN
F 4 "C111022" H 3000 7400 50  0001 C CNN "LCSC"
	1    3000 7400
	0    -1   1    0   
$EndComp
Wire Wire Line
	3150 7400 3200 7400
Wire Wire Line
	3200 7400 3200 6950
Connection ~ 3200 6950
Wire Wire Line
	3200 6700 3200 6850
Connection ~ 3200 6850
$Comp
L Jumper:Jumper_3_Bridged12 JP1
U 1 1 60AC4564
P 2600 7400
F 0 "JP1" V 2646 7467 50  0000 L CNN
F 1 "Jumper_3_Bridged12" V 2555 7467 50  0000 L CNN
F 2 "Connector_PinHeader_2.00mm:PinHeader_1x03_P2.00mm_Vertical" H 2600 7400 50  0001 C CNN
F 3 "~" H 2600 7400 50  0001 C CNN
	1    2600 7400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2750 7400 2850 7400
Wire Wire Line
	2600 7150 2600 6700
Wire Wire Line
	2600 6700 3200 6700
NoConn ~ 2600 7650
$Comp
L Transistor-FET:BTS5200-4 S1
U 1 1 60E8955F
P 5500 5150
F 0 "S1" H 5500 4727 50  0000 C CNN
F 1 "BTS5200-4" H 5500 4636 50  0000 C CNN
F 2 "MRV_SMD_Packages:PG-DSO-14-40_EP" H 5450 5150 50  0001 C CNN
F 3 "" H 5450 5150 50  0001 C CNN
	1    5500 5150
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR05
U 1 1 60E8C4A1
P 5500 4700
F 0 "#PWR05" H 5500 4550 50  0001 C CNN
F 1 "+12V" H 5515 4873 50  0000 C CNN
F 2 "" H 5500 4700 50  0001 C CNN
F 3 "" H 5500 4700 50  0001 C CNN
	1    5500 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 60E8CEB0
P 6100 5150
F 0 "#PWR06" H 6100 4900 50  0001 C CNN
F 1 "GND" H 6105 4977 50  0000 C CNN
F 2 "" H 6100 5150 50  0001 C CNN
F 3 "" H 6100 5150 50  0001 C CNN
	1    6100 5150
	0    -1   1    0   
$EndComp
Wire Wire Line
	5900 5150 6100 5150
Wire Wire Line
	5900 5250 6100 5250
Wire Wire Line
	6100 5250 6100 5150
Connection ~ 6100 5150
Text GLabel 5900 5450 2    50   Input ~ 0
Teensy_OUT4
Text GLabel 5900 5350 2    50   Input ~ 0
Teensy_OUT3
Text GLabel 5900 4950 2    50   Input ~ 0
J4_OUT3
Text GLabel 5900 4850 2    50   Input ~ 0
J4_OUT4
Text GLabel 5100 4850 0    50   Input ~ 0
J4_OUT1
Text GLabel 5100 4950 0    50   Input ~ 0
J4_OUT2
Text GLabel 5100 5450 0    50   Input ~ 0
Teensy_OUT1
Text GLabel 5100 5350 0    50   Input ~ 0
Teensy_OUT2
Wire Wire Line
	3600 6250 3600 6550
NoConn ~ 5100 5150
NoConn ~ 5100 5250
$EndSCHEMATC