EESchema Schematic File Version 2
LIBS:Glidertrack-rescue
LIBS:Connectors_kl
LIBS:modules
LIBS:pcb_details
LIBS:power
LIBS:Power_kl
LIBS:st_kl
LIBS:Tittar_kl
LIBS:Transistors_kl
LIBS:RF ICs
LIBS:memory
LIBS:Glidertrack-cache
EELAYER 25 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Glidertrack"
Date "30 dec 2014"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	7900 4350 7900 4550
Wire Wire Line
	7900 4550 7450 4550
Wire Wire Line
	7450 4350 7450 4750
Wire Wire Line
	7550 4350 7550 4550
Connection ~ 7550 4550
Wire Wire Line
	7650 4350 7650 4550
Connection ~ 7650 4550
Wire Wire Line
	7750 4350 7750 4550
Connection ~ 7750 4550
Connection ~ 7450 4550
Wire Wire Line
	9200 3000 9350 3000
Wire Wire Line
	9200 3100 9350 3100
Text Label 9350 3000 0    60   ~ 0
SWDIO
Text Label 9350 3100 0    60   ~ 0
SWCLK
Wire Wire Line
	9200 2900 9350 2900
Wire Wire Line
	9200 2800 9350 2800
Text Label 9350 2900 0    60   ~ 0
USB_DP
Text Label 9350 2800 0    60   ~ 0
USB_DM
Wire Wire Line
	9200 2700 9350 2700
Wire Wire Line
	9200 2600 9350 2600
Text Label 9350 2700 0    60   ~ 0
DEBUG_RX
Text Label 9350 2600 0    60   ~ 0
DEBUG_TX
Wire Wire Line
	9200 2400 9350 2400
Text Label 9350 2400 0    60   ~ 0
MOTOR_SPI_MOSI
Text Label 9350 2300 0    60   ~ 0
MOTOR_SPI_MISO
Wire Wire Line
	9200 2300 9350 2300
Wire Wire Line
	9200 2200 9350 2200
Text Label 9350 2200 0    60   ~ 0
MOTOR_SPI_SCK
Wire Wire Line
	9200 1900 9350 1900
Wire Wire Line
	9200 2000 9350 2000
Text Label 9350 1900 0    60   ~ 0
WIFI_TX
Text Label 9350 2000 0    60   ~ 0
WIFI_RX
Wire Wire Line
	9200 1700 9350 1700
Wire Wire Line
	9350 1800 9200 1800
Text Label 9350 1700 0    60   ~ 0
ENDSTOP1
Text Label 9350 1800 0    60   ~ 0
ENDSTOP2
Text Label 9350 3200 0    60   ~ 0
USB_CABLE
Wire Wire Line
	9350 3200 9200 3200
Wire Wire Line
	9200 3350 10250 3350
Wire Wire Line
	9200 3450 9700 3450
Wire Wire Line
	9700 3450 9700 3550
$Comp
L R R5
U 1 1 5476133F
P 9700 3800
F 0 "R5" V 9600 3600 50  0000 C CNN
F 1 "1K2" V 9700 3800 50  0000 C CNN
F 2 "RES_0603" V 9780 3900 28  0000 C CNN
F 3 "~" H 9700 3800 60  0000 C CNN
	1    9700 3800
	1    0    0    -1  
$EndComp
$Comp
L CRYSTAL XTAL1
U 1 1 5476134E
P 9950 4150
F 0 "XTAL1" H 9950 4425 60  0000 C CNN
F 1 "8/16 MHz" H 9975 4350 60  0000 C CNN
F 2 "CRYSTAL" H 9975 4275 39  0000 C CNN
F 3 "~" H 9950 4150 60  0000 C CNN
	1    9950 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 4050 9700 4250
Wire Wire Line
	10100 4150 10250 4150
Wire Wire Line
	10250 3350 10250 4250
Wire Wire Line
	9700 4150 9800 4150
Connection ~ 9700 4150
Connection ~ 10250 4150
$Comp
L C C9
U 1 1 54761441
P 9700 4450
F 0 "C9" H 9600 4350 50  0000 L CNN
F 1 "15p" H 9600 4550 50  0000 L CNN
F 2 "CAP_0603" V 9800 4300 28  0000 C BNN
F 3 "~" H 9700 4450 60  0000 C CNN
	1    9700 4450
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 54761450
P 10250 4450
F 0 "C10" H 10150 4350 50  0000 L CNN
F 1 "15p" H 10150 4550 50  0000 L CNN
F 2 "CAP_0603" V 10350 4300 28  0000 C BNN
F 3 "~" H 10250 4450 60  0000 C CNN
	1    10250 4450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 5476145F
P 9700 4750
F 0 "#PWR01" H 9790 4730 30  0001 C CNN
F 1 "GND" H 9700 4670 30  0001 C CNN
F 2 "~" H 9700 4750 60  0000 C CNN
F 3 "~" H 9700 4750 60  0000 C CNN
	1    9700 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 4750 9700 4650
Wire Wire Line
	10250 4750 10250 4650
$Comp
L GND #PWR02
U 1 1 547614B3
P 10250 4750
F 0 "#PWR02" H 10340 4730 30  0001 C CNN
F 1 "GND" H 10250 4670 30  0001 C CNN
F 2 "~" H 10250 4750 60  0000 C CNN
F 3 "~" H 10250 4750 60  0000 C CNN
	1    10250 4750
	1    0    0    -1  
$EndComp
$Comp
L USB_MINI_B-RESCUE-Glidertrack XL1
U 1 1 5476152F
P 1400 2000
F 0 "XL1" H 1200 2450 60  0000 C CNN
F 1 "USB_MINI_B" H 1300 2350 60  0000 C CNN
F 2 "~" H 1400 2000 60  0000 C CNN
F 3 "~" H 1400 2000 60  0000 C CNN
	1    1400 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 1800 1800 1800
$Comp
L R R1
U 1 1 54761572
P 2050 1900
F 0 "R1" V 2130 1750 50  0000 C CNN
F 1 "22R" V 2050 1900 50  0000 C CNN
F 2 "RES_0603" V 2130 2000 28  0000 C CNN
F 3 "~" H 2050 1900 60  0000 C CNN
	1    2050 1900
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 54761581
P 2050 2000
F 0 "R2" V 2130 1850 50  0000 C CNN
F 1 "22R" V 2050 2000 50  0000 C CNN
F 2 "RES_0603" V 2130 2100 28  0000 C CNN
F 3 "~" H 2050 2000 60  0000 C CNN
	1    2050 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 1900 1600 1900
Wire Wire Line
	1600 2000 1800 2000
Wire Wire Line
	1750 2000 1750 2150
Wire Wire Line
	1750 2150 1800 2150
Connection ~ 1750 2000
$Comp
L R R3
U 1 1 54761606
P 2050 2150
F 0 "R3" V 2130 2000 50  0000 C CNN
F 1 "1K5" V 2050 2150 50  0000 C CNN
F 2 "RES_0603" V 2130 2250 28  0000 C CNN
F 3 "~" H 2050 2150 60  0000 C CNN
	1    2050 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	2300 1900 2450 1900
Wire Wire Line
	2300 2000 2450 2000
Wire Wire Line
	2300 2150 2450 2150
Text Label 2450 1900 0    60   ~ 0
USB_DM
Text Label 2450 2000 0    60   ~ 0
USB_DP
Text Label 2450 2150 0    60   ~ 0
USB_CABLE
Wire Wire Line
	1600 2100 1700 2100
Wire Wire Line
	1600 2200 1700 2200
Wire Wire Line
	1250 2500 1250 2900
$Comp
L R R4
U 1 1 5476172E
P 1700 2550
F 0 "R4" V 1780 2400 50  0000 C CNN
F 1 "R" V 1700 2550 50  0000 C CNN
F 2 "RES_0603" V 1780 2650 28  0000 C CNN
F 3 "~" H 1700 2550 60  0000 C CNN
	1    1700 2550
	-1   0    0    1   
$EndComp
Wire Wire Line
	1700 2100 1700 2300
Wire Wire Line
	1700 2800 1700 2850
Wire Wire Line
	1700 2850 1250 2850
Connection ~ 1250 2850
$Comp
L GND #PWR03
U 1 1 54761894
P 1250 2900
F 0 "#PWR03" H 1340 2880 30  0001 C CNN
F 1 "GND" H 1250 2820 30  0001 C CNN
F 2 "~" H 1250 2900 60  0000 C CNN
F 3 "~" H 1250 2900 60  0000 C CNN
	1    1250 2900
	1    0    0    -1  
$EndComp
Connection ~ 1700 2200
$Comp
L ST_SWD XL2
U 1 1 54761961
P 1300 3950
F 0 "XL2" H 1150 4250 60  0000 C CNN
F 1 "ST_SWD" H 1500 4250 60  0000 C CNN
F 2 "~" H 1300 3950 60  0000 C CNN
F 3 "~" H 1300 3950 60  0000 C CNN
	1    1300 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 3800 1900 3800
Wire Wire Line
	1650 3900 1900 3900
Wire Wire Line
	1650 4000 1750 4000
Wire Wire Line
	1750 4000 1750 3550
Wire Wire Line
	1650 4100 1750 4100
Wire Wire Line
	1750 4100 1750 4250
$Comp
L GND #PWR04
U 1 1 54761A53
P 1750 4250
F 0 "#PWR04" H 1840 4230 30  0001 C CNN
F 1 "GND" H 1750 4170 30  0001 C CNN
F 2 "~" H 1750 4250 60  0000 C CNN
F 3 "~" H 1750 4250 60  0000 C CNN
	1    1750 4250
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR05
U 1 1 54761A78
P 1750 3550
F 0 "#PWR05" H 1750 3510 30  0001 C CNN
F 1 "+3.3V" H 1830 3580 30  0000 C CNN
F 2 "~" H 1750 3550 60  0000 C CNN
F 3 "~" H 1750 3550 60  0000 C CNN
	1    1750 3550
	0    -1   -1   0   
$EndComp
Text Label 1900 3900 0    60   ~ 0
SWDIO
Text Label 1900 3800 0    60   ~ 0
SWCLK
$Comp
L TLV700xx DA2
U 1 1 54761A87
P 7000 9950
F 0 "DA2" H 6850 10250 60  0000 C CNN
F 1 "TLV700XX" H 7000 10150 60  0000 C CNN
F 2 "~" H 7000 9950 60  0000 C CNN
F 3 "~" H 7000 9950 60  0000 C CNN
	1    7000 9950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 9900 6700 9900
Wire Wire Line
	6600 10000 6700 10000
Wire Wire Line
	6600 9300 6600 10000
Connection ~ 6600 9900
Wire Wire Line
	7300 9900 7850 9900
$Comp
L +3.3V #PWR06
U 1 1 54761B51
P 7850 9900
F 0 "#PWR06" H 7850 9860 30  0001 C CNN
F 1 "+3.3V" H 7930 9930 30  0000 C CNN
F 2 "~" H 7850 9900 60  0000 C CNN
F 3 "~" H 7850 9900 60  0000 C CNN
	1    7850 9900
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 54761B60
P 6450 10200
F 0 "C12" H 6350 10100 50  0000 L CNN
F 1 "4u7" H 6350 10300 50  0000 L CNN
F 2 "CAP_0603" V 6550 10050 28  0000 C BNN
F 3 "~" H 6450 10200 60  0000 C CNN
	1    6450 10200
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 54761B6F
P 6200 10200
F 0 "C11" H 6100 10100 50  0000 L CNN
F 1 "0.1u" H 6100 10300 50  0000 L CNN
F 2 "CAP_0603" V 6300 10050 28  0000 C BNN
F 3 "~" H 6200 10200 60  0000 C CNN
	1    6200 10200
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 54761B7E
P 7500 10200
F 0 "C13" H 7400 10100 50  0000 L CNN
F 1 "1n" H 7400 10300 50  0000 L CNN
F 2 "CAP_0603" V 7600 10050 28  0000 C BNN
F 3 "~" H 7500 10200 60  0000 C CNN
	1    7500 10200
	1    0    0    -1  
$EndComp
$Comp
L C C14
U 1 1 54761B8D
P 7750 10200
F 0 "C14" H 7650 10100 50  0000 L CNN
F 1 "1u" H 7650 10300 50  0000 L CNN
F 2 "CAP_0603" V 7850 10050 28  0000 C BNN
F 3 "~" H 7750 10200 60  0000 C CNN
	1    7750 10200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 9300 7500 10000
Connection ~ 7500 9900
Wire Wire Line
	7750 10000 7750 9900
Connection ~ 7750 9900
Wire Wire Line
	7500 10400 7500 10550
Wire Wire Line
	7750 10400 7750 10500
Wire Wire Line
	7750 10500 7500 10500
Connection ~ 7500 10500
Wire Wire Line
	7000 10300 7000 10550
Wire Wire Line
	6200 10400 6200 10500
Wire Wire Line
	6200 10500 6450 10500
Wire Wire Line
	6450 10400 6450 10550
Connection ~ 6450 10500
Wire Wire Line
	6200 10000 6200 9900
Connection ~ 6200 9900
Wire Wire Line
	6450 10000 6450 9900
Connection ~ 6450 9900
$Comp
L GND #PWR07
U 1 1 54761EBE
P 6450 10550
F 0 "#PWR07" H 6540 10530 30  0001 C CNN
F 1 "GND" H 6450 10470 30  0001 C CNN
F 2 "~" H 6450 10550 60  0000 C CNN
F 3 "~" H 6450 10550 60  0000 C CNN
	1    6450 10550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 54761ECD
P 7000 10550
F 0 "#PWR08" H 7090 10530 30  0001 C CNN
F 1 "GND" H 7000 10470 30  0001 C CNN
F 2 "~" H 7000 10550 60  0000 C CNN
F 3 "~" H 7000 10550 60  0000 C CNN
	1    7000 10550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 54761EDC
P 7500 10550
F 0 "#PWR09" H 7590 10530 30  0001 C CNN
F 1 "GND" H 7500 10470 30  0001 C CNN
F 2 "~" H 7500 10550 60  0000 C CNN
F 3 "~" H 7500 10550 60  0000 C CNN
	1    7500 10550
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR010
U 1 1 54761EEB
P 6000 9900
F 0 "#PWR010" H 6000 9860 30  0001 C CNN
F 1 "+5V" H 6080 9930 30  0000 C CNN
F 2 "~" H 6000 9900 60  0000 C CNN
F 3 "~" H 6000 9900 60  0000 C CNN
	1    6000 9900
	-1   0    0    1   
$EndComp
$Comp
L TPS71533 DA1
U 1 1 54761EFA
P 7000 9300
F 0 "DA1" H 6850 9550 60  0000 C CNN
F 1 "TPS71533" H 7000 9450 60  0000 C CNN
F 2 "~" H 7000 9300 60  0000 C CNN
F 3 "~" H 7000 9300 60  0000 C CNN
	1    7000 9300
	1    0    0    1   
$EndComp
Wire Wire Line
	7400 9300 7500 9300
Wire Wire Line
	7000 8850 7000 8700
$Comp
L GND #PWR011
U 1 1 5476200B
P 7000 8700
F 0 "#PWR011" H 7090 8680 30  0001 C CNN
F 1 "GND" H 7000 8620 30  0001 C CNN
F 2 "~" H 7000 8700 60  0000 C CNN
F 3 "~" H 7000 8700 60  0000 C CNN
	1    7000 8700
	-1   0    0    -1  
$EndComp
NoConn ~ 7100 8850
NoConn ~ 6900 8850
$Comp
L C C8
U 1 1 547623E9
P 6650 4350
F 0 "C8" H 6550 4250 50  0000 L CNN
F 1 "1u" H 6550 4450 50  0000 L CNN
F 2 "CAP_0603" V 6750 4200 28  0000 C BNN
F 3 "~" H 6650 4350 60  0000 C CNN
	1    6650 4350
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 547623F8
P 6400 4350
F 0 "C7" H 6300 4250 50  0000 L CNN
F 1 "0.1u" H 6300 4450 50  0000 L CNN
F 2 "CAP_0603" V 6500 4200 28  0000 C BNN
F 3 "~" H 6400 4350 60  0000 C CNN
	1    6400 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 4050 6400 4150
Connection ~ 6400 4050
Wire Wire Line
	6650 4150 6650 4050
Connection ~ 6650 4050
Wire Wire Line
	6400 4550 6400 4750
Wire Wire Line
	6650 4550 6650 4750
$Comp
L C C6
U 1 1 54762569
P 6150 4350
F 0 "C6" H 6050 4250 50  0000 L CNN
F 1 "0.1u" H 6050 4450 50  0000 L CNN
F 2 "CAP_0603" V 6250 4200 28  0000 C BNN
F 3 "~" H 6150 4350 60  0000 C CNN
	1    6150 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 4050 6700 4050
Wire Wire Line
	5900 3900 6700 3900
Wire Wire Line
	6150 3900 6150 4150
Connection ~ 6150 3900
$Comp
L C C5
U 1 1 547627FF
P 5900 4350
F 0 "C5" H 5800 4250 50  0000 L CNN
F 1 "4u7" H 5800 4450 50  0000 L CNN
F 2 "CAP_0603" V 6000 4200 28  0000 C BNN
F 3 "~" H 5900 4350 60  0000 C CNN
	1    5900 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 3750 5900 4150
Wire Wire Line
	4750 3550 6700 3550
Connection ~ 5900 3900
Connection ~ 5900 3750
$Comp
L +3.3V #PWR012
U 1 1 54762A26
P 5850 3750
F 0 "#PWR012" H 5850 3710 30  0001 C CNN
F 1 "+3.3V" H 5930 3780 30  0000 C CNN
F 2 "~" H 5850 3750 60  0000 C CNN
F 3 "~" H 5850 3750 60  0000 C CNN
	1    5850 3750
	-1   0    0    1   
$EndComp
Wire Wire Line
	5850 3750 6700 3750
$Comp
L C C4
U 1 1 54762B06
P 5650 4350
F 0 "C4" H 5550 4250 50  0000 L CNN
F 1 "0.1u" H 5550 4450 50  0000 L CNN
F 2 "CAP_0603" V 5750 4200 28  0000 C BNN
F 3 "~" H 5650 4350 60  0000 C CNN
	1    5650 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 4150 5650 3650
Connection ~ 5650 3650
$Comp
L C C3
U 1 1 54762B79
P 5400 4350
F 0 "C3" H 5300 4250 50  0000 L CNN
F 1 "0.1u" H 5300 4450 50  0000 L CNN
F 2 "CAP_0603" V 5500 4200 28  0000 C BNN
F 3 "~" H 5400 4350 60  0000 C CNN
	1    5400 4350
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 54762B88
P 5150 4350
F 0 "C2" H 5050 4250 50  0000 L CNN
F 1 "0.1u" H 5050 4450 50  0000 L CNN
F 2 "CAP_0603" V 5250 4200 28  0000 C BNN
F 3 "~" H 5150 4350 60  0000 C CNN
	1    5150 4350
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 54762B97
P 4900 4350
F 0 "C1" H 4800 4250 50  0000 L CNN
F 1 "0.1u" H 4800 4450 50  0000 L CNN
F 2 "CAP_0603" V 5000 4200 28  0000 C BNN
F 3 "~" H 4900 4350 60  0000 C CNN
	1    4900 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 4150 5400 3650
Connection ~ 5400 3650
Wire Wire Line
	5150 4150 5150 3550
Connection ~ 5150 3550
Wire Wire Line
	4900 4150 4900 3550
Connection ~ 4900 3550
Wire Wire Line
	5350 3650 6700 3650
$Comp
L +3.3V #PWR013
U 1 1 54762DB2
P 5350 3650
F 0 "#PWR013" H 5350 3610 30  0001 C CNN
F 1 "+3.3V" H 5430 3680 30  0000 C CNN
F 2 "~" H 5350 3650 60  0000 C CNN
F 3 "~" H 5350 3650 60  0000 C CNN
	1    5350 3650
	-1   0    0    1   
$EndComp
$Comp
L +3.3V #PWR014
U 1 1 54762DC1
P 4750 3550
F 0 "#PWR014" H 4750 3510 30  0001 C CNN
F 1 "+3.3V" H 4830 3580 30  0000 C CNN
F 2 "~" H 4750 3550 60  0000 C CNN
F 3 "~" H 4750 3550 60  0000 C CNN
	1    4750 3550
	-1   0    0    1   
$EndComp
$Comp
L +3.3V #PWR015
U 1 1 54762DD0
P 6300 4050
F 0 "#PWR015" H 6300 4010 30  0001 C CNN
F 1 "+3.3V" H 6380 4080 30  0000 C CNN
F 2 "~" H 6300 4050 60  0000 C CNN
F 3 "~" H 6300 4050 60  0000 C CNN
	1    6300 4050
	-1   0    0    1   
$EndComp
Wire Wire Line
	6150 4550 6150 4750
Wire Wire Line
	5900 4550 5900 4750
Wire Wire Line
	5650 4550 5650 4750
Wire Wire Line
	5400 4550 5400 4750
Wire Wire Line
	5150 4550 5150 4750
Wire Wire Line
	4900 4550 4900 4750
$Comp
L GND #PWR016
U 1 1 547630EC
P 6650 4750
F 0 "#PWR016" H 6740 4730 30  0001 C CNN
F 1 "GND" H 6650 4670 30  0001 C CNN
F 2 "~" H 6650 4750 60  0000 C CNN
F 3 "~" H 6650 4750 60  0000 C CNN
	1    6650 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 547630FB
P 6150 4750
F 0 "#PWR017" H 6240 4730 30  0001 C CNN
F 1 "GND" H 6150 4670 30  0001 C CNN
F 2 "~" H 6150 4750 60  0000 C CNN
F 3 "~" H 6150 4750 60  0000 C CNN
	1    6150 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 5476310A
P 5650 4750
F 0 "#PWR018" H 5740 4730 30  0001 C CNN
F 1 "GND" H 5650 4670 30  0001 C CNN
F 2 "~" H 5650 4750 60  0000 C CNN
F 3 "~" H 5650 4750 60  0000 C CNN
	1    5650 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 54763119
P 5900 4750
F 0 "#PWR019" H 5990 4730 30  0001 C CNN
F 1 "GND" H 5900 4670 30  0001 C CNN
F 2 "~" H 5900 4750 60  0000 C CNN
F 3 "~" H 5900 4750 60  0000 C CNN
	1    5900 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 54763137
P 5150 4750
F 0 "#PWR020" H 5240 4730 30  0001 C CNN
F 1 "GND" H 5150 4670 30  0001 C CNN
F 2 "~" H 5150 4750 60  0000 C CNN
F 3 "~" H 5150 4750 60  0000 C CNN
	1    5150 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 54763146
P 5400 4750
F 0 "#PWR021" H 5490 4730 30  0001 C CNN
F 1 "GND" H 5400 4670 30  0001 C CNN
F 2 "~" H 5400 4750 60  0000 C CNN
F 3 "~" H 5400 4750 60  0000 C CNN
	1    5400 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 54763155
P 4900 4750
F 0 "#PWR022" H 4990 4730 30  0001 C CNN
F 1 "GND" H 4900 4670 30  0001 C CNN
F 2 "~" H 4900 4750 60  0000 C CNN
F 3 "~" H 4900 4750 60  0000 C CNN
	1    4900 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 54763164
P 6400 4750
F 0 "#PWR023" H 6490 4730 30  0001 C CNN
F 1 "GND" H 6400 4670 30  0001 C CNN
F 2 "~" H 6400 4750 60  0000 C CNN
F 3 "~" H 6400 4750 60  0000 C CNN
	1    6400 4750
	1    0    0    -1  
$EndComp
$Comp
L SPWF01SX MDL1
U 1 1 54764987
P 13800 3250
F 0 "MDL1" H 14250 4350 60  0000 C CNN
F 1 "SPWF01SX" H 13400 4350 60  0000 C CNN
F 2 "" H 13750 3650 60  0000 C CNN
F 3 "" H 13750 3650 60  0000 C CNN
	1    13800 3250
	1    0    0    -1  
$EndComp
$Comp
L C C17
U 1 1 5481E3DB
P 12500 2000
F 0 "C17" H 12400 1900 50  0000 L CNN
F 1 "C" H 12400 2100 50  0000 L CNN
F 2 "CAP_0603" V 12600 1850 28  0000 C BNN
F 3 "~" H 12500 2000 60  0000 C CNN
	1    12500 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	12800 2300 12950 2300
Wire Wire Line
	12800 1200 12800 2300
Wire Wire Line
	12500 1200 12500 1800
$Comp
L +3.3V #PWR024
U 1 1 5481E4D5
P 10950 1200
F 0 "#PWR024" H 10950 1160 30  0001 C CNN
F 1 "+3.3V" H 11030 1230 30  0000 C CNN
F 2 "~" H 10950 1200 60  0000 C CNN
F 3 "~" H 10950 1200 60  0000 C CNN
	1    10950 1200
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR025
U 1 1 5481E4E4
P 12500 2300
F 0 "#PWR025" H 12590 2280 30  0001 C CNN
F 1 "GND" H 12500 2220 30  0001 C CNN
F 2 "~" H 12500 2300 60  0000 C CNN
F 3 "~" H 12500 2300 60  0000 C CNN
	1    12500 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	12500 2200 12500 2300
Wire Wire Line
	11700 2500 12950 2500
Wire Wire Line
	11700 2600 12950 2600
Wire Wire Line
	14600 2850 15700 2850
Wire Wire Line
	14600 2950 15400 2950
Wire Wire Line
	14600 3050 15100 3050
Wire Wire Line
	13700 4300 13700 4600
Wire Wire Line
	13850 4300 13850 4400
Wire Wire Line
	13850 4400 13700 4400
Connection ~ 13700 4400
$Comp
L GND #PWR026
U 1 1 5481EAB9
P 13700 4600
F 0 "#PWR026" H 13790 4580 30  0001 C CNN
F 1 "GND" H 13700 4520 30  0001 C CNN
F 2 "~" H 13700 4600 60  0000 C CNN
F 3 "~" H 13700 4600 60  0000 C CNN
	1    13700 4600
	1    0    0    -1  
$EndComp
NoConn ~ 9200 3600
NoConn ~ 9200 3700
NoConn ~ 9200 3800
$Comp
L LD1117-RESCUE-Glidertrack DA3
U 1 1 5481EBBD
P 4500 9950
F 0 "DA3" H 4400 10250 60  0000 C CNN
F 1 "LD1117" H 4500 10150 60  0000 C CNN
F 2 "~" H 4500 9950 60  0000 C CNN
F 3 "~" H 4500 9950 60  0000 C CNN
	1    4500 9950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 9900 4200 9900
$Comp
L +Vs #PWR027
U 1 1 5481EC4D
P 3650 9900
F 0 "#PWR027" H 3650 9860 30  0001 C CNN
F 1 "+VS" H 3730 9930 30  0000 C CNN
F 2 "~" H 3650 9900 60  0000 C CNN
F 3 "~" H 3650 9900 60  0000 C CNN
	1    3650 9900
	-1   0    0    1   
$EndComp
$Comp
L C C15
U 1 1 5481EC5C
P 4000 10150
F 0 "C15" H 3900 10050 50  0000 L CNN
F 1 "0.1u" H 3900 10250 50  0000 L CNN
F 2 "CAP_0603" V 4100 10000 28  0000 C BNN
F 3 "~" H 4000 10150 60  0000 C CNN
	1    4000 10150
	1    0    0    -1  
$EndComp
Connection ~ 4000 9900
$Comp
L GND #PWR028
U 1 1 5481ED72
P 4000 10450
F 0 "#PWR028" H 4090 10430 30  0001 C CNN
F 1 "GND" H 4000 10370 30  0001 C CNN
F 2 "~" H 4000 10450 60  0000 C CNN
F 3 "~" H 4000 10450 60  0000 C CNN
	1    4000 10450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR029
U 1 1 5481ED81
P 4500 10450
F 0 "#PWR029" H 4590 10430 30  0001 C CNN
F 1 "GND" H 4500 10370 30  0001 C CNN
F 2 "~" H 4500 10450 60  0000 C CNN
F 3 "~" H 4500 10450 60  0000 C CNN
	1    4500 10450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 10050 5050 10050
Wire Wire Line
	5050 10050 5050 9900
Wire Wire Line
	4900 9900 5250 9900
Connection ~ 5050 9900
$Comp
L +5V #PWR030
U 1 1 5481EF26
P 5250 9900
F 0 "#PWR030" H 5250 9860 30  0001 C CNN
F 1 "+5V" H 5330 9930 30  0000 C CNN
F 2 "~" H 5250 9900 60  0000 C CNN
F 3 "~" H 5250 9900 60  0000 C CNN
	1    5250 9900
	1    0    0    -1  
$EndComp
$Comp
L C C16
U 1 1 5481EF35
P 5200 10150
F 0 "C16" H 5100 10050 50  0000 L CNN
F 1 "10u" H 5100 10250 50  0000 L CNN
F 2 "CAP_0603" V 5300 10000 28  0000 C BNN
F 3 "~" H 5200 10150 60  0000 C CNN
	1    5200 10150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 9950 5200 9900
Connection ~ 5200 9900
Wire Wire Line
	4000 9950 4000 9900
$Comp
L GND #PWR031
U 1 1 5481F075
P 5200 10450
F 0 "#PWR031" H 5290 10430 30  0001 C CNN
F 1 "GND" H 5200 10370 30  0001 C CNN
F 2 "~" H 5200 10450 60  0000 C CNN
F 3 "~" H 5200 10450 60  0000 C CNN
	1    5200 10450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 10450 5200 10350
Wire Wire Line
	4500 10450 4500 10300
Wire Wire Line
	4000 10450 4000 10350
$Comp
L DIODE D1
U 1 1 5481F2B2
P 1800 1500
F 0 "D1" H 1800 1600 40  0000 C CNN
F 1 "DIODE" H 1800 1400 40  0000 C CNN
F 2 "~" H 1800 1500 60  0000 C CNN
F 3 "~" H 1800 1500 60  0000 C CNN
	1    1800 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1800 1800 1800 1700
Wire Wire Line
	1800 1300 1800 1150
$Comp
L +5V #PWR032
U 1 1 5481F502
P 1800 1150
F 0 "#PWR032" H 1800 1110 30  0001 C CNN
F 1 "+5V" H 1880 1180 30  0000 C CNN
F 2 "~" H 1800 1150 60  0000 C CNN
F 3 "~" H 1800 1150 60  0000 C CNN
	1    1800 1150
	0    -1   -1   0   
$EndComp
Text Label 11700 2600 2    60   ~ 0
WIFI_TX
Text Label 11700 2500 2    60   ~ 0
WIFI_RX
NoConn ~ 12950 2700
NoConn ~ 12950 2800
Wire Wire Line
	12950 3900 12800 3900
Wire Wire Line
	12950 4000 12800 4000
Text Label 12800 3900 2    60   ~ 0
WIFI_RESET
Text Label 12800 4000 2    60   ~ 0
WIFI_BOOT
NoConn ~ 14600 2300
NoConn ~ 14600 2400
NoConn ~ 14600 2500
NoConn ~ 14600 2600
NoConn ~ 14600 2700
NoConn ~ 14600 3250
NoConn ~ 14600 3350
NoConn ~ 12950 3050
NoConn ~ 12950 3150
NoConn ~ 12950 3250
NoConn ~ 12950 3350
NoConn ~ 12950 3600
NoConn ~ 12950 3700
NoConn ~ 14600 3950
NoConn ~ 14600 3850
NoConn ~ 14600 3750
NoConn ~ 14600 3650
NoConn ~ 14600 3550
Wire Wire Line
	15100 3050 15100 3200
Wire Wire Line
	15400 2950 15400 3200
Wire Wire Line
	15700 2850 15700 3200
Wire Wire Line
	15100 3700 15100 3600
Wire Wire Line
	15400 3700 15400 3600
Wire Wire Line
	15700 3700 15700 3600
Wire Wire Line
	15100 4200 15100 4600
Wire Wire Line
	15400 4200 15400 4600
Wire Wire Line
	15700 4200 15700 4600
$Comp
L GND #PWR033
U 1 1 5482009D
P 15100 4600
F 0 "#PWR033" H 15150 4650 60  0001 C CNN
F 1 "GND" H 15100 4600 60  0001 C CNN
F 2 "" H 15100 4600 60  0001 C CNN
F 3 "" H 15100 4600 60  0001 C CNN
F 4 "#PWR" H 15190 4580 30  0001 C CNN "Reference"
F 5 "GND" H 15100 4520 30  0001 C CNN "Value"
F 6 "~" H 15100 4600 60  0000 C CNN "Footprint"
F 7 "~" H 15100 4600 60  0000 C CNN "Datasheet"
	1    15100 4600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR034
U 1 1 548200B4
P 15400 4600
F 0 "#PWR034" H 15450 4650 60  0001 C CNN
F 1 "GND" H 15400 4600 60  0001 C CNN
F 2 "" H 15400 4600 60  0001 C CNN
F 3 "" H 15400 4600 60  0001 C CNN
F 4 "#PWR" H 15490 4580 30  0001 C CNN "Reference"
F 5 "GND" H 15400 4520 30  0001 C CNN "Value"
F 6 "~" H 15400 4600 60  0000 C CNN "Footprint"
F 7 "~" H 15400 4600 60  0000 C CNN "Datasheet"
	1    15400 4600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR035
U 1 1 548200CB
P 15700 4600
F 0 "#PWR035" H 15750 4650 60  0001 C CNN
F 1 "GND" H 15700 4600 60  0001 C CNN
F 2 "" H 15700 4600 60  0001 C CNN
F 3 "" H 15700 4600 60  0001 C CNN
F 4 "#PWR" H 15790 4580 30  0001 C CNN "Reference"
F 5 "GND" H 15700 4520 30  0001 C CNN "Value"
F 6 "~" H 15700 4600 60  0000 C CNN "Footprint"
F 7 "~" H 15700 4600 60  0000 C CNN "Datasheet"
	1    15700 4600
	1    0    0    -1  
$EndComp
$Comp
L LED POWER_UP1
U 1 1 5481EAFF
P 15100 3400
F 0 "POWER_UP1" H 15300 3200 50  0000 C CNN
F 1 "LED" H 14950 3200 50  0000 C CNN
F 2 "~" H 15100 3400 60  0000 C CNN
F 3 "~" H 15100 3400 60  0000 C CNN
	1    15100 3400
	0    1    1    0   
$EndComp
$Comp
L R R6
U 1 1 5481EB0E
P 15100 3950
F 0 "R6" V 15180 3800 50  0000 C CNN
F 1 "22R" V 15100 3950 50  0000 C CNN
F 2 "RES_0603" V 15180 4050 28  0000 C CNN
F 3 "~" H 15100 3950 60  0000 C CNN
	1    15100 3950
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 5481EB1D
P 15400 3950
F 0 "R7" V 15480 3800 50  0000 C CNN
F 1 "22R" V 15400 3950 50  0000 C CNN
F 2 "RES_0603" V 15480 4050 28  0000 C CNN
F 3 "~" H 15400 3950 60  0000 C CNN
	1    15400 3950
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 5481EB2C
P 15700 3950
F 0 "R8" V 15780 3800 50  0000 C CNN
F 1 "22R" V 15700 3950 50  0000 C CNN
F 2 "RES_0603" V 15780 4050 28  0000 C CNN
F 3 "~" H 15700 3950 60  0000 C CNN
	1    15700 3950
	1    0    0    -1  
$EndComp
$Comp
L LED LINK_UP1
U 1 1 5481EB60
P 15400 3400
F 0 "LINK_UP1" H 15600 3250 50  0000 C CNN
F 1 "LED" H 15300 3250 50  0000 C CNN
F 2 "~" H 15400 3400 60  0000 C CNN
F 3 "~" H 15400 3400 60  0000 C CNN
	1    15400 3400
	0    1    1    0   
$EndComp
$Comp
L LED RUN1
U 1 1 5481EB7E
P 15700 3400
F 0 "RUN1" H 15900 3250 50  0000 C CNN
F 1 "LED" H 15600 3250 50  0000 C CNN
F 2 "~" H 15700 3400 60  0000 C CNN
F 3 "~" H 15700 3400 60  0000 C CNN
	1    15700 3400
	0    1    1    0   
$EndComp
$Comp
L CONN_2 XL4
U 1 1 5481EB8D
P 1250 6000
F 0 "XL4" V 1200 6000 40  0000 C CNN
F 1 "CONN_2" V 1300 6000 40  0000 C CNN
F 2 "~" H 1250 6000 60  0000 C CNN
F 3 "~" H 1250 6000 60  0000 C CNN
	1    1250 6000
	-1   0    0    1   
$EndComp
Wire Wire Line
	1450 5900 1650 5900
Wire Wire Line
	1450 6100 1650 6100
Text Label 1650 5900 0    60   ~ 0
WIFI_RESET
Text Label 1650 6100 0    60   ~ 0
WIFI_BOOT
$Comp
L CONN_3 XL3
U 1 1 5481ECD1
P 1250 5250
F 0 "XL3" V 1200 5250 40  0000 C CNN
F 1 "CONN_3" V 1300 5250 40  0000 C CNN
F 2 "~" H 1250 5250 60  0000 C CNN
F 3 "~" H 1250 5250 60  0000 C CNN
	1    1250 5250
	1    0    0    1   
$EndComp
Wire Wire Line
	1450 5350 1550 5350
Wire Wire Line
	1550 5350 1550 5500
$Comp
L GND #PWR036
U 1 1 5481EE3A
P 1550 5500
F 0 "#PWR036" H 1640 5480 30  0001 C CNN
F 1 "GND" H 1550 5420 30  0001 C CNN
F 2 "~" H 1550 5500 60  0000 C CNN
F 3 "~" H 1550 5500 60  0000 C CNN
	1    1550 5500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1450 5250 1550 5250
Wire Wire Line
	1450 5150 1550 5150
Text Label 1550 5250 0    60   ~ 0
DEBUG_TX
Text Label 1550 5150 0    60   ~ 0
DEBUG_RX
$Comp
L CONN_3 XL7
U 1 1 5481EF86
P 2300 6850
F 0 "XL7" V 2250 6850 40  0000 C CNN
F 1 "CONN_3" V 2350 6850 40  0000 C CNN
F 2 "~" H 2300 6850 60  0000 C CNN
F 3 "~" H 2300 6850 60  0000 C CNN
	1    2300 6850
	1    0    0    1   
$EndComp
$Comp
L CONN_3 XL5
U 1 1 5481EF95
P 1250 6850
F 0 "XL5" V 1200 6850 40  0000 C CNN
F 1 "CONN_3" V 1300 6850 40  0000 C CNN
F 2 "~" H 1250 6850 60  0000 C CNN
F 3 "~" H 1250 6850 60  0000 C CNN
	1    1250 6850
	1    0    0    1   
$EndComp
Wire Wire Line
	2500 6950 2650 6950
Wire Wire Line
	2650 6950 2650 7100
Wire Wire Line
	1450 6950 1600 6950
Wire Wire Line
	1600 6950 1600 7100
$Comp
L GND #PWR037
U 1 1 5481F0FB
P 2650 7100
F 0 "#PWR037" H 2740 7080 30  0001 C CNN
F 1 "GND" H 2650 7020 30  0001 C CNN
F 2 "~" H 2650 7100 60  0000 C CNN
F 3 "~" H 2650 7100 60  0000 C CNN
	1    2650 7100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR038
U 1 1 5481F10A
P 1600 7100
F 0 "#PWR038" H 1690 7080 30  0001 C CNN
F 1 "GND" H 1600 7020 30  0001 C CNN
F 2 "~" H 1600 7100 60  0000 C CNN
F 3 "~" H 1600 7100 60  0000 C CNN
	1    1600 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 6850 1600 6850
Wire Wire Line
	2500 6850 2650 6850
Text Label 1600 6850 0    60   ~ 0
ENDSTOP1
Text Label 2650 6850 0    60   ~ 0
ENDSTOP2
Wire Wire Line
	2500 6750 2650 6750
Wire Wire Line
	2650 6750 2650 6600
Wire Wire Line
	1600 6750 1450 6750
Wire Wire Line
	1600 6600 1600 6750
$Comp
L +3.3V #PWR039
U 1 1 5481F3B3
P 2650 6600
F 0 "#PWR039" H 2650 6560 30  0001 C CNN
F 1 "+3.3V" H 2730 6630 30  0000 C CNN
F 2 "~" H 2650 6600 60  0000 C CNN
F 3 "~" H 2650 6600 60  0000 C CNN
	1    2650 6600
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR040
U 1 1 5481F3C2
P 1600 6600
F 0 "#PWR040" H 1600 6560 30  0001 C CNN
F 1 "+3.3V" H 1680 6630 30  0000 C CNN
F 2 "~" H 1600 6600 60  0000 C CNN
F 3 "~" H 1600 6600 60  0000 C CNN
	1    1600 6600
	0    -1   -1   0   
$EndComp
$Comp
L CONN_10 XL6
U 1 1 5481F52C
P 1300 8400
F 0 "XL6" V 1250 8400 50  0000 C CNN
F 1 "CONN_10" V 1350 8400 50  0000 C CNN
F 2 "~" H 1300 8400 60  0000 C CNN
F 3 "~" H 1300 8400 60  0000 C CNN
	1    1300 8400
	-1   0    0    1   
$EndComp
Wire Wire Line
	1500 8750 1700 8750
Wire Wire Line
	1700 8750 1700 9000
$Comp
L GND #PWR041
U 1 1 5481F740
P 1700 9000
F 0 "#PWR041" H 1790 8980 30  0001 C CNN
F 1 "GND" H 1700 8920 30  0001 C CNN
F 2 "~" H 1700 9000 60  0000 C CNN
F 3 "~" H 1700 9000 60  0000 C CNN
	1    1700 9000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 8850 1850 8850
Wire Wire Line
	1500 8550 1700 8550
Wire Wire Line
	1500 8350 1700 8350
Wire Wire Line
	1500 8150 1700 8150
Wire Wire Line
	1500 7950 1700 7950
Text Label 1700 7950 0    60   ~ 0
MOTOR_SPI_SCK
Text Label 1700 8150 0    60   ~ 0
MOTOR_SPI_CS
Text Label 1700 8350 0    60   ~ 0
MOTOR_SPI_MISO
Text Label 1700 8550 0    60   ~ 0
MOTOR_SPI_MOSI
Wire Wire Line
	9200 2100 9350 2100
Text Label 9350 2100 0    60   ~ 0
MOTOR_SPI_CS
Wire Wire Line
	1500 8050 1700 8050
Text Label 1700 8050 0    60   ~ 0
MOTOR_STBY
Text Label 1700 8250 0    60   ~ 0
MOTOR_STCK
Wire Wire Line
	1500 8250 1700 8250
Wire Wire Line
	1500 8450 1700 8450
Text Label 1700 8450 0    60   ~ 0
MOTOR_BUSY
Text Label 1700 8650 0    60   ~ 0
MOTOR_FLAG
Wire Wire Line
	1700 8650 1500 8650
Text Label 9350 2500 0    60   ~ 0
MOTOR_STCK
Wire Wire Line
	9350 2500 9200 2500
Wire Wire Line
	6700 1700 6550 1700
Text Label 6550 1700 2    60   ~ 0
MOTOR_STBY
Text Label 6550 1800 2    60   ~ 0
MOTOR_BUSY
Wire Wire Line
	6550 1800 6700 1800
Wire Wire Line
	6700 1900 6550 1900
Text Label 6550 1900 2    60   ~ 0
MOTOR_FLAG
$Comp
L Fuse0R F1
U 1 1 548204A9
P 2100 8850
F 0 "F1" V 2030 8850 50  0000 C CNN
F 1 "FUSE0R" V 2100 8850 50  0000 C CNN
F 2 "~" H 2100 8850 60  0000 C CNN
F 3 "~" H 2100 8850 60  0000 C CNN
	1    2100 8850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2350 8850 2500 8850
$Comp
L +3.3V #PWR042
U 1 1 54820572
P 2500 8850
F 0 "#PWR042" H 2500 8810 30  0001 C CNN
F 1 "+3.3V" H 2580 8880 30  0000 C CNN
F 2 "~" H 2500 8850 60  0000 C CNN
F 3 "~" H 2500 8850 60  0000 C CNN
	1    2500 8850
	1    0    0    -1  
$EndComp
NoConn ~ 6700 3400
$Comp
L GND #PWR043
U 1 1 54820B0D
P 7450 4750
F 0 "#PWR043" H 7540 4730 30  0001 C CNN
F 1 "GND" H 7450 4670 30  0001 C CNN
F 2 "~" H 7450 4750 60  0000 C CNN
F 3 "~" H 7450 4750 60  0000 C CNN
	1    7450 4750
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG044
U 1 1 54820B22
P 3350 10450
F 0 "#FLG044" H 3350 10720 30  0001 C CNN
F 1 "PWR_FLAG" H 3350 10680 30  0000 C CNN
F 2 "~" H 3350 10450 60  0000 C CNN
F 3 "~" H 3350 10450 60  0000 C CNN
	1    3350 10450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 10450 3350 10550
$Comp
L GND #PWR045
U 1 1 54820BF6
P 3350 10550
F 0 "#PWR045" H 3440 10530 30  0001 C CNN
F 1 "GND" H 3350 10470 30  0001 C CNN
F 2 "~" H 3350 10550 60  0000 C CNN
F 3 "~" H 3350 10550 60  0000 C CNN
	1    3350 10550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 9750 1650 9750
Wire Wire Line
	1650 9750 1650 9550
$Comp
L +Vs #PWR046
U 1 1 54820F54
P 1650 9550
F 0 "#PWR046" H 1650 9510 30  0001 C CNN
F 1 "+VS" H 1730 9580 30  0000 C CNN
F 2 "~" H 1650 9550 60  0000 C CNN
F 3 "~" H 1650 9550 60  0000 C CNN
	1    1650 9550
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR047
U 1 1 54820F63
P 1250 10400
F 0 "#PWR047" H 1340 10380 30  0001 C CNN
F 1 "GND" H 1250 10320 30  0001 C CNN
F 2 "~" H 1250 10400 60  0000 C CNN
F 3 "~" H 1250 10400 60  0000 C CNN
	1    1250 10400
	1    0    0    -1  
$EndComp
$Comp
L BARREL_SOCKET XL8
U 1 1 5482111E
P 1350 9850
F 0 "XL8" H 1200 10100 60  0000 C CNN
F 1 "BARREL_SOCKET" V 1050 9850 60  0000 C CNN
F 2 "~" H 1350 9850 60  0000 C CNN
F 3 "~" H 1350 9850 60  0000 C CNN
	1    1350 9850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 10250 1250 10400
NoConn ~ 1550 9950
$Comp
L STM32F10X-48 DD1
U 1 1 547610DE
P 7950 2900
F 0 "DD1" H 7200 4250 60  0000 C CNN
F 1 "STM32F10X-48" H 7850 4250 60  0000 C CNN
F 2 "~" H 7950 2900 60  0000 C CNN
F 3 "~" H 7950 2900 60  0000 C CNN
	1    7950 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 2000 6550 2000
Wire Wire Line
	6700 2100 6550 2100
Wire Wire Line
	6700 2200 6550 2200
Wire Wire Line
	6700 2300 6550 2300
Wire Wire Line
	6700 2400 6550 2400
Wire Wire Line
	6700 2500 6550 2500
Wire Wire Line
	6700 2600 6550 2600
Wire Wire Line
	6700 2700 6550 2700
Wire Wire Line
	6700 2800 6550 2800
Wire Wire Line
	6700 2900 6550 2900
Wire Wire Line
	6700 3000 6550 3000
Wire Wire Line
	6700 3100 6550 3100
Wire Wire Line
	6550 3200 6700 3200
Text Label 6550 2100 2    60   ~ 0
GPIOB4
Text Label 6550 2200 2    60   ~ 0
GPIOB5
Text Label 6550 2300 2    60   ~ 0
GPIOB6
Text Label 6550 2400 2    60   ~ 0
GPIOB7
Text Label 6550 2500 2    60   ~ 0
GPIOB8
Text Label 6550 2600 2    60   ~ 0
GPIOB9
Wire Wire Line
	3000 4100 3150 4100
Wire Wire Line
	3000 4200 3150 4200
Wire Wire Line
	3000 4300 3150 4300
Wire Wire Line
	3000 4400 3150 4400
Wire Wire Line
	3000 4500 3150 4500
Wire Wire Line
	3000 4600 3150 4600
Wire Wire Line
	3000 4700 3150 4700
Wire Wire Line
	3000 4800 3650 4800
Text Label 3150 4200 0    60   ~ 0
GPIOB4
Text Label 3150 4300 0    60   ~ 0
GPIOB5
Text Label 3150 4400 0    60   ~ 0
GPIOB6
Text Label 3150 4500 0    60   ~ 0
GPIOB7
Text Label 3150 4600 0    60   ~ 0
GPIOB8
Text Label 3150 4700 0    60   ~ 0
GPIOB9
Wire Wire Line
	3650 4800 3650 4850
$Comp
L GND #PWR048
U 1 1 54822EA4
P 3650 4850
F 0 "#PWR048" H 3740 4830 30  0001 C CNN
F 1 "GND" H 3650 4770 30  0001 C CNN
F 2 "~" H 3650 4850 60  0000 C CNN
F 3 "~" H 3650 4850 60  0000 C CNN
	1    3650 4850
	1    0    0    -1  
$EndComp
$Comp
L CONN_8 XL9
U 1 1 54823092
P 2800 4450
F 0 "XL9" V 2750 4450 50  0000 C CNN
F 1 "CONN_8" V 2850 4450 50  0000 C CNN
F 2 "~" H 2800 4450 60  0000 C CNN
F 3 "~" H 2800 4450 60  0000 C CNN
	1    2800 4450
	-1   0    0    1   
$EndComp
$Comp
L R R9
U 1 1 54885714
P 11950 2050
F 0 "R9" V 12030 1900 50  0000 C CNN
F 1 "22K" V 11950 2050 50  0000 C CNN
F 2 "RES_0603" V 12030 2150 28  0000 C CNN
F 3 "~" H 11950 2050 60  0000 C CNN
	1    11950 2050
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 54885723
P 12150 2050
F 0 "R10" V 12230 1900 50  0000 C CNN
F 1 "22K" V 12150 2050 50  0000 C CNN
F 2 "RES_0603" V 12230 2150 28  0000 C CNN
F 3 "~" H 12150 2050 60  0000 C CNN
	1    12150 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	11950 1800 11950 1700
Wire Wire Line
	12150 1800 12150 1700
Wire Wire Line
	11950 2300 11950 2600
Connection ~ 11950 2600
Wire Wire Line
	12150 2300 12150 2500
Connection ~ 12150 2500
$Comp
L EEPROM_SPI DA4
U 1 1 54888782
P 4550 2050
F 0 "DA4" H 4750 2450 60  0000 C CNN
F 1 "EEPROM_SPI" H 4250 2450 60  0000 C CNN
F 2 "" H 4550 2050 60  0000 C CNN
F 3 "" H 4550 2050 60  0000 C CNN
	1    4550 2050
	-1   0    0    -1  
$EndComp
Text Label 6550 3200 2    60   ~ 0
EE_MOSI
Text Label 6550 3100 2    60   ~ 0
EE_MISO
Text Label 6550 3000 2    60   ~ 0
EE_SCK
Text Label 6550 2900 2    60   ~ 0
EE_CS
Wire Wire Line
	5050 2150 5250 2150
Wire Wire Line
	5050 2050 5250 2050
Wire Wire Line
	5050 1950 5250 1950
Wire Wire Line
	5050 1850 5250 1850
Text Label 5250 1950 0    60   ~ 0
EE_MISO
Text Label 5250 2150 0    60   ~ 0
EE_MOSI
Text Label 5250 2050 0    60   ~ 0
EE_SCK
Text Label 5250 1850 0    60   ~ 0
EE_CS
Wire Wire Line
	3600 1850 4150 1850
Wire Wire Line
	3600 1500 3600 2100
Connection ~ 3600 1850
$Comp
L +3.3V #PWR049
U 1 1 54888DF5
P 3600 1500
F 0 "#PWR049" H 3600 1460 30  0001 C CNN
F 1 "+3.3V" H 3680 1530 30  0000 C CNN
F 2 "~" H 3600 1500 60  0000 C CNN
F 3 "~" H 3600 1500 60  0000 C CNN
	1    3600 1500
	0    -1   -1   0   
$EndComp
$Comp
L C C18
U 1 1 54888E04
P 3600 2300
F 0 "C18" H 3500 2200 50  0000 L CNN
F 1 "0.1u" H 3500 2400 50  0000 L CNN
F 2 "CAP_0603" V 3700 2150 28  0000 C BNN
F 3 "~" H 3600 2300 60  0000 C CNN
	1    3600 2300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR050
U 1 1 54888F00
P 3600 2600
F 0 "#PWR050" H 3690 2580 30  0001 C CNN
F 1 "GND" H 3600 2520 30  0001 C CNN
F 2 "~" H 3600 2600 60  0000 C CNN
F 3 "~" H 3600 2600 60  0000 C CNN
	1    3600 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 2600 3600 2500
Wire Wire Line
	4150 2150 4050 2150
Wire Wire Line
	4050 2150 4050 2600
$Comp
L GND #PWR051
U 1 1 548890F9
P 4050 2600
F 0 "#PWR051" H 4140 2580 30  0001 C CNN
F 1 "GND" H 4050 2520 30  0001 C CNN
F 2 "~" H 4050 2600 60  0000 C CNN
F 3 "~" H 4050 2600 60  0000 C CNN
	1    4050 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 1950 4050 1950
Wire Wire Line
	4150 2050 4050 2050
Text Label 4050 1950 2    60   ~ 0
EE_HOLD
Text Label 4050 2050 2    60   ~ 0
EE_WRITE
Text Label 6550 2800 2    60   ~ 0
EE_HOLD
Text Label 6550 2700 2    60   ~ 0
EE_WRITE
$Comp
L IRLML6302 Q1
U 1 1 5489849E
P 11500 1250
F 0 "Q1" H 11700 1300 60  0000 C CNN
F 1 "IRLML6302" H 11900 1150 60  0000 C CNN
F 2 "~" H 11500 1250 60  0000 C CNN
F 3 "~" H 11500 1250 60  0000 C CNN
	1    11500 1250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10950 1200 11300 1200
Wire Wire Line
	11450 1800 11450 1450
Wire Wire Line
	11700 1200 12800 1200
$Comp
L R R11
U 1 1 5489895C
P 11050 1500
F 0 "R11" V 11130 1350 50  0000 C CNN
F 1 "10K" V 11050 1500 50  0000 C CNN
F 2 "RES_0603" V 11130 1600 28  0000 C CNN
F 3 "~" H 11050 1500 60  0000 C CNN
	1    11050 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	11050 1250 11050 1200
Connection ~ 11050 1200
Wire Wire Line
	10850 1800 11450 1800
Wire Wire Line
	11050 1800 11050 1750
Connection ~ 11050 1800
Text Label 10850 1800 2    60   ~ 0
WIFI_PWR
$Comp
L PWR_FLAG #FLG052
U 1 1 54898D30
P 12800 1200
F 0 "#FLG052" H 12800 1470 30  0001 C CNN
F 1 "PWR_FLAG" H 12800 1430 30  0000 C CNN
F 2 "~" H 12800 1200 60  0000 C CNN
F 3 "~" H 12800 1200 60  0000 C CNN
	1    12800 1200
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR053
U 1 1 5489910E
P 3150 4100
F 0 "#PWR053" H 3150 4060 30  0001 C CNN
F 1 "+3.3V" H 3230 4130 30  0000 C CNN
F 2 "~" H 3150 4100 60  0000 C CNN
F 3 "~" H 3150 4100 60  0000 C CNN
	1    3150 4100
	1    0    0    -1  
$EndComp
Text Label 6550 2000 2    60   ~ 0
WIFI_PWR
$Comp
L HOLE_METALLED HOLE1
U 1 1 5492EBC9
P 9800 9950
F 0 "HOLE1" H 9300 9900 60  0000 C CNN
F 1 "HOLE_METALLED" H 9250 10000 60  0000 C CNN
F 2 "" H 9800 9950 60  0000 C CNN
F 3 "" H 9800 9950 60  0000 C CNN
	1    9800 9950
	0    1    1    0   
$EndComp
$Comp
L HOLE_METALLED HOLE2
U 1 1 5492EBD8
P 10050 9950
F 0 "HOLE2" H 9550 9900 60  0000 C CNN
F 1 "HOLE_METALLED" H 9500 10000 60  0000 C CNN
F 2 "" H 10050 9950 60  0000 C CNN
F 3 "" H 10050 9950 60  0000 C CNN
	1    10050 9950
	0    1    1    0   
$EndComp
Wire Wire Line
	9800 10250 9800 10500
Wire Wire Line
	9800 10500 10050 10500
Wire Wire Line
	10050 10250 10050 10600
Connection ~ 10050 10500
$Comp
L GND #PWR054
U 1 1 5492EEE5
P 10050 10600
F 0 "#PWR054" H 10140 10580 30  0001 C CNN
F 1 "GND" H 10050 10520 30  0001 C CNN
F 2 "~" H 10050 10600 60  0000 C CNN
F 3 "~" H 10050 10600 60  0000 C CNN
	1    10050 10600
	1    0    0    -1  
$EndComp
Connection ~ 12500 1200
$Comp
L +3.3V #PWR055
U 1 1 54A2A8A6
P 12150 1700
F 0 "#PWR055" H 12150 1660 30  0001 C CNN
F 1 "+3.3V" H 12230 1730 30  0000 C CNN
F 2 "~" H 12150 1700 60  0000 C CNN
F 3 "~" H 12150 1700 60  0000 C CNN
	1    12150 1700
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR056
U 1 1 54A2A8B5
P 11950 1700
F 0 "#PWR056" H 11950 1660 30  0001 C CNN
F 1 "+3.3V" H 12030 1730 30  0000 C CNN
F 2 "~" H 11950 1700 60  0000 C CNN
F 3 "~" H 11950 1700 60  0000 C CNN
	1    11950 1700
	0    -1   -1   0   
$EndComp
Text Label 11950 1200 0    60   ~ 0
PWR
$EndSCHEMATC
