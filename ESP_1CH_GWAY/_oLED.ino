// 1-channel LoRa Gateway for ESP8266 and ESP32
// Copyright (c) 2016-2021 Maarten Westenberg version for ESP8266
//
// 	based on work done by Thomas Telkamp for Raspberry PI 1ch gateway
//	and many others.
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the MIT License
// which accompanies this distribution, and is available at
// https://opensource.org/licenses/mit-license.php
//
// NO WARRANTY OF ANY KIND IS PROVIDED
//
// Author: Maarten Westenberg (mw12554@hotmail.com)
//
// This file contains the state machine code enabling to receive
// and transmit packages/messages.
// ========================================================================================
//
// --------------------------------------------------------------------	
// Initilize the Oled functions.
// This function will init the Oled screen. Depending on the 
// availability of the reset button it will reset the display first.
// --------------------------------------------------------------------
#include <Wire.h>

#if defined(USE_HELTEC_DISPLAY)
#define GW_DISPLAY Heltec.display
#else
#include "OLEDDisplayFonts.h"
#define GW_DISPLAY (&display)
#endif

void init_oLED() 
{
#if _OLED>=1
#if !defined(USE_HELTEC_DISPLAY)
#if defined(OLED_SDA) && defined(OLED_SCL)
	Wire.begin(OLED_SDA, OLED_SCL);
#endif
#if defined OLED_RST
	pinMode(OLED_RST,OUTPUT);
	digitalWrite(OLED_RST, LOW); 	// low to reset Oled
	delay(100); 
	digitalWrite(OLED_RST, HIGH); 	// must be high to turn on Oled
	delay(50);
#endif
	display.init();
	delay(100);
#endif

	auto *d = GW_DISPLAY;
	d->clear();
	d->flipScreenVertically();
	d->setFont(ArialMT_Plain_16);
	d->setTextAlignment(TEXT_ALIGN_LEFT);
	d->drawString(0, 0, "STARTING");
	d->display();
#endif
}

// --------------------------------------------------------------------
// Activate the Oled. Always print the same info.
// These are 4 fields:
// SSID, IP, ID, 
//
// --------------------------------------------------------------------
void acti_oLED() 
{
#if _OLED>=1
	auto *d = GW_DISPLAY;
	d->clear();
	d->setFont(ArialMT_Plain_10);
	d->drawString(0, 0, "READY SSID=");
	d->drawString(0, 12, WiFi.SSID());
	d->drawString(0, 24, "IP=");
	d->drawString(0, 36, WiFi.localIP().toString().c_str());
	d->display();

#endif //_OLED
	delay(4000);
}

// --------------------------------------------------------------------
// Print a message on the Oled.
// Note: The whole message must fit in the buffer
//
// --------------------------------------------------------------------
void msg_oLED(String mesg) 
{
#if _OLED>=1
    auto *d = GW_DISPLAY;
    d->clear();
    d->setFont(ArialMT_Plain_16);
    d->setTextAlignment(TEXT_ALIGN_LEFT);
    d->drawString(0, 16, String(mesg));
    d->display();
	yield();
#endif //_OLED
}

// Print a larger Oled message consisting of two strings

void msg_lLED(String mesg, String mesg2) 
{
#if _OLED>=1
    auto *d = GW_DISPLAY;
    d->clear();
    d->setFont(ArialMT_Plain_10);
    d->setTextAlignment(TEXT_ALIGN_LEFT);
    d->drawString(0, 0, String(mesg));
    d->drawString(0, 20, String(mesg2));
    d->display();
	yield();
#endif //_OLED
}

// --------------------------------------------------------------------
// Print the Oled address in use
//
// --------------------------------------------------------------------
void addr_oLED() 
{
#if _OLED>=1
	#if _DUSB>=1
		Serial.print(F("OLED_ADDR=0x"));
		Serial.println(OLED_ADDR, HEX);
	#endif //_DUSB
#endif
}
