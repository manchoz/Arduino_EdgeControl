/*
  This file is part of the Arduino Outdoor Carrier library.
  Copyright (C) 2017  Arduino AG (http://www.arduino.cc/)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "OutdoorCarrier_Power.h"

void OutdoorCarrier_PowerClass::begin()
{
    pinMode(GATED_VBAT_ENABLE, OUTPUT);
    pinMode(GATED_3V3_ENABLE_N, OUTPUT);
    pinMode(GATED_19V_ENABLE, OUTPUT);
    pinMode(ON_MKR1, OUTPUT);
    pinMode(ON_MKR2, OUTPUT);

    pinMode(VBAT_PROBE, INPUT);

    for (auto status : _statuses)
        status = false;
}

void OutdoorCarrier_PowerClass::set(const PowerRail rail, const bool status)
{
    switch (rail) {
    case PWR_VBAT: {
        digitalWrite(GATED_VBAT_ENABLE, status ? HIGH : LOW);
        _statuses[PWR_VBAT] = status;
        break;
    }
    case PWR_3V3: {
        digitalWrite(GATED_3V3_ENABLE_N, status ? LOW : HIGH);
        _statuses[PWR_3V3] = status;
        break;
    }
    case PWR_19V: {
        if(!_statuses[PWR_VBAT] && status)
            digitalWrite(GATED_VBAT_ENABLE, HIGH);
        digitalWrite(GATED_19V_ENABLE, status ? HIGH : LOW);
        _statuses[PWR_19V] = status;
        break;
    }
    case PWR_MKR1: {
        digitalWrite(ON_MKR1, status ? HIGH : LOW);
        _statuses[PWR_MKR1] = status;
        break;
    }
    case PWR_MKR2: {
        digitalWrite(ON_MKR2, status ? HIGH : LOW);
        _statuses[PWR_MKR2] = status;
        break;
    }
    default:
        break;
    }
}

float OutdoorCarrier_PowerClass::getVBat(const int adcResolution) const
{
    constexpr float vbatVD { 100.0f / (100.0f + 475.0f) }; // Voltage Divider on VBAT_PROBE
    float valToV { 3.3f / float { (1 << adcResolution) - 1 } };
    constexpr unsigned int count { 100 };

    int val { 0 };
    for (auto i = 0; i < count; i++) {
        val += analogRead(VBAT_PROBE);
        delay(10);
    }

    // float vbat = val * valToV / count / 1.2f / vbatVD;
    auto vbat = (float { val / count } * valToV) / vbatVD;

    return vbat;
}

OutdoorCarrier_PowerClass Power {};
