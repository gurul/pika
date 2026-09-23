// TF-Luna bench reader for Mega 2560. No car or sensor commands.
// Sensor 1 -> 5V, 3(TX) -> 19(RX1), 4 -> GND. Sensor 2,5,6 OPEN.
// Do not connect Mega TX1(18) to the 3.3 V sensor RX.
#include <Arduino.h>
#include "TfLunaParser.h"

TfLunaParser parser;
uint32_t reportAt = 0, statusAt = 0, reportedFrames = 0;

void setup() {
  // Configure UART1 for RX only, without ever enabling the TX pin or a 5 V
  // input pull-up. Arduino's Serial1 ISR supplies its usual receive buffer.
  pinMode(18, INPUT); digitalWrite(18, LOW);
  pinMode(19, INPUT); digitalWrite(19, LOW);
  UCSR1B = 0;
  UCSR1A = _BV(U2X1);
  const uint16_t divider = (F_CPU / 4 / 115200UL - 1) / 2;
  UBRR1H = uint8_t(divider >> 8); UBRR1L = uint8_t(divider);
  UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);
  UCSR1B = _BV(RXEN1) | _BV(RXCIE1);
  Serial.begin(115200);
  Serial.println(F("{\"type\":\"boot\",\"firmware\":\"mega_lidar_v1\",\"sensor_rx_only\":true,\"units\":\"factory_cm_assumed\"}"));
}

void loop() {
  while (Serial1.available()) parser.feed(uint8_t(Serial1.read()), millis());
  uint32_t now = millis();
  // Read every sensor frame; limit USB display to 20 Hz to avoid backpressure.
  if (parser.frames != reportedFrames && uint32_t(now - reportAt) >= 50) {
    reportAt = now; reportedFrames = parser.frames;
    const TfLunaSample &s = parser.last;
    Serial.print(F("{\"type\":\"sample\",\"seq\":")); Serial.print(parser.frames);
    Serial.print(F(",\"sample_ms\":")); Serial.print(s.stamp_ms);
    Serial.print(F(",\"distance_raw\":")); Serial.print(s.distance_raw);
    Serial.print(F(",\"amp\":")); Serial.print(s.strength);
    Serial.print(F(",\"temp_c\":")); Serial.print(s.temperature_raw / 8.0 - 256.0, 2);
    Serial.print(F(",\"usable_default_cm\":")); Serial.print(s.usableDefaultCm() ? F("true") : F("false"));
    Serial.print(F(",\"status\":\"")); Serial.print(s.status());
    Serial.print(F("\",\"bad_crc\":")); Serial.print(parser.bad_crc); Serial.println('}');
  }
  if (uint32_t(now - statusAt) >= 1000) {
    statusAt = now;
    Serial.print(F("{\"type\":\"status\",\"ms\":")); Serial.print(now);
    Serial.print(F(",\"bytes\":")); Serial.print(parser.bytes);
    Serial.print(F(",\"frames\":")); Serial.print(parser.frames);
    Serial.print(F(",\"bad_crc\":")); Serial.print(parser.bad_crc);
    Serial.print(F(",\"age_ms\":"));
    if (parser.frames) Serial.print(uint32_t(now - parser.last.stamp_ms));
    else Serial.print(F("null"));
    Serial.println('}');
  }
}
