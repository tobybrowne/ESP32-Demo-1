#include <Arduino.h>

#define B_PIN 5  // Change this if necessary
#define R_PIN 3  // Change this if necessary
#define G_PIN 4  // Change this if necessary

void setup() {
  Serial.begin(9600);
  pinMode(R_PIN, OUTPUT);  // Set LED pin as output
  pinMode(G_PIN, OUTPUT);  // Set LED pin as output
  pinMode(B_PIN, OUTPUT);  // Set LED pin as outputsasd
}

void loop() {
  Serial.println("hello");
  digitalWrite(R_PIN, HIGH); // Turn LED on
  delay(500); // Wait 500ms
  Serial.println("hello");
  digitalWrite(R_PIN, LOW); // Turn LED on
  digitalWrite(G_PIN, HIGH);  // Turn LED off
  Serial.println("hello");
  delay(500); // Wait 500ms
  digitalWrite(G_PIN, LOW);  // Turn LED off
  digitalWrite(B_PIN, HIGH);  // Turn LED off
  Serial.println("hello");
  delay(500); // Wait 500ms
  digitalWrite(B_PIN, LOW);  // Turn LED off
}