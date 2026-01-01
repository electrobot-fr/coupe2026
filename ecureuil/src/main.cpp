#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <SerialTransfer.h>
#include "Wire.h"
#include <ESP32Servo.h>
#include <AccelStepper.h>

#undef LED_BUILTIN
#define LED_BUILTIN 8

#define BUFFER_SIZE 250 // max of 250 bytes
uint8_t buf_recv[BUFFER_SIZE];

struct __attribute__((packed)) STRUCT
{
    int16_t x;
    int16_t y;
    int16_t z;
    bool cmdGliss;         // Glissière : 0: retracter / 1: deployer
    bool cmdAimantInt;     // Pince aimant interieur 0: détacher / 1: attacher
    bool cmdAimantExt;     // Pince aimant exterieur
    bool cmdPompe;         // Commande Pompe : 0: Off / 1: On
    bool cmdVanne;         // Commande Electrovanne : 0: Off / 1: On
    bool cmdServoPlanche;  // Lever les planches
    bool cmdServoBanniere; // Lacher la banniere
    int16_t ascPlanche;    // Position de l'ascenceur des planches
    int16_t ascBoites;     // Position de l'ascenceur des boites
    uint8_t compteur;      // Compteur
} message;

//  servo
Servo servo1;

AccelStepper stepperD(1, 1, 2); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepperG(1, 3, 4); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

#define STX 0x7E
#define ETX 0x81
#define HEADER_SIZE 4 // start + packetID + COBS + length
#define FOOTER_SIZE 2 // CRC + stop

#define PIN_SERVO1 5 // Servo sur GPIO05

// Paramètres du robot à ajuster
const float WHEEL_DIAMETER = 60.0; // Diamètre des roues en mm
const int STEPS_PER_REV = 200;     // Nombre de pas par tour du moteur (200 pour 1.8°, 400 pour 0.9°)
const int MICROSTEPS = 1;          // Microstepping (1, 2, 4, 8, 16, etc.)
const float STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / (PI * WHEEL_DIAMETER);

// Fonction pour avancer d'une distance en mm
void avancer(float distance_mm)
{
    long steps = (long)(distance_mm * STEPS_PER_MM);

    Serial.print("Avance de ");
    Serial.print(distance_mm);
    Serial.print(" mm (");
    Serial.print(steps);
    Serial.println(" pas)");

    stepperD.move(steps);
    stepperG.move(steps);

    // Attendre que les deux moteurs atteignent leur position
    while (stepperD.distanceToGo() != 0 || stepperG.distanceToGo() != 0)
    {
        stepperD.run();
        stepperG.run();
    }
}

// Fonction pour tourner à droite (roue droite recule, roue gauche avance)
void tourner_droite(float distance_mm)
{
    long steps = (long)(distance_mm * STEPS_PER_MM);

    Serial.print("Tourne à droite de ");
    Serial.print(distance_mm);
    Serial.print(" mm (");
    Serial.print(steps);
    Serial.println(" pas)");

    stepperD.move(-steps); // Roue droite recule
    stepperG.move(steps);  // Roue gauche avance

    // Attendre que les deux moteurs atteignent leur position
    while (stepperD.distanceToGo() != 0 || stepperG.distanceToGo() != 0)
    {
        stepperD.run();
        stepperG.run();
    }

    Serial.println("Rotation à droite terminée !");
} // Fonction pour tourner à gauche (roue droite avance, roue gauche recule)
void tourner_gauche(float distance_mm)
{
    long steps = (long)(distance_mm * STEPS_PER_MM);

    Serial.print("Tourne à gauche de ");
    Serial.print(distance_mm);
    Serial.print(" mm (");
    Serial.print(steps);
    Serial.println(" pas)");

    stepperD.move(steps);  // Roue droite avance
    stepperG.move(-steps); // Roue gauche recule

    // Attendre que les deux moteurs atteignent leur position
    while (stepperD.distanceToGo() != 0 || stepperG.distanceToGo() != 0)
    {
        stepperD.run();
        stepperG.run();
    }

    Serial.println("Rotation à gauche terminée !");
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *macAddr, const uint8_t *incomingData, int len)
{
    if (incomingData[0] != STX)
    {
        Serial.println("Invalid start byte");
        return;
    }

    if (incomingData[len - 1] != ETX)
    {
        Serial.println("Invalid stop byte");
        return;
    }

    uint8_t payloadLength = incomingData[3];

    if (payloadLength != sizeof(STRUCT))
    {
        Serial.print("Payload size mismatch: ");
        Serial.println(payloadLength);
        return;
    }

    // Copy ONLY the payload
    memcpy(&message, &incomingData[HEADER_SIZE], sizeof(STRUCT));

    digitalWrite(LED_BUILTIN, LOW);

    int a = map(message.x, 0, 1023, -3000, 3000);
    int b = map(message.y, 0, 1023, -3000, 3000);

    if (abs(a) < 200 && abs(b) < 200) {
        stepperG.setSpeed(0);
        stepperD.setSpeed(0);
    } else if (abs(a) > abs(b)) {
        stepperG.setSpeed(-a);
        stepperD.setSpeed(-a);
    } else {
        stepperG.setSpeed(-b);
        stepperD.setSpeed(b);
    }

    servo1.write(90 + map(message.z, 0, 1023, 60, -60));

    digitalWrite(LED_BUILTIN, HIGH);
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    servo1.attach(PIN_SERVO1);
    servo1.write(90);

    int speed = 3000;
    pinMode(0, OUTPUT);
    digitalWrite(0, LOW);
    stepperD.setMaxSpeed(speed);
    stepperD.setSpeed(0);
    stepperD.setAcceleration(1000); // Accélération pour la fonction avancer()
    stepperG.setMaxSpeed(speed);
    stepperG.setSpeed(0);
    stepperG.setAcceleration(1000); // Accélération pour la fonction avancer()

    // Initialize Serial Monitor
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    esp_wifi_set_max_tx_power(40);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop()
{
    stepperD.runSpeed();
    stepperG.runSpeed();
}
