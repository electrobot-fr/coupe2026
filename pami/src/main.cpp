#include <Arduino.h>

// PAMI variant selection — set via Makefile build flag (-DPAMI_VARIANT=N)
#define NINJA 0
#define PAMI1 1
#define PAMI2 2
#define PAMI3 3
#define PAMI4 4

#ifndef PAMI_VARIANT
#error "PAMI_VARIANT not set. Build via Makefile target: pami-ninja, pami-1, pami-2, pami-3, pami-4"
#endif

// Servo
#include <ESP32Servo.h>

// Steppers
#include <AccelStepper.h>

// I2C & OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// tirette
#define PIN_START 10      // GPIO 10 utilisé pour la tirette

// Servos
#define PIN_SERVO1 6      // Servo sur GPIO6
Servo servo1;

// stepper motors : Droite stepperD, gauche stepperG
AccelStepper stepperG(1, 1, 2);  // DRIVER mode (step, dir)
AccelStepper stepperD(1, 3, 4);  // DRIVER mode (step, dir)
#define SPEED 6000
#define ACCELERATION 2000

// Variables globales
bool started = false;
bool decompte_termine = false;
bool mouvement_termine = false;

// Equipe
#define PIN_EQUIPE 5      // GPIO05 (connecteur servo)
#define JAUNE 0
#define BLEU 1
int equipe = BLEU;
char lettreGauche = 'B';

// Valeur decompte initiale
#define DECOMPTE_INITIAL 5 // secondes

// Paramètres du robot
const float WHEEL_DIAMETER = 60.0;  // Diamètre des roues en mm
const int STEPS_PER_REV = 200;      // Nombre de pas par tour du moteur (200 pour 1.8°)
const int MICROSTEPS = 1;
const float STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / (PI * WHEEL_DIAMETER);

// Attendre que les deux moteurs atteignent leur position
void attendre_moteurs() {
    while (stepperD.distanceToGo() != 0 || stepperG.distanceToGo() != 0) {
        stepperD.run();
        stepperG.run();
    }
}

// Déplacer les deux moteurs d'un nombre de steps chacun
void deplacer(long stepsD, long stepsG) {
    stepperD.move(stepsD);
    stepperG.move(stepsG);
    attendre_moteurs();
}

void avancer(float distance_mm) {
    long steps = (long)(distance_mm * STEPS_PER_MM);
    deplacer(steps, -steps);
}

void tourner_droite(float distance_mm) {
    long steps = (long)(distance_mm * STEPS_PER_MM);
    deplacer(-steps, -steps);
}

void tourner_gauche(float distance_mm) {
    long steps = (long)(distance_mm * STEPS_PER_MM);
    deplacer(steps, steps);
}

// Afficher l'écran d'attente tirette
void afficher_attente() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(4);
    display.setCursor(2, 0);
    display.print(lettreGauche);

    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print("Tirette en place !");

    display.display();
}

// Fonction de décompte avec affichage OLED
void decompte(int decompte_initial = DECOMPTE_INITIAL) {
    for (int i = decompte_initial; i >= 0; i--) {
        display.clearDisplay();

        // Afficher la lettre à gauche
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(4);
        display.setCursor(2, (SCREEN_HEIGHT - 8*4)/2);
        display.print(lettreGauche);

        // Taille du texte du compteur selon nombre de chiffres
        int textSize = (i >= 100) ? 3 : (i >= 10) ? 4 : 5;
        display.setTextSize(textSize);

        int nbChiffres = (i < 10 ? 1 : (i < 100 ? 2 : 3));
        int largeurTexte = 6 * textSize * nbChiffres;
        int hauteurTexte = 8 * textSize;

        int16_t espaceLettre = 20;
        int16_t compteurX = (SCREEN_WIDTH - largeurTexte + espaceLettre)/2 + espaceLettre;
        int16_t compteurY = (SCREEN_HEIGHT - hauteurTexte)/2;

        display.setCursor(compteurX, compteurY);
        display.print(i);
        display.display();

        Serial.print("Décompte : ");
        Serial.println(i);
        delay(1000);
    }

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(10, 10);
    display.print("GO !");
    display.display();

    Serial.println("Décompte terminé ! Début des mouvements.");
    decompte_termine = true;
}

void setup() {
    Serial.begin(115200);

    // Tirette
    pinMode(PIN_START, INPUT_PULLUP);

    // Equipe
    pinMode(PIN_EQUIPE, INPUT_PULLUP);

    // Servo
    servo1.attach(PIN_SERVO1);
    servo1.write(0);

   // Steppers
   pinMode(0, OUTPUT);
   digitalWrite(0, LOW);
   stepperD.setMaxSpeed(SPEED);
   stepperD.setSpeed(SPEED);
   stepperD.setAcceleration(ACCELERATION);
   stepperG.setMaxSpeed(SPEED);
   stepperG.setSpeed(SPEED);
   stepperG.setAcceleration(ACCELERATION);

   // Display OLED
   Wire.begin(8, 9); // SDA=8, SCL=9
   if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
     Serial.println(F("Erreur : OLED non détecté"));
     for (;;);
   }

   afficher_attente();
}

void loop() {

    if (!started) {
        // Lire l'équipe tant qu'on n'a pas démarré
        if (digitalRead(PIN_EQUIPE) == LOW) {
            equipe = JAUNE;
            lettreGauche = 'J';
        } else {
            equipe = BLEU;
            lettreGauche = 'B';
        }

        if (digitalRead(PIN_START) == LOW) {
            started = true;
        } else {
            afficher_attente();
            Serial.println("En attente de la tirette...");
            delay(300);
            return;
        }
    }

    // Décompte après retrait de la tirette
    if (!decompte_termine) {
        decompte(DECOMPTE_INITIAL);
        return;
    }

    // Séquence de mouvements (exécutée une seule fois)
    if (!mouvement_termine) {

#if PAMI_VARIANT == NINJA
        // 10 000 = 123cm
        auto tourner_ext = (equipe == BLEU) ? tourner_gauche : tourner_droite;
        auto tourner_int = (equipe == BLEU) ? tourner_droite : tourner_gauche;
        avancer(520);
        tourner_ext(515);
        stepperG.setMaxSpeed(1.5 * SPEED);
        stepperD.setMaxSpeed(1.5 * SPEED);
        avancer(2800);
        stepperG.setMaxSpeed(SPEED / 10);
        stepperD.setMaxSpeed(SPEED / 10);
        avancer(-3200);
        stepperG.setMaxSpeed(1.5 * SPEED);
        stepperD.setMaxSpeed(1.5 * SPEED);
        avancer(2800);
        stepperG.setMaxSpeed(SPEED / 10);
        stepperD.setMaxSpeed(SPEED / 10);
        avancer(-3200);
        stepperG.setMaxSpeed(1.5 * SPEED);
        stepperD.setMaxSpeed(1.5 * SPEED);
        avancer(2800);
        stepperG.setMaxSpeed(SPEED / 10);
        stepperD.setMaxSpeed(SPEED / 10);
        avancer(-3500);
        avancer(175);
        tourner_int(515);
        avancer(-1700);
        avancer(3200);
        tourner_ext(515);
        avancer(2550);
        avancer(-3000);
        avancer(2550);
        avancer(-3000);
        avancer(1200);
        tourner_int(515);
        avancer(2300);
        avancer(-1200);
        tourner_ext(515);
        avancer(1600);
#endif
#if PAMI_VARIANT == PAMI1
        // PAMI 1: 6097. Premier virage asymétrique : 175 (BLEU) vs 155 (JAUNE).
        auto tourner_ext = (equipe == BLEU) ? tourner_droite : tourner_gauche;
        avancer(2400);
        delay(10);
        tourner_ext((equipe == BLEU) ? 175 : 155);
        avancer(8000);
        delay(10);
        tourner_ext(240);
        avancer(6470);
#endif
#if PAMI_VARIANT == PAMI2
        // PAMI 2: 13089. Attention : inversion dans le câblage Bleu/Jaune (sens des virages inversé)
        auto tourner_ext = (equipe == BLEU) ? tourner_gauche : tourner_droite;
        delay(6000);
        avancer(5000);
        delay(10);
        tourner_ext(330);
        avancer(4700);
#endif
#if PAMI_VARIANT == PAMI3
        // PAMI 3: 9593
        auto tourner_ext = (equipe == BLEU) ? tourner_gauche : tourner_droite;
        delay(3000);
        avancer(1500);
        delay(10);
        tourner_ext(150);
        avancer(11500);
#endif
#if PAMI_VARIANT == PAMI4
        // PAMI 4 : ligne droite, identique BLEU/JAUNE
        delay(8500);
        avancer(6900);
#endif

        mouvement_termine = true;
    }

    // Oscillation servo en continu
    servo1.write(180);
    delay(500);
    servo1.write(0);
    delay(500);
}
