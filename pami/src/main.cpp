#include <Arduino.h>

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
#define DECOMPTE_INITIAL 87 // secondes

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

#define PAMI1

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
#ifdef PAMI1
        if (equipe == BLEU) {
            // 10 000 = 123cm, PAMI 1: 6097, PAMI 2: 13089, PAMI 3: 9593
            avancer(2500);
            delay(10);
                tourner_droite(170);
            avancer(8000);
            delay(10);
                tourner_droite(230);
            avancer(7000);
        } else {
            avancer(2500);
            delay(10);
                tourner_gauche(170);
            avancer(8000);
            delay(10);
                tourner_gauche(230);
            avancer(7000);
        }
#endif
#ifdef PAMI2
        if (equipe == BLEU) {
            // 10 000 = 123cm, PAMI 1: 6097, PAMI 2: 13089, PAMI 3: 9593
            delay(3000);
            avancer(6000);
            delay(10);
                tourner_gauche(420);
            avancer(5000);
        } else {
            delay(3000);
            avancer(6000);
            delay(10);
                tourner_droite(420);
            avancer(5000);
        }
#endif
#ifdef PAMI3
        if (equipe == BLEU) {
            // 10 000 = 123cm, PAMI 1: 6097, PAMI 2: 13089, PAMI 3: 9593
            avancer(3000);
            delay(10);
                tourner_gauche(190);
            avancer(9512);
        } else {
            avancer(3000);
            delay(10);
                tourner_droite(190);
            avancer(10000);
        }
#endif
#ifdef PAMI4
        if (equipe == BLEU) {
            // 10 000 = 123cm, PAMI 1: 6097, PAMI 2: 13089, PAMI 3: 9593
            delay(5000);
            avancer(8000);
        } else {
            delay(5000);
            avancer(8000);
        }
#endif


        mouvement_termine = true;
    }

    // Oscillation servo en continu
    servo1.write(180);
    delay(500);
    servo1.write(0);
    delay(500);
}
