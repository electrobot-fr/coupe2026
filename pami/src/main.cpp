#include <Arduino.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>



// tirette
#define PIN_START 10      // GPIO 10 utilisé pour la tirette
#define PIN_SERVO1 5      // Servo sur GPIO05
#define PIN_SERVO2 6      // Servo sur GPIO6

// stepper motors : Droite stepperD, gauche stepperG
AccelStepper stepperD(1, 1, 2);  // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepperG(1, 3, 4); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

//  servo
Servo servo1; 

// Variables globales
bool started = false; // Indique si le robot a démarré
bool mouvement_termine = false; // Indique si la séquence de mouvement est terminée

// Paramètres du robot à ajuster 
const float WHEEL_DIAMETER = 60.0;  // Diamètre des roues en mm
const int STEPS_PER_REV = 200;      // Nombre de pas par tour du moteur (200 pour 1.8°, 400 pour 0.9°)
const int MICROSTEPS = 1;            // Microstepping (1, 2, 4, 8, 16, etc.)
const float STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / (PI * WHEEL_DIAMETER);

// Fonction pour avancer d'une distance en mm
void avancer(float distance_mm) {
    long steps = (long)(distance_mm * STEPS_PER_MM);
    
    Serial.print("Avance de ");
    Serial.print(distance_mm);
    Serial.print(" mm (");
    Serial.print(steps);
    Serial.println(" pas)");
    
    stepperD.move(steps);
    stepperG.move(steps);
    
    // Attendre que les deux moteurs atteignent leur position
    while (stepperD.distanceToGo() != 0 || stepperG.distanceToGo() != 0) {
        stepperD.run();
        stepperG.run();
    }
    
    Serial.println("Distance atteinte ! Arrêt des moteurs.");
}

// Fonction pour tourner à droite (roue droite recule, roue gauche avance)
void tourner_droite(float distance_mm) {
    long steps = (long)(distance_mm * STEPS_PER_MM);
    
    Serial.print("Tourne à droite de ");
    Serial.print(distance_mm);
    Serial.print(" mm (");
    Serial.print(steps);
    Serial.println(" pas)");
    
    stepperD.move(-steps);  // Roue droite recule
    stepperG.move(steps);   // Roue gauche avance
    
    // Attendre que les deux moteurs atteignent leur position
    while (stepperD.distanceToGo() != 0 || stepperG.distanceToGo() != 0) {
        stepperD.run();
        stepperG.run();
    }
    
    Serial.println("Rotation à droite terminée !");
}

// Fonction pour tourner à gauche (roue droite avance, roue gauche recule)
void tourner_gauche(float distance_mm) {
    long steps = (long)(distance_mm * STEPS_PER_MM);
    
    Serial.print("Tourne à gauche de ");
    Serial.print(distance_mm);
    Serial.print(" mm (");
    Serial.print(steps);
    Serial.println(" pas)");
    
    stepperD.move(steps);   // Roue droite avance
    stepperG.move(-steps);  // Roue gauche recule
    
    // Attendre que les deux moteurs atteignent leur position
    while (stepperD.distanceToGo() != 0 || stepperG.distanceToGo() != 0) {
        stepperD.run();
        stepperG.run();
    }
    
    Serial.println("Rotation à gauche terminée !");
}

void setup() {
    Serial.begin(115200);

    // Tirette
    pinMode(PIN_START, INPUT_PULLUP);  

    servo1.attach(PIN_SERVO1);
    servo1.write(0);  // Position neutre initiale


   // Steppers
   int speed = 3000;
   pinMode(0, OUTPUT);
   digitalWrite(0, LOW);
   stepperD.setMaxSpeed(speed);
   stepperD.setSpeed(speed);
   stepperD.setAcceleration(1000);  // Accélération pour la fonction avancer()
   stepperG.setMaxSpeed(speed);
   stepperG.setSpeed(speed);
   stepperG.setAcceleration(1000);  // Accélération pour la fonction avancer()

}

void loop() {
    int startSignal = digitalRead(PIN_START);

    if (!started) {
        if (startSignal == HIGH) {
            Serial.println("Démarrage ...");
            started = true;
        } else {
            Serial.println("En attente de la tirette...");
            delay(300);
            return;   // Tant que la tirette n’est pas retirée, on ne fait rien
        }
    }

    // Les steppers - exécuté une seule fois
    if (!mouvement_termine) {
        Serial.println("Démarrage des Stepper !");
        
        // Séquence de mouvements
        avancer(1000);        // Avance de 1000 mm
        delay(500);
        
        tourner_droite(500);  // Tourne à droite de 500 mm
        delay(500);
        
        tourner_gauche(400);  // Tourne à gauche de 400 mm
        delay(500);
        
        mouvement_termine = true;
        Serial.println("Séquence de mouvements terminée. Activation du servo en continu.");
    }
    
    // Une fois le mouvement terminé, le servo fonctionne en permanence
    if (mouvement_termine) {
        servo1.write(180);
        delay(500);  
        servo1.write(0);  
        delay(500);
    }

}


// Ecran OLED avec décompte et lettre à gauche

// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 32
// #define OLED_ADDR 0x3C

// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// // Lettre à afficher à gauche
// char lettreGauche = 'J'; // peut être 'J' ou 'B'

// void setup() {
//   Serial.begin(115200);
//   delay(100);

//   Wire.begin(8, 9); // SDA=8, SCL=9

//   if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
//     Serial.println(F("Erreur : OLED non détecté"));
//     for (;;);
//   }

//   display.clearDisplay();
//   display.display();
// }

// void loop() {
//   for (int i = 85; i >= 0; i--) {
//     display.clearDisplay();
    
//     // === Afficher la lettre à gauche ===
//     display.setTextColor(SSD1306_WHITE);
//     display.setTextSize(4);  
//     int16_t lettreX = 2;
//     int16_t lettreY = (SCREEN_HEIGHT - 8*4)/2; // centré verticalement
//     display.setCursor(lettreX, lettreY);
//     display.print(lettreGauche);

//     // === Définir taille du texte du compteur selon nombre de chiffres ===
//     int textSize = 4;
//     if (i >= 100) textSize = 3;
//     else if (i >= 10) textSize = 4;
//     else textSize = 5;

//     display.setTextSize(textSize);

//     // Calcul largeur du texte
//     int nbChiffres = (i < 10 ? 1 : (i < 100 ? 2 : 3));
//     int largeurTexte = 6 * textSize * nbChiffres;
//     int hauteurTexte = 8 * textSize;

//     // Centrage horizontal du compteur avec espace pour la lettre
//     int16_t espaceLettre = 20; // espace réservé pour la lettre à gauche
//     int16_t compteurX = (SCREEN_WIDTH - largeurTexte + espaceLettre)/2 + espaceLettre;
//     int16_t compteurY = (SCREEN_HEIGHT - hauteurTexte)/2;

//     display.setCursor(compteurX, compteurY);
//     display.print(i);

//     display.display();

//     Serial.print("Décompte : ");
//     Serial.println(i);
//     delay(1000);
//   }

//   // Fin du décompte
//   display.clearDisplay();
//   display.setTextSize(2);
//   display.setCursor(10, 10);
//   display.print("TERMINE !");
//   display.display();

//   while (true); // stop
// }

