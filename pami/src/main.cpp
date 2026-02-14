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

// Lettre à afficher à gauche
char lettreGauche = 'J'; // peut être 'J' ou 'B'


// tirette
#define PIN_START 10      // GPIO 10 utilisé pour la tirette

// Servos
#define PIN_SERVO1 6      // Servo sur GPIO6
Servo servo1; 

// stepper motors : Droite stepperD, gauche stepperG
AccelStepper stepperG(1, 1, 2);  // Defaults to AccelStepper::FULL4WIRE (4 pins) 
AccelStepper stepperD(1, 3, 4); // Defaults to AccelStepper::FULL4WIRE (4 pins)
#define SPEED 6000
#define ACCELERATION 2000

// Variables globales
bool started = false; // Indique si le robot a démarré
bool decompte_termine = false; // Indique si le décompte est terminé
bool mouvement_termine = false; // Indique si la séquence de mouvement est terminée


// Equipe
#define PIN_EQUIPE 5      // GPIO05 (connecteur servo)
#define JAUNE 0
#define BLEU 1


// Valeur decompte initiale
#define DECOMPTE_INITIAL 5 // 85 secondes

// Paramètres du robot à ajuster 
const float WHEEL_DIAMETER = 60.0;  // Diamètre des roues en mm
const int STEPS_PER_REV = 200;      // Nombre de pas par tour du moteur (200 pour 1.8°, 400 pour 0.9°)
const int MICROSTEPS = 1;            // Microstepping (1, 2, 4, 8, 16, etc.)
const float STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / (PI * WHEEL_DIAMETER);

// Fonction pour avancer d'une distance en mm
void avancer(float distance_mm) {
    long steps = (long)(distance_mm * STEPS_PER_MM);    
    stepperD.move(steps);
    stepperG.move(-steps);
    
    // Attendre que les deux moteurs atteignent leur position
    while (stepperD.distanceToGo() != 0 || stepperG.distanceToGo() != 0) {
        stepperD.run();
        stepperG.run();
    }
}

// Fonction pour tourner à droite (roue droite recule, roue gauche avance)
void tourner_droite(float distance_mm) {
    long steps = (long)(distance_mm * STEPS_PER_MM);   
    stepperD.move(-steps);  // Roue droite recule
    stepperG.move(-steps);   // Roue gauche avance
    
    // Attendre que les deux moteurs atteignent leur position
    while (stepperD.distanceToGo() != 0 || stepperG.distanceToGo() != 0) {
        stepperD.run();
        stepperG.run();
    }    
}

// Fonction pour tourner à gauche (roue droite avance, roue gauche recule)
void tourner_gauche(float distance_mm) {
    long steps = (long)(distance_mm * STEPS_PER_MM);
    stepperD.move(steps);   // Roue droite avance
    stepperG.move(steps);   // Roue gauche recule
    
    // Attendre que les deux moteurs atteignent leur position
    while (stepperD.distanceToGo() != 0 || stepperG.distanceToGo() != 0) {
        stepperD.run();
        stepperG.run();
    }
}

// Fonction de décompte de 85 secondes avec affichage OLED
void decompte(int decompte_initial = DECOMPTE_INITIAL) {
    for (int i = decompte_initial; i >= 0; i--) {
        display.clearDisplay();
        
        // === Afficher la lettre à gauche ===
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(4);  
        int16_t lettreX = 2;
        int16_t lettreY = (SCREEN_HEIGHT - 8*4)/2; // centré verticalement
        display.setCursor(lettreX, lettreY);
        display.print(lettreGauche);

        // === Définir taille du texte du compteur selon nombre de chiffres ===
        int textSize = 4;
        if (i >= 100) textSize = 3;
        else if (i >= 10) textSize = 4;
        else textSize = 5;

        display.setTextSize(textSize);

        // Calcul largeur du texte
        int nbChiffres = (i < 10 ? 1 : (i < 100 ? 2 : 3));
        int largeurTexte = 6 * textSize * nbChiffres;
        int hauteurTexte = 8 * textSize;

        // Centrage horizontal du compteur avec espace pour la lettre
        int16_t espaceLettre = 20; // espace réservé pour la lettre à gauche
        int16_t compteurX = (SCREEN_WIDTH - largeurTexte + espaceLettre)/2 + espaceLettre;
        int16_t compteurY = (SCREEN_HEIGHT - hauteurTexte)/2;

        display.setCursor(compteurX, compteurY);
        display.print(i);

        display.display();

        Serial.print("Décompte : ");
        Serial.println(i);
        delay(1000);
    }

    // Fin du décompte
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
    pinMode(PIN_START, INPUT_PULLUP); // Utiliser une résistance de pull-up interne
    
    // Equipe
    pinMode(PIN_EQUIPE, INPUT_PULLUP); // Utiliser une résistance de pull-up interne
    // par défaut à 1 donc équipe Bleu

    // Servo
    servo1.attach(PIN_SERVO1);
    servo1.write(0);  // Position neutre initiale


   // Steppers
   int speed = SPEED;
   int acceleration = ACCELERATION;
   pinMode(0, OUTPUT);
   digitalWrite(0, LOW);
   stepperD.setMaxSpeed(speed);
   stepperD.setSpeed(speed);
   stepperD.setAcceleration(acceleration);  // Accélération pour la fonction avancer()
   stepperG.setMaxSpeed(speed);
   stepperG.setSpeed(speed);
   stepperG.setAcceleration(acceleration);  // Accélération pour la fonction avancer()

   // Display OLED
   Wire.begin(8, 9); // SDA=8, SCL=9
   if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
     Serial.println(F("Erreur : OLED non détecté"));
     for (;;);
   }
   
   // Affichage initial : lettre et message d'attente
   display.clearDisplay();
   display.setTextColor(SSD1306_WHITE);
   
   // Afficher la lettre en haut à gauche
   display.setTextSize(4);
   display.setCursor(2, 0);
   display.print(lettreGauche);
   
   // Afficher "tirette en place !"
   display.setTextSize(1);
   display.setCursor(0, 24);
   display.print("Tirette en place !");
   
   display.display();

}

void loop() {

    // attendre tant que la tirette n'est pas retirée
    int startSignal = digitalRead(PIN_START);
    int equipe = digitalRead(PIN_EQUIPE);
    if (!started) {
        if (startSignal == LOW) {
            // "Tirette retirée ! Démarrage du décompte...
            started = true;
        } else {
            // Afficher en continu le message d'attente
            display.clearDisplay();
            display.setTextColor(SSD1306_WHITE);
            
            // Afficher la lettre en haut à gauche
            display.setTextSize(4);
            display.setCursor(2, 0);
            display.print(lettreGauche);
            
            // Afficher "tirette en place !"
            display.setTextSize(1);
            display.setCursor(0, 24);
            display.print("Tirette en place !");
            
            display.display();
            
            Serial.println("En attente de la tirette...");
            delay(300);
            return;   // Tant que la tirette n'est pas retirée, on ne fait rien
        }
    }

    // Décompte de 85 secondes après retrait de la tirette
    if (started && !decompte_termine) {
        decompte(DECOMPTE_INITIAL);
        return;
    }

    // Se déplacer à la destination (Les steppers - exécuté une seule fois)
    if (decompte_termine && !mouvement_termine) {
          
       if (equipe == BLEU) 
         { // Equipe BLEU
            
            // Séquence de mouvements
            avancer(1000);        // Avance de 10cm
            delay(10);
            
            tourner_droite(600);  // Tourne à droite 
            delay(10);

            avancer(5000);        // Avance de 50cm
            delay(10);

            tourner_gauche(600);  // Tourne à gauche 
            delay(10);

            avancer(6000);        // Avance de 50cm
            delay(10);

        
         } else // Equipe JAUNE
         {  

            // Séquence de mouvements
            avancer(1000);        // Avance de 10cm
            delay(500);
            
            tourner_gauche(600);  // Tourne à gauche 
            delay(500);

            avancer(5000);        // Avance de 50cm
            delay(500);
        
         }
        
        mouvement_termine = true;
        //Séquence de mouvements terminée. Activation du servo en continu
    }
    
    // mouvement_termine = true;
    // Une fois arrivé à destination, le servo fonctionne en permanence
    if (mouvement_termine) {
        servo1.write(180);
        delay(500);  
        servo1.write(0);  
        delay(500);
    }

}




