
Dans pami/main.cpp ce qu'il faut mettre à jour :

- décompte initial à mettre à 85 secondes 
#define DECOMPTE_INITIAL 5   // 5 secondes pour les tests



- Ajuster la vitesse et l'accélération :
#define SPEED 6000
#define ACCELERATION 2000

- Paramètres des rouesu à ajuster si changement
const float WHEEL_DIAMETER = 60.0;  // Diamètre des roues en mm
const int STEPS_PER_REV = 200;      // Nombre de pas par tour du moteur (200 pour 1.8°, 400 pour 0.9°)
const int MICROSTEPS = 1;            // Microstepping (1, 2, 4, 8, 16, etc.)
const float STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / (PI * WHEEL_DIAMETER);

- le déplacement se fait avec appel des fonctions:  
  -- avancer(distance) à régler avec adhérence de roues   avancer(1000) => avance de 10cm 
  -- tourner_droite(600) : 90 degrès à droite
  -- tourner_gauche(600) : 90 degrés à gauche


- l'interrupteur permet de sélectionner l'équipe Jaune ou Bleu avec affichage lettre J ou B  (0 : JAUNE : mise à la masse)
  en fonction de l'équipe Jaune ou Bleu on décrit le déplacement du PAMI (dans void loop)
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


