#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Lettre à afficher à gauche
char lettreGauche = 'J'; // peut être 'J' ou 'B'

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(8, 9); // SDA=8, SCL=9

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("Erreur : OLED non détecté"));
    for (;;);
  }

  display.clearDisplay();
  display.display();
}

void loop() {
  for (int i = 85; i >= 0; i--) {
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
  display.print("TERMINE !");
  display.display();

  while (true); // stop
}
