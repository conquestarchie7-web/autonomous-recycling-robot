/*
 * Autonomous Recycling Robot - Secondary Colour Sensor Board
 *
 * Responsibilities:
 *   - Read RGB values from the TCS34725 colour sensor
 *   - Classify collected objects as RED, GREEN or BLUE
 *   - Display robot status and sensor readings on a 16x2 LCD
 *   - Notify the primary robot board of the detected object colour
 *   - Provide audible status feedback
 */

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

// TCS34725 colour sensor
Adafruit_TCS34725 tcs =
    Adafruit_TCS34725(
        TCS34725_INTEGRATIONTIME_50MS,
        TCS34725_GAIN_4X
    );

// 16x2 LCD: RS, E, D4, D5, D6, D7
LiquidCrystal lcd(11, 10, 9, 7, 6, 5, 4);

// Communication with primary board
SoftwareSerial mySerial(2, 3);  // RX=2, TX=3

const int buzzer = 8;
String lastStatus = "";

// Tone frequencies used for the completion melody.
#define G4  392
#define E4  330
#define C4  262
#define D4  294
#define F4  349
#define A4  440
#define AS4 466
#define GS4 415
#define DS4 311

// -------------------- Audio feedback --------------------

void playTone(int freq, int dur) {
    tone(buzzer, freq, dur);
    delay(dur * 1.3);
    noTone(buzzer);
}

/*
 * Completion melody used when all required deposits are finished.
 */
void imperialMarch() {
    playTone(G4,  500);
    playTone(G4,  500);
    playTone(G4,  500);
    playTone(DS4, 350);
    playTone(AS4, 150);
    playTone(G4,  500);
    playTone(DS4, 350);
    playTone(AS4, 150);
    playTone(G4,  700);

    delay(200);

    playTone(D4,  500);
    playTone(D4,  500);
    playTone(D4,  500);
    playTone(DS4, 350);
    playTone(AS4, 150);
    playTone(GS4, 500);
    playTone(DS4, 350);
    playTone(AS4, 150);
    playTone(G4,  700);

    delay(200);

    playTone(G4,  500);
    playTone(G4,  350);
    playTone(G4,  150);
    playTone(G4,  500);
    playTone(GS4, 350);
    playTone(AS4, 150);
    playTone(A4,  300);
    playTone(GS4, 150);
    playTone(AS4, 150);

    delay(150);

    playTone(DS4, 500);
    playTone(D4,  350);
    playTone(C4,  150);
    playTone(AS4, 500);
    playTone(G4,  350);
    playTone(DS4, 150);
    playTone(AS4, 150);
    playTone(G4,  700);
}

// -------------------- Colour detection --------------------

/*
 * Read RGB values and classify the object.
 * Raw RGB values are also shown on the LCD for calibration.
 */
String readColour() {
    float red, green, blue;
    tcs.getRGB(&red, &green, &blue);

    lcd.setCursor(0, 1);

    lcd.print("R:");
    lcd.print((int)red);

    lcd.print(" G:");
    lcd.print((int)green);

    lcd.print(" B:");
    lcd.print((int)blue);

    if (red > green &&
        red > (blue + 15) &&
        red > 90) {

        return "RED";

    } else if (green > red &&
               green > (blue + 15) &&
               green > 90) {

        return "GREEN";

    } else if ((blue + 15) > red &&
               (blue + 15) > green &&
               (blue + 15) > 90) {

        return "BLUE";
    }

    return "UNKNOWN";
}

// -------------------- Arduino setup --------------------

void setup() {
    Serial.begin(9600);
    mySerial.begin(57600);

    lcd.begin(16, 2);
    lcd.print("Ready");

    if (!tcs.begin()) {
        lcd.clear();
        lcd.print("TCS34725 Error!");

        while (true) {
        }
    }

    pinMode(buzzer, OUTPUT);
}

// -------------------- Main loop --------------------

void loop() {
    if (mySerial.available()) {
        String incoming = mySerial.readStringUntil('\n');
        incoming.trim();

        lastStatus = incoming;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(incoming);

        // Primary board is requesting an object colour reading.
        if (incoming == "GRABBING") {
            delay(2000);

            String colour = readColour();

            lcd.clear();
            lcd.print("Can is: ");
            lcd.print(colour);

            lcd.setCursor(0, 1);
            lcd.print("Looking for depo");

            mySerial.println(colour);

            delay(2000);
        }

        else if (incoming == "DROPPING" ||
                 incoming == "DEPOSITING") {

            delay(3000);

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(incoming);
        }

        else if (incoming == "FINISHED") {
            imperialMarch();

            while (true) {
            }
        }

        else if (incoming == "OBJECT IN FRONT") {
            tone(buzzer, 1000);
            delay(3000);
            noTone(buzzer);
        }
    }
}
