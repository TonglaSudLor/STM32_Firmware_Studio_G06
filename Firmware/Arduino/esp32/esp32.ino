/**
 * @file esp32.ino
 * @brief ESP32 Bluetooth Gamepad to UART Bridge (Full Sound Version)
 */

#include <Bluepad32.h>

#define LED_PIN 2 
#define BUZZER_PIN 14

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
String controlState = "OON";
bool isControllerConnected = false;

// UART Handshake
char pending_handshake_char = '\0';

void setup() {
    Serial.begin(115200);
    delay(1000); 

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    playOdeToJoy();
    
    BP32.setup(&onConnectedController, &onDisconnectedController);
}

void playOdeToJoy() {
    const int C4=262, D4=294, E4=330, F4=349, G4=392;
    const int q = 180;  // quarter note ms
    // E  E  F  G  G  F  E  D  C  C  D  E  E.    D/8  D(half)
    int n[] = {E4,E4,F4,G4, G4,F4,E4,D4, C4,C4,D4,E4, E4,D4,D4};
    int d[] = { q, q, q, q,  q, q, q, q,  q, q, q, q, q*3/2, q/2, q*2};
    for (int i = 0; i < 15; i++) {
        tone(BUZZER_PIN, n[i], d[i]);
        delay(d[i] + 20);
    }
}

void playGhostOn() {
    tone(BUZZER_PIN, 1500, 50); delay(60);
    tone(BUZZER_PIN, 2000, 50); delay(60);
    tone(BUZZER_PIN, 2500, 100);
}

void playGhostOff() {
    tone(BUZZER_PIN, 2500, 50); delay(60);
    tone(BUZZER_PIN, 2000, 50); delay(60);
    tone(BUZZER_PIN, 1500, 100);
}

void playJoystickOn() {
    tone(BUZZER_PIN, 800, 100); delay(120);
    tone(BUZZER_PIN, 1400, 150);
}

void playJoystickOff() {
    tone(BUZZER_PIN, 1400, 100); delay(120);
    tone(BUZZER_PIN, 800, 150);
}

void loop() {
    BP32.update();

    static bool is_audio_cmd = false;

    // --- UART AUDIO COMMAND RECEIVER ---
    while (Serial.available() > 0) {
        char rx_char = Serial.read();

        // Check for audio command prefix
        if (rx_char == '@') {
            is_audio_cmd = true;
            continue;
        }

        if (is_audio_cmd) {
            is_audio_cmd = false;
            switch (rx_char) {
                case 'H': // Homing Complete
                    tone(BUZZER_PIN, 1000, 100); delay(120);
                    tone(BUZZER_PIN, 1500, 100); delay(120);
                    tone(BUZZER_PIN, 2500, 200);
                    break;
                case 'E': // Emergency / Fault
                    for (int i=0; i<3; i++) {
                        tone(BUZZER_PIN, 500, 200); delay(250);
                    }
                    break;
                case 'C': // E-Stop Cleared
                    tone(BUZZER_PIN, 1500, 100); delay(150);
                    tone(BUZZER_PIN, 2000, 150);
                    break;
                case 'W': // Soft Limit Warning
                    tone(BUZZER_PIN, 300, 150);
                    break;
                case 'T': // Temp Home Set (Single A)
                    tone(BUZZER_PIN, 3000, 50); delay(70);
                    tone(BUZZER_PIN, 3000, 50);
                    break;
                case '2': // Go to Temp Home (Double A)
                    tone(BUZZER_PIN, 1000, 80); delay(100);
                    tone(BUZZER_PIN, 2000, 150);
                    break;
                case '3': // Go to Origin (Triple A)
                    tone(BUZZER_PIN, 2000, 80); delay(100);
                    tone(BUZZER_PIN, 1000, 150);
                    break;
                case 'G': // Ghost Mode ON
                    playGhostOn();
                    break;
                case 'g': // Ghost Mode OFF
                    playGhostOff();
                    break;
                case 'J': // Joystick Mode ON
                    playJoystickOn();
                    break;
                case 'S': // Base System Mode ON
                    playJoystickOff();
                    break;
                case 'M': // Jog Mode Toggle
                    tone(BUZZER_PIN, 1800, 80); delay(100);
                    tone(BUZZER_PIN, 2200, 100);
                    break;
                case 'h': // Homing Started
                    tone(BUZZER_PIN, 2000, 100); delay(120);
                    tone(BUZZER_PIN, 2500, 100); delay(120);
                    tone(BUZZER_PIN, 3000, 100);
                    break;
            }
        } 
        // 3. Handle standard button handshake (no prefix)
        else if (rx_char == pending_handshake_char && pending_handshake_char != '\0') {
            tone(BUZZER_PIN, 2800, 30);  // Quick confirm blip
            pending_handshake_char = '\0';
        }
    }

    // --- BLUETOOTH JOYSTICK PROCESSOR ---
    isControllerConnected = false;
    char baseChar = 'O';
    char emergencyChar = 'O';
    char statusChar = 'N';

    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected()) {
            isControllerConnected = true;
            statusChar = 'C';
            
            uint16_t btns = ctl->buttons();
            int ry = ctl->axisRY();
            int lx = ctl->axisX();
            int ly = ctl->axisY();

            if (btns & 0x0001)      baseChar = 'A';
            else if (btns & 0x0002) baseChar = 'B';
            else if (btns & 0x0004) baseChar = 'Y';
            else if (btns & 0x0010) baseChar = 'M';
            else if (ly < -150)     baseChar = 'U';
            else if (ly > 150)      baseChar = 'D';
            else if (lx < -150)     baseChar = 'L';
            else if (lx > 150)      baseChar = 'R';
            else if (ry > 150)      baseChar = 'F';

            if (btns & 0x0020) emergencyChar = 'P';

            if (emergencyChar == 'P' && baseChar == 'O') {
                baseChar = 'X';
            }
            break; 
        }
    }

    String currentState = String(baseChar) + String(emergencyChar) + String(statusChar);

    if (controlState != currentState) {
        // Queue the action character for handshake verification
        if (baseChar != 'O' || emergencyChar != 'O') {
            pending_handshake_char = baseChar;
        } else {
            pending_handshake_char = '\0';
        }

        controlState = currentState;
        Serial.println(controlState);
        // Short blip = command transmitted to STM32 (2800Hz blip = STM32 ACK'd it back)
        if (baseChar != 'O' || emergencyChar != 'O') {
            tone(BUZZER_PIN, 2200, 15);
        }
    }

    if (isControllerConnected) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, (millis() / 500) % 2); 
    }

    delay(20);
}

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            tone(BUZZER_PIN, 2000, 100); delay(150);
            tone(BUZZER_PIN, 2500, 150);
            ctl->setRumble(0xFF, 0xFF);
            delay(300);
            ctl->setRumble(0x00, 0x00);
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            tone(BUZZER_PIN, 500, 500); 
            break;
        }
    }
}
