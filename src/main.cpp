#include <Arduino.h>
#include <TFT_eSPI.h>
#include <TFT_eFEX.h>

#include <menu.h>
#include <menuIO/TFT_eSPIOut.h>
#include <streamFlow.h>
#include <ClickEncoder.h>
// Using this library: https://github.com/soligen2010/encoder.git
#include <menuIO/clickEncoderIn.h>
#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>
#include <menuIO/serialIO.h>
// For debugging the TFT_eSPIOut.h library
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>

#include <AccelStepper.h>

using namespace Menu;

// define menu colors --------------------------------------------------------
#define Black RGB565(0, 0, 0)
#define Red RGB565(255, 0, 0)
#define Green RGB565(0, 255, 0)
#define Blue RGB565(0, 0, 255)
#define Gray RGB565(128, 128, 128)
#define LighterRed RGB565(255, 60, 92)    // colores OP-1
#define LighterGreen RGB565(6, 236, 150)  //
#define LighterBlue RGB565(111, 132, 225) //
#define LighterGray RGB565(211, 211, 211)
#define DarkerRed RGB565(150, 0, 0)
#define DarkerGreen RGB565(0, 150, 0)
#define DarkerBlue RGB565(0, 0, 150)
#define Cyan RGB565(0, 255, 255)
#define Magenta RGB565(255, 0, 255)
#define Yellow RGB565(255, 255, 0)
#define White RGB565(255, 255, 255)
#define DarkerOrange RGB565(255, 140, 0)

// TFT color table
const colorDef<uint16_t> colors[6] MEMMODE = {
    //{{disabled normal,disabled selected},{enabled normal,enabled selected, enabled editing}}
    {{(uint16_t)Black, (uint16_t)Black}, {(uint16_t)Black, (uint16_t)Red, (uint16_t)Red}},     // bgColor
    {{(uint16_t)White, (uint16_t)White}, {(uint16_t)White, (uint16_t)White, (uint16_t)White}}, // fgColor
    {{(uint16_t)Red, (uint16_t)Red}, {(uint16_t)Yellow, (uint16_t)Yellow, (uint16_t)Yellow}},  // valColor
    {{(uint16_t)White, (uint16_t)White}, {(uint16_t)White, (uint16_t)White, (uint16_t)White}}, // unitColor
    {{(uint16_t)White, (uint16_t)Gray}, {(uint16_t)Black, (uint16_t)Red, (uint16_t)White}},    // cursorColor
    {{(uint16_t)White, (uint16_t)Yellow}, {(uint16_t)Black, (uint16_t)Red, (uint16_t)Red}},    // titleColor
};

// Define the width and height of the TFT and how much of it to take up
#define GFX_WIDTH 240
#define GFX_HEIGHT 135
#define fontW 6
#define fontH 9

// Declare pins for rotary encoder
#define encA 9
#define encB 10
#define encBtn 4
// steps per detent
#define encSteps 4

// Declare pins for joystick
#define joyX 36
#define joyY 39
#define joySW 21 // por ahi puedo usar el mismo pin que el encoder

#define stopSW 32

// motor pins
#define dirX 33
#define stepX 25

// Setup TFT colors.  Probably stop using these and use the colors defined by ArduinoMenu
#define BACKCOLOR TFT_BLACK
#define TEXTCOLOR TFT_WHITE

// params menu
int chooseField = 1;
int exitMenuOptions = 0;
int menuDelayTime = 100;

// params medicion
int fuerzaInicial = 0; // N
int fuerzaFinal = 5;   // N
int velocidad = 10;    // mm x minuto
int largo = 10;        // mm

// params calibracion
int cantVeces = 10;
int cantMm = 5;
int pasosPorMm = 1600;

// params motores
// int maxSpeedX = 25000;
int maxSpeedX = 5000;
const int MICROSTEP = 32;

// params tiempo
int INPUT_READ_INTERVAL = 100;
unsigned long last_input_time = 0;

// params botones
int joySW_status = 1;

AccelStepper stepperX(AccelStepper::DRIVER, stepX, dirX);

// Declare the clickencoder
// Disable doubleclicks in setup makes the response faster.  See: https://github.com/soligen2010/encoder/issues/6
ClickEncoder clickEncoder = ClickEncoder(encA, encB, encBtn, encSteps);
ClickEncoderStream encStream(clickEncoder, 1);

// TFT gfx is what the ArduinoMenu TFT_eSPIOut.h is expecting
TFT_eSPI gfx = TFT_eSPI();
TFT_eFEX fex = TFT_eFEX(&gfx);

void medir();
void medirProgreso();
void definirOrigen();
void calibrar();
void homing();
void IRAM_ATTR onTimer(); // Start the timer to read the clickEncoder every 1 ms

float mm2step(float mm) { return mm * MICROSTEP * 100; }
float step2mm(float step) { return step / (MICROSTEP * 100); }
float mmxm2stepxs(float mmxm) { return mmxm * MICROSTEP * 100 / 60; }
float stepxs2mmxm(float stepxs) { return stepxs * 60 / (MICROSTEP * 100); }

//////////////////////////////////////////////////////////
// Start ArduinoMenu
//////////////////////////////////////////////////////////

result doDefinirOrigen()
{
    delay(menuDelayTime);
    exitMenuOptions = 1;
    return proceed;
}

result doMedir()
{
    delay(menuDelayTime);
    exitMenuOptions = 2;
    return proceed;
}

result doMedirProgreso()
{
    delay(menuDelayTime);
    exitMenuOptions = 3;
    return proceed;
}

result doHoming()
{
    delay(menuDelayTime);
    exitMenuOptions = 4;
    return proceed;
}

result doCalibrar()
{
    delay(menuDelayTime);
    exitMenuOptions = 5;
    return proceed;
}

result updateEEPROM()
{
    // writeEEPROM();
    return quit;
}

#define MAX_DEPTH 2

MENU(subMenuCalibrar, "Menu de calibracion", doNothing, noEvent, noStyle,
     OP("Calibrar", doCalibrar, enterEvent),
     FIELD(cantVeces, "Cantidad de veces:", "", 0, 200, 10, 1, doNothing, noEvent, noStyle),
     FIELD(cantMm, "Cantidad de mm:", "", 0, 100, 10, 1, doNothing, noEvent, noStyle),
     FIELD(pasosPorMm, "Pasos por mm:", "", 1500, 1700, 10, 1, doNothing, noEvent, noStyle),
     EXIT("<- Volver"));

MENU(mainMenu, "SCRATCH TESTER 3000", doNothing, noEvent, wrapStyle,
     FIELD(fuerzaInicial, "Fuerza inicial:", "N", 0, 200, 10, 1, doNothing, noEvent, noStyle),
     FIELD(fuerzaFinal, "Fuerza final:", "N", 0, 200, 10, 1, doNothing, noEvent, noStyle),
     FIELD(velocidad, "Velocidad:", "mm/s", 0, 200, 10, 1, doNothing, noEvent, noStyle),
     FIELD(largo, "Largo:", "mm", 0, 20, 1, 1, doNothing, noEvent, noStyle),
     OP("Definir origen", doDefinirOrigen, enterEvent),
     OP("Medir!", doMedir, enterEvent),
     OP("Medir con progreso", doMedirProgreso, enterEvent),
     OP("Homing", doHoming, enterEvent),
     SUBMENU(subMenuCalibrar)
     //  ,SUBMENU(configuracion)
);

const panel panels[] MEMMODE = {{0, 0, GFX_WIDTH / fontW, GFX_HEIGHT / fontH}}; // Main menu panel
navNode *nodes[sizeof(panels) / sizeof(panel)];                                 // navNodes to store navigation status
panelsList pList(panels, nodes, sizeof(panels) / sizeof(panel));                // a list of panels and nodes
// idx_t tops[MAX_DEPTH]={0,0}; // store cursor positions for each level
idx_t eSpiTops[MAX_DEPTH] = {0};
TFT_eSPIOut eSpiOut(gfx, colors, eSpiTops, pList, fontW, fontH + 1);
idx_t serialTops[MAX_DEPTH] = {0};
serialOut outSerial(Serial, serialTops);
menuOut *constMEM outputs[] MEMMODE = {&outSerial, &eSpiOut};  // list of output devices
outputsList out(outputs, sizeof(outputs) / sizeof(menuOut *)); // outputs list
serialIn serial(Serial);
MENU_INPUTS(in, &encStream, &serial); // &encButton,
NAVROOT(nav, mainMenu, MAX_DEPTH, in, out);

// ESP32 timer thanks to: http://www.iotsharing.com/2017/06/how-to-use-interrupt-timer-in-arduino-esp32.html
// and: https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//////////////////////////////////////////////////////////
// End Arduino Menu
//////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(115200);
    delay(3000);

    clickEncoder.setAccelerationEnabled(true);
    clickEncoder.setDoubleClickEnabled(false); // Disable doubleclicks makes the response faster.  See: https://github.com/soligen2010/encoder/issues/6

    // // ESP32 timer
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);

    gfx.init();         // Initialize the display
    gfx.setRotation(0); // Set the rotation (0-3) to vertical
    Serial.println("Initialized display");
    gfx.fillScreen(TFT_BLACK);
    Serial.println("done");

    nav.showTitle = true; // Show titles in the menus and submenus
    //  nav.timeOut = 60;  // Timeout after 60 seconds of inactivity
    //  nav.idleOn(); // Start with the main screen and not the menu

    stepperX.setMaxSpeed(maxSpeedX);
    pinMode(joySW, INPUT_PULLUP);
}

void loop()
{
    // Slow down the menu redraw rate
    constexpr int menuFPS = 1000 / 30;
    static unsigned long lastMenuFrame = -menuFPS;
    unsigned long now = millis();
    //... other stuff on loop, will keep executing
    // Serial.println(analogRead(joyX));
    // Serial.print("joy ");
    // Serial.println(digitalRead(joySW));
    // Serial.print("enc ");
    // Serial.println(digitalRead(encBtn));
    switch (exitMenuOptions)
    {
    case 1:
    {
        delay(menuDelayTime);
        definirOrigen();
        break;
    }
    case 2:
    {
        delay(menuDelayTime);
        medir();
        break;
    }
    case 3:
    {
        delay(menuDelayTime);
        medirProgreso();
        break;
    }
    case 4:
    {
        delay(menuDelayTime);
        homing();
        break;
    }
    case 5:
    {
        delay(menuDelayTime);
        calibrar();
        break;
    }
    default: // Do the normal program functions with ArduinoMenu
        if (now - lastMenuFrame >= menuFPS)
        {
            lastMenuFrame = millis();
            nav.poll(); // Poll the input devices
        }
    }
}

void definirOrigen()
{
    exitMenuOptions = 0;
    // Return to the menu
    delay(menuDelayTime);

    // samplea 10 veces y toma el promedio para ver el cero del joystick
    int suma = 0;
    for (int i = 0; i < 50; i++)
    {
        suma += analogRead(joyX);
    }
    float bufferMax = (suma / 10) + 50;
    float bufferMin = (suma / 10) - 50;
    Serial.print("buffer min ");
    Serial.println(bufferMin);
    stepperX.setAcceleration(4 * maxSpeedX);

    while (digitalRead(joySW))
    {
        // Every INPUT_READ_INTERVAL milliseconds, read inputs.
        // We do this infrequently to prevent interfering
        // with the stepper motor high speed stepping
        // Get the current joystick position as analog value

        unsigned long current_time = millis();
        if (current_time - last_input_time > INPUT_READ_INTERVAL)
        {
            int joyX_value = analogRead(joyX);
            // Map the raw analog value to speed range from -maxSpeedX to maxSpeedX
            int desired_speed = map(joyX_value, 0, 4095, -maxSpeedX, maxSpeedX);
            // Serial.println(desired_speed);

            // Based on the input, set targets and max speed
            stepperX.setMaxSpeed(abs(desired_speed));
            Serial.print("speed ");
            Serial.println(desired_speed);
            Serial.print("analog ");
            Serial.println(joyX_value);
            if (joyX_value > bufferMin && bufferMax > joyX_value)
            {
                Serial.println("skip");
                continue;
            }
            else if (desired_speed == 0 && stepperX.speed() == 0)
            {
                // Prevent running off the end of the position range, quizas no lo necesito, interfiere con el posicionamiento
                // stepperX.setCurrentPosition(0);
                Serial.print("Posicion = ");
                Serial.println(stepperX.currentPosition());
            }
            else if (desired_speed < 0)
            {
                Serial.println("negativo");
                stepperX.moveTo(-1000000000);
            }
            else if (desired_speed > 0)
            {
                stepperX.moveTo(1000000000);
            }
            last_input_time = current_time;
        }
        stepperX.run();
    }
    stepperX.setCurrentPosition(0);
    mainMenu.dirty = true; // Force the main menu to redraw itself
}

void medir()
{
    exitMenuOptions = 0; // Return to the menu
    delay(menuDelayTime);
    float largoSteps = mm2step(largo);
    stepperX.setSpeed(mmxm2stepxs(velocidad));
    stepperX.move(largoSteps);
    stepperX.runToPosition();
    // volver a la posicion anterior
    delay(menuDelayTime);
    stepperX.setSpeed(maxSpeedX);
    stepperX.move(-largoSteps);
    stepperX.runToPosition();

    mainMenu.dirty = true; // Force the main menu to redraw itself
}

void medirProgreso()
{
    exitMenuOptions = 0; // Return to the menu
    delay(menuDelayTime);
    float largoSteps = mm2step(largo);
    stepperX.setSpeed(mmxm2stepxs(velocidad));
    stepperX.move(largoSteps);
    last_input_time = 0;
    while (stepperX.distanceToGo() != 0)
    {
        unsigned long current_time = millis();
        if (current_time - last_input_time > 500)
        {
            float percent = 100 * stepperX.currentPosition() / largoSteps;
            fex.drawProgressBar(10, 70, 20, 20, percent, TFT_DARKGREEN, TFT_DARKCYAN);
            last_input_time = current_time;
        }
        stepperX.runSpeedToPosition();
    }
    delay(menuDelayTime);
    stepperX.setSpeed(maxSpeedX);
    stepperX.move(-largoSteps);
    stepperX.runToPosition();
    mainMenu.dirty = true;
}

void homing()
{
    exitMenuOptions = 0;
    stepperX.setSpeed(maxSpeedX / 2.0);
    stepperX.move(-mm2step(300));
    while (digitalRead(stopSW))
    {
        stepperX.runSpeedToPosition();
    }
    stepperX.move(mm2step(300));
    while (!digitalRead(stopSW))
    {
        stepperX.runSpeedToPosition();
    }
    stepperX.setCurrentPosition(0);
    mainMenu.dirty = true;
}

void calibrar()
{
    exitMenuOptions = 0;
    stepperX.setSpeed(mmxm2stepxs(velocidad));
    for (int i = 0; i < cantVeces; i++)
    {
        stepperX.move(pasosPorMm * cantMm);
        stepperX.runToPosition();

        stepperX.move(-pasosPorMm * cantMm);
        stepperX.runToPosition();
    }
    mainMenu.dirty = true;
}
// ESP32 timer
void IRAM_ATTR onTimer()
{
    clickEncoder.service();
}
