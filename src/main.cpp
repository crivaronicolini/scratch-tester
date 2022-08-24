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

#include <TMCStepper.h>
#include <AccelStepper.h>

#include <PID_v1.h>

#include <HX711.h>

HX711 scale;

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
#define fontW 12
#define fontH 15

// Declare pins for rotary encoder
#define encB 36 // dt
#define encA 39 // clk
#define encBtn 34
// steps per detent
#define encSteps 4

// Declare pins for joystick
#define joyY 35
#define joyX 32
#define joySW 34

#define stopSW 0

// motor pins
#define dirX 4
#define stepX 16
#define dirY 21
#define stepY 22

#define SW_MISO 19 // Software Master In Slave Out (MISO)
#define CS_PINX 17 // Chip select
#define CS_PINY 5  // Chip select
#define SW_SCK 18  // Software Slave Clock (SCK)
#define SW_MOSI 23 // Software Master Out Slave In (MOSI)
#define R_SENSE 0.11f

// HX711
#define load 33
#define SCK 25

// 500g
// #define CALIBRATION_FACTOR 50100
// 100g
// #define CALIBRATION_FACTOR 10297
// 1000g
#define CALIBRATION_FACTOR 10180500

// Setup TFT colors.  Probably stop using these and use the colors defined by ArduinoMenu
#define BACKCOLOR TFT_BLACK
#define TEXTCOLOR TFT_WHITE

// params menu
int chooseField = 1;
int exitMenuOptions = 0;
int menuDelayTime = 100;

// params medicion
int fuerzaInicial = 0; // N
int fuerzaFinal = 1;   // N
int velocidad = 100;   // mm x minuto
int largo = 10;        // mm

// params PID
double fuerzaSetpoint = fuerzaFinal * 102, fuerzaInput, fuerzaOutput;
double Kp = 2, Ki = 5, Kd = 1;

// params calibracion
int cantVeces = 2;
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

PID fuerzaPID(&fuerzaInput, &fuerzaOutput, &fuerzaSetpoint, Kp, Ki, Kd, DIRECT);

TMC2130Stepper driverX = TMC2130Stepper(CS_PINX, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
TMC2130Stepper driverY = TMC2130Stepper(CS_PINY, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

AccelStepper stepperX(AccelStepper::DRIVER, stepX, dirX);
AccelStepper stepperY(AccelStepper::DRIVER, stepY, dirY);

// Declare the clickencoder
// Disable doubleclicks in setup makes the response faster.  See: https://github.com/soligen2010/encoder/issues/6
ClickEncoder clickEncoder = ClickEncoder(encA, encB, encBtn, encSteps);
ClickEncoderStream encStream(clickEncoder, 1);

// TFT gfx is what the ArduinoMenu TFT_eSPIOut.h is expecting
TFT_eSPI gfx = TFT_eSPI();
TFT_eFEX fex = TFT_eFEX(&gfx);

result medir();
result medirProgreso();
result definirOrigen();
result homing();
result calibrarMotores();
result calibrarPID();
result calibrarCelda();
void IRAM_ATTR onTimer(); // Start the timer to read the clickEncoder every 1 ms

float mm2step(float mm) { return mm * MICROSTEP * 100; }
float step2mm(float step) { return step / (MICROSTEP * 100); }
float mmxm2stepxs(float mmxm) { return mmxm * MICROSTEP * 100 / 60; }
float stepxs2mmxm(float stepxs) { return stepxs * 60 / (MICROSTEP * 100); }

#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(...) Serial.printf(__VA_ARGS__)
#else
#define debug(x)
#define debugln(x)
#define debugf(x)
#endif

//////////////////////////////////////////////////////////
// Start ArduinoMenu
//////////////////////////////////////////////////////////

result updateEEPROM()
{
    // writeEEPROM();
    return quit;
}

#define MAX_DEPTH 3

int ejeACalibrar = 1;

TOGGLE(ejeACalibrar, subMenuToggleEjeACalibrar, "Motor a Calibrar: ", doNothing, noEvent, noStyle,
       VALUE("X", 1, doNothing, noEvent),
       VALUE("Y", 2, doNothing, noEvent));

MENU(subMenuCalibrarMotores, "Calibracion  de Motores", doNothing, noEvent, wrapStyle,
     OP("Calibrar", calibrarMotores, enterEvent),
     SUBMENU(subMenuToggleEjeACalibrar),
     FIELD(cantVeces, "Cantidad de veces:", "", 0, 200, 10, 0, doNothing, noEvent, noStyle),
     FIELD(cantMm, "Cantidad de mm:", "", 0, 100, 10, 1, doNothing, noEvent, noStyle),
     FIELD(pasosPorMm, "Pasos por mm:", "", 1500, 1700, 10, 1, doNothing, noEvent, noStyle),
     EXIT("<- Volver"));

MENU(subMenuCalibrarPID, "Calibracion de PID", doNothing, noEvent, wrapStyle,
     OP("Calibrar PID", calibrarPID, enterEvent),
     FIELD(Kp, "Proporcional: ", "", 0, 50, 1, 0, doNothing, noEvent, noStyle),
     FIELD(Ki, "Integrador: ", "", 0, 50, 1, 0, doNothing, noEvent, noStyle),
     FIELD(Kd, "Derivador: ", "", 0, 50, 1, 0, doNothing, noEvent, noStyle),
     EXIT("<- Volver"));

MENU(subMenuCalibrar, "Menu de calibracion", doNothing, noEvent, wrapStyle,
     OP("Calibrar Celda de Carga", calibrarCelda, enterEvent),
     SUBMENU(subMenuCalibrarPID),
     SUBMENU(subMenuCalibrarMotores),
     EXIT("<- Volver"));

MENU(mainMenu, "SCRATCH TESTER 3000", doNothing, noEvent, wrapStyle,
     FIELD(fuerzaInicial, "Fuerza inicial:", "N", 0, 200, 10, 1, doNothing, noEvent, noStyle),
     FIELD(fuerzaFinal, "Fuerza final:", "N", 0, 200, 10, 1, doNothing, noEvent, noStyle),
     FIELD(velocidad, "Velocidad:", "mm/s", 0, 200, 10, 1, doNothing, noEvent, noStyle),
     FIELD(largo, "Largo:", "mm", 0, 20, 1, 1, doNothing, noEvent, noStyle),
     OP("Definir origen", definirOrigen, enterEvent),
     OP("Medir!", medir, enterEvent),
     OP("Medir con progreso", medirProgreso, enterEvent),
     OP("Homing", homing, enterEvent),
     SUBMENU(subMenuCalibrar));

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
    stepperX.setMaxSpeed(maxSpeedX);
    stepperX.setAcceleration(10000);
    // pinMode(CS_PIN,OUTPUT);
    // digitalWrite(CS_PIN, LOW);
    driverX.begin();          // Initiate pins and registeries
    driverX.rms_current(600); // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    driverX.en_pwm_mode(1);   // Enable extremely quiet stepping
    driverX.pwm_autoscale(1);
    driverX.microsteps(16);

    stepperY.setMaxSpeed(maxSpeedX);
    stepperY.setAcceleration(10000);
    // pinMode(CS_PIN,OUTPUT);
    // digitalWrite(CS_PIN, LOW);
    driverY.begin();          // Initiate pins and registeries
    driverY.rms_current(600); // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    driverY.en_pwm_mode(1);   // Enable extremely quiet stepping
    driverY.pwm_autoscale(1);
    driverY.microsteps(16);
    // digitalWrite(CS_PIN, HIGH);
    // delay(1000);

    stepperX.move(100 * 16);
    stepperX.runToPosition();
    stepperY.move(100 * 16);
    stepperY.runToPosition();

    clickEncoder.setAccelerationEnabled(true);
    clickEncoder.setDoubleClickEnabled(false); // Disable doubleclicks makes the response faster.  See: https://github.com/soligen2010/encoder/issues/6

    // // ESP32 timer
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);

    // pinMode(5, OUTPUT);
    // digitalWrite(5, LOW);
    gfx.init();         // Initialize the display
    gfx.setRotation(1); // Set the rotation (0-3) to vertical
    debugln("Initialized display");
    // gfx.fillScreen(TFT_BLACK);
    gfx.fillScreen(TFT_WHITE);
    debugln("done");

    nav.showTitle = true; // Show titles in the menus and submenus
    //  nav.timeOut = 60;  // Timeout after 60 seconds of inactivity
    //  nav.idleOn(); // Start with the main screen and not the menu

    pinMode(encBtn, INPUT_PULLUP);

    scale.begin(load, SCK);
    if (scale.wait_ready_retry(3, 500))
    {
        scale.set_scale(CALIBRATION_FACTOR);
        debugln("\nTare... remove any weights from the scale.");
        scale.tare(20);
        debugln("Tare done...");
    }
    else
    {
        debugln("\nHX711 not found.");
    }
    fuerzaPID.SetMode(0);
}

void loop()
{
    // Slow down the menu redraw rate
    constexpr int menuFPS = 1000 / 30;
    static unsigned long lastMenuFrame = -menuFPS;
    unsigned long now = millis();

    if (now - lastMenuFrame >= menuFPS)
    {
        lastMenuFrame = millis();
        nav.poll();
    }
}

result definirOrigen()
{
    exitMenuOptions = 0;
    // Return to the menu
    delay(menuDelayTime);

    // samplea 10 veces y toma el promedio para ver el cero del joystick
    int suma = 0;
    int cantidad = 50;
    for (int i = 0; i < cantidad; i++)
    {
        suma += analogRead(joyX);
    }
    float bufferMaxX = (suma / cantidad) + 100;
    float bufferMinX = (suma / cantidad) - 100;
    suma = 0;
    cantidad = 50;
    for (int i = 0; i < cantidad; i++)
    {
        suma += analogRead(joyY);
    }
    float bufferMaxY = (suma / cantidad) + 100;
    float bufferMinY = (suma / cantidad) - 100;
    debugf("buffer maxX: %f\n", bufferMaxX);
    debugf("buffer minX: %f\n", bufferMinX);
    debugf("buffer maxY: %f\n", bufferMaxY);
    debugf("buffer minY: %f\n", bufferMinY);
    stepperX.setAcceleration(4 * maxSpeedX);
    stepperY.setAcceleration(4 * maxSpeedX);

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
            int joyY_value = analogRead(joyY);
            // Map the raw analog value to speed range from -maxSpeedX to maxSpeedX
            int desired_speedX = map(joyX_value, 0, 4095, -maxSpeedX, maxSpeedX);
            int desired_speedY = map(joyY_value, 0, 4095, -maxSpeedX, maxSpeedX);
            // debugln(desired_speedX);

            // Based on the input, set targets and max speed
            stepperX.setMaxSpeed(abs(desired_speedX));
            stepperY.setMaxSpeed(abs(desired_speedY));
            debugf("X %d; Y %d\n", joyX_value, joyY_value);

            if (not(joyX_value > bufferMinX && bufferMaxX > joyX_value))
            {
                // {
                //     // debugln("skip");
                //     continue;
                // }
                // if ((desired_speedX == 0 && stepperX.speed() == 0) || (desired_speedY == 0 && stepperY.speed() == 0))
                // {
                //     // Prevent running off the end of the position range, quizas no lo necesito, interfiere con el posicionamiento
                //     // stepperX.setCurrentPosition(0);
                //     debugf("Posicion X %d\n", stepperX.currentPosition());
                //     debugf("Posicion Y %d\n", stepperY.currentPosition());
                //     continue;
                // }
                if (desired_speedX < 0)
                {
                    debugln("X negativo");
                    // debugf("X speed %d\n", desired_speedX);
                    // debugf("X analog %d\n", joyX_value);
                    // debugf("Y speed %d\n", desired_speedY);
                    // debugf("Y analog %d\n", joyY_value);
                    stepperX.moveTo(-1000000000);
                }
                else if (desired_speedX > 0)
                {
                    debugln("X positivo");
                    // debugf("X speed %d\n", desired_speedX);
                    // debugf("X analog %d\n", joyX_value);
                    // debugf("Y speed %d\n", desired_speedY);
                    // debugf("Y analog %d\n", joyY_value);
                    stepperX.moveTo(1000000000);
                }
            }
            if (not(joyY_value > bufferMinY && bufferMaxY > joyY_value))
            {
                if (desired_speedY < 0)
                {
                    debugln("Y negativo");
                    // debugf("X speed %d\n", desired_speedX);
                    // debugf("X analog %d\n", joyX_value);
                    // debugf("Y speed %d\n", desired_speedY);
                    // debugf("Y analog %d\n", joyY_value);
                    stepperY.moveTo(-1000000000);
                }
                else if (desired_speedY > 0)
                {
                    debugln("Y positivo");
                    // debugf("X speed %d\n", desired_speedX);
                    // debugf("X analog %d\n", joyX_value);
                    // debugf("Y speed %d\n", desired_speedY);
                    // debugf("Y analog %d\n", joyY_value);
                    stepperY.moveTo(1000000000);
                }
                last_input_time = current_time;
                debugf("\n");
            }
        }
        stepperX.run();
        stepperY.run();
    }
    stepperX.setCurrentPosition(0);
    stepperY.setCurrentPosition(0);
    mainMenu.dirty = true;
    return proceed;
}

result medir()
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
    return proceed;
}

result medirProgreso()
{
    exitMenuOptions = 0; // Return to the menu
    delay(menuDelayTime);
    float largoSteps = mm2step(largo);
    // stepperX.setSpeed(mmxm2stepxs(velocidad));
    stepperX.setSpeed(10000000);
    stepperX.move(largoSteps);
    last_input_time = 0;
    gfx.fillScreen(Black);
    while (stepperX.distanceToGo() != 0)
    {
        unsigned long current_time = millis();
        if (current_time - last_input_time > 500)
        {
            float percent = 100 * stepperX.currentPosition() / largoSteps;
            fex.drawProgressBar(20, 70, 200, 25, percent, Red, White);
            last_input_time = current_time;
        }
        stepperX.runSpeed();
        // stepperX.runSpeedToPosition();
        debugln(stepperX.currentPosition());
    }
    delay(menuDelayTime);
    debug("volviendo al origen");
    stepperX.setSpeed(maxSpeedX);
    stepperX.move(-largoSteps);
    stepperX.runToPosition();
    mainMenu.dirty = true;
    return proceed;
}

result homing()
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
    return proceed;
}

result calibrarMotores()
{
    exitMenuOptions = 0;
    AccelStepper stp;
    if (ejeACalibrar == 1)
    {
        stp = stepperX;
    }
    else
    {
        stp = stepperY;
    }
    stp.setSpeed(mmxm2stepxs(velocidad));
    debugf("cant veces %d, pasosxmm %d, cantmm %d mm %d step\n", cantVeces, pasosPorMm, cantMm, mm2step(cantMm));
    debugf("velocidad %d mmxm, %f pxs\n", velocidad, mmxm2stepxs(velocidad));
    stp.setAcceleration(4 * maxSpeedX);

    for (int i = 0; i < cantVeces; i++)
    {
        // stp.move(pasosPorMm * cantMm);
        // stp.runToPosition();

        // stp.move(-pasosPorMm * cantMm);
        // stp.runToPosition();
        // debugln("loop 1");

        stp.move(mm2step(cantMm));
        stp.runToPosition();
        delay(1000);
        stp.move(-mm2step(cantMm));
        stp.runToPosition();
        delay(1000);
        debugln("loop 2");
    }
    mainMenu.dirty = true;
    return proceed;
}

result calibrarPID()
{
    exitMenuOptions = 0;
    gfx.fillScreen(TFT_BLACK);
    gfx.setCursor(120, 60);
    gfx.setTextColor(TFT_WHITE, TFT_BLACK);
    gfx.setTextSize(2);
    gfx.drawCentreString("Peso", 120, 30, 1);
    gfx.setTextPadding(100);

    gfx.drawRect(0, 0, 240, 135, TFT_WHITE);

    fuerzaPID.SetMode(AUTOMATIC);

    while (digitalRead(joySW))
    {
        fuerzaInput = 100000 * scale.get_units(1); // gramos
        fuerzaPID.Compute();
        stepperY.setSpeed(fuerzaOutput);
        stepperY.runSpeed();
        gfx.drawFloat(fuerzaInput, 0, 100, 70, 1);
    }
    // debugf("peso %f\n", reading);
    return proceed;
}

result calibrarCelda()
{
    exitMenuOptions = 0;
    gfx.fillScreen(TFT_BLACK);
    gfx.setCursor(120, 60);
    gfx.setTextColor(TFT_WHITE, TFT_BLACK);
    gfx.setTextSize(2);
    gfx.drawCentreString("Peso", 120, 30, 1);
    gfx.setTextPadding(100);

    gfx.drawRect(0, 0, 240, 135, TFT_WHITE);

    if (scale.wait_ready_retry(3, 500))
    {
        scale.set_scale(CALIBRATION_FACTOR);
        // scale.set_scale();
        debugln("\nTare... remove any weights from the scale.");
        // delay(1000);
        scale.tare(50);
        debugln("Tare done...");
        // delay(1000);
        while (digitalRead(joySW))
        {
            long reading = 100000 * scale.get_units(1);
            // debugf("peso %f\n", reading);
            gfx.drawFloat(reading, 0, 100, 70, 1);
        }
    }
    else
    {
        debugln("\nHX711 not found.");
    }
    gfx.setTextPadding(0);
    gfx.setTextSize(1);
    mainMenu.dirty = true;
    return proceed;
}
// ESP32 timer
void IRAM_ATTR onTimer()
{
    clickEncoder.service();
}
