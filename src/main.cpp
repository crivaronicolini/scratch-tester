#include <Arduino.h>
#include <Preferences.h>
Preferences prefs;
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
#define GFX_WIDTH 480
#define GFX_HEIGHT 135
#define fontW 12
#define fontH 15

// Declare pins for rotary encoder
#define encB 36 // dt
#define encA 39 // clk
#define encBtn 2
// steps per detent
#define encSteps 4

// Declare pins for joystick
#define joyY 35
#define joyX 32
#define joySW 2

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
float CALIBRATION_FACTOR = -101.805;

// Setup TFT colors.  Probably stop using these and use the colors defined by ArduinoMenu
#define BACKCOLOR TFT_BLACK
#define TEXTCOLOR TFT_WHITE

// params menu
int chooseField = 1;
int menuDelayTime = 100;

// params medicion
int fuerzaInicial = 0; // N
int fuerzaFinal = 1;   // N
int velocidad = 400;   // mm x minuto
int largo = 5;         // mm

// params PID
double fuerzaSetpoint, fuerzaInput, fuerzaOutput;
double Kp = 0.2, Ki = 0.5, Kd = 0;
int TOL = 20;
float MULT = 0.2;

// params calibracion
int cantVeces = 2;
int cantMm = 5;
int pasosPorMm = 1600;

// params motores
// int maxSpeedX = 25000;
int maxSpeedX = 5000;
int accelerationX = 10000;
int MICROSTEP = 16;

// params tiempo
int INPUT_READ_INTERVAL = 100;
unsigned long last_input_time = 0;
unsigned long lastButtonPress = 0;
unsigned long lastStopTime = 0;

// params botones
int joySW_status = 1;
bool activateEmergencyStop = false;
int buffer = 200;

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
result updatePref();
result medir();
result medirProgreso();
result definirOrigen();
result homing();
result calibrarMotores();
result calibrarPID();
result calibrarCelda();
void initPreferences();
void initMotors();
bool emergencyStopCheck();
void emergencyStop();
void IRAM_ATTR emergencyStopActivate();
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

MENU(subMenuCalibrarCelda, "Calibracion de Celda de Carga", doNothing, noEvent, wrapStyle,
     OP("Calibrar Celda de Carga", calibrarCelda, enterEvent),
     FIELD(CALIBRATION_FACTOR, "Factor de calibracion:", "", 1, 200, 10, 1, doNothing, noEvent, noStyle),
     EXIT("<- Volver"));

TOGGLE(ejeACalibrar, subMenuToggleEjeACalibrar, "Motor a Calibrar: ", doNothing, noEvent, noStyle,
       VALUE("X", 1, doNothing, noEvent),
       VALUE("Y", 2, doNothing, noEvent));

MENU(subMenuCalibrarMotores, "Calibracion de Motores", doNothing, noEvent, wrapStyle,
     OP("Calibrar", calibrarMotores, enterEvent),
     SUBMENU(subMenuToggleEjeACalibrar),
     FIELD(cantVeces, "Cantidad de veces:", "", 0, 200, 10, 0, doNothing, noEvent, noStyle),
     FIELD(cantMm, "Cantidad de mm:", "", 0, 100, 10, 1, doNothing, noEvent, noStyle),
     FIELD(pasosPorMm, "Pasos por mm:", "", 1500, 1700, 10, 1, doNothing, noEvent, noStyle),
     EXIT("<- Volver"));

MENU(subMenuCalibrarPID, "Calibracion de PID", doNothing, noEvent, wrapStyle,
     OP("Calibrar PID", calibrarPID, enterEvent),
     FIELD(MULT, "Factor:", "", 0, 100, 10, 0.25, doNothing, noEvent, noStyle),
     FIELD(Kp, "Proporcional: ", "", 0, 50, 1, 0.25, doNothing, noEvent, noStyle),
     FIELD(Ki, "Integrador: ", "", 0, 50, 1, 0.25, doNothing, noEvent, noStyle),
     FIELD(Kd, "Derivador: ", "", 0, 50, 1, 0.25, doNothing, noEvent, noStyle),
     EXIT("<- Volver"));

MENU(subMenuCalibrar, "Menu de calibracion", doNothing, noEvent, wrapStyle,
     SUBMENU(subMenuCalibrarCelda),
     SUBMENU(subMenuCalibrarPID),
     SUBMENU(subMenuCalibrarMotores),
     EXIT("<- Volver"));

MENU(mainMenu, "SCRATCH TESTER 3000", doNothing, noEvent, wrapStyle,
     //  FIELD(Kd, "Derivador: ", "", 0, 50, 1, 5, showEvent("hola"), anyEvent, noStyle),
     // OP("Test", testPrefs, enterEvent),
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

    initMotors();
    initPreferences();

    clickEncoder.setAccelerationEnabled(true);
    clickEncoder.setDoubleClickEnabled(false);

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);

    gfx.init();
    gfx.setRotation(1);
    debugln("Initialized display");
    // gfx.fillScreen(TFT_BLACK);
    gfx.fillScreen(TFT_WHITE);
    gfx.setTextFont(1);
    gfx.setTextSize(2);

    debugln("done");

    nav.showTitle = true; // Show titles in the menus and submenus
    //  nav.timeOut = 60;  // Timeout after 60 seconds of inactivity
    //  nav.idleOn(); // Start with the main screen and not the menu

    pinMode(encBtn, INPUT_PULLUP);
    pinMode(stopSW, INPUT_PULLUP);
    attachInterrupt(stopSW, emergencyStopActivate, RISING);

    scale.begin(load, SCK);
    if (scale.wait_ready_retry(3, 500))
    {
        scale.set_scale(CALIBRATION_FACTOR / 9.8066);
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
    constexpr int menuFPS = 60;
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
    // samplea 10 veces y toma el promedio para ver el cero del joystick
    int suma = 0;
    int cantidad = 50;
    for (int i = 0; i < cantidad; i++)
    {
        suma += analogRead(joyX);
    }
    float bufferMaxX = (suma / cantidad) + buffer;
    float bufferMinX = (suma / cantidad) - buffer;
    suma = 0;
    for (int i = 0; i < cantidad; i++)
    {
        suma += analogRead(joyY);
    }
    float bufferMaxY = (suma / cantidad) + buffer;
    float bufferMinY = (suma / cantidad) - buffer;
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
                if (desired_speedX < 0)
                {
                    debugln("X negativo");
                    stepperX.moveTo(-1000000000);
                }
                else if (desired_speedX > 0)
                {
                    debugln("X positivo");
                    stepperX.moveTo(1000000000);
                }
            }
            if (not(joyY_value > bufferMinY && bufferMaxY > joyY_value))
            {
                if (desired_speedY < 0)
                {
                    debugln("Y negativo");
                    stepperY.moveTo(-1000000000);
                }
                else if (desired_speedY > 0)
                {
                    debugln("Y positivo");
                    stepperY.moveTo(1000000000);
                }
                debugf("\n");
            }
            last_input_time = current_time;
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
    float largoSteps = mm2step(largo);
    stepperX.setSpeed(mmxm2stepxs(velocidad));
    stepperX.move(largoSteps);
    stepperX.runToPosition();
    // volver a la posicion anterior
    stepperX.setSpeed(maxSpeedX);
    stepperX.move(-largoSteps);
    stepperX.runToPosition();

    mainMenu.dirty = true; // Force the main menu to redraw itself
    return proceed;
}

result medirProgreso()
{
    float largoSteps = mm2step(largo);
    stepperX.setSpeed(mmxm2stepxs(velocidad));
    // stepperX.setSpeed(10000000);
    stepperX.move(largoSteps);
    last_input_time = 0;
    gfx.fillScreen(Black);
    fuerzaPID.SetMode(AUTOMATIC);
    fuerzaPID.SetTunings(Kp, Ki, Kd); // los aplico por si los cambie en el menu
    fuerzaPID.SetOutputLimits(-5000, 5000);
    fuerzaSetpoint = fuerzaFinal / 10;
    while (digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }
        fuerzaInput = scale.get_units(1); // newton
        fuerzaPID.Compute();
        stepperY.setSpeed(fuerzaOutput);
        stepperY.runSpeed();
        gfx.drawFloat(fuerzaInput, 0, 100, 70, 1);
        double error = fuerzaSetpoint - fuerzaInput;
        if (error < TOL)
            break;
    }
    int deltaF = fuerzaFinal - fuerzaInicial;
    while (stepperX.distanceToGo() != 0)
    {
        if (emergencyStopCheck())
        {
            break;
        }
        float ratio = stepperX.currentPosition() / largoSteps;
        fuerzaSetpoint = fuerzaInicial + (deltaF * ratio);
        unsigned long current_time = millis();
        if (current_time - last_input_time > 500)
        {
            fex.drawProgressBar(20, 70, 200, 25, 100 * ratio, Red, White);
            last_input_time = current_time;
        }
        fuerzaInput = scale.get_units(1); // newton
        fuerzaPID.Compute();
        stepperY.setSpeed(fuerzaOutput);
        stepperY.runSpeed();
        gfx.drawFloat(fuerzaInput, 0, 100, 70, 1);
        stepperX.runSpeed();
        // stepperX.runSpeedToPosition();
        // debugln(stepperX.currentPosition());
    }
    debug("volviendo al origen");
    stepperX.setSpeed(maxSpeedX);
    stepperX.move(-largoSteps);
    stepperX.runToPosition();
    mainMenu.dirty = true;
    return proceed;
}

result homing()
{
    stepperX.setSpeed(maxSpeedX / 2.0);
    stepperX.move(-mm2step(300));
    while (digitalRead(stopSW))
    {
        if (emergencyStopCheck())
        {
            return proceed;
        }
        stepperX.runSpeed();
    }
    stepperX.move(mm2step(300));
    while (!digitalRead(stopSW))
    {
        if (emergencyStopCheck())
        {
            return proceed;
        }
        stepperX.runSpeed();
    }
    stepperX.setCurrentPosition(0);
    mainMenu.dirty = true;
    return proceed;
}

result calibrarMotores()
{
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
        while (stp.distanceToGo() != 0)
        {
            if (emergencyStopCheck())
            {
                break;
            }
            stp.run();
        }

        delay(1000);
        stp.move(-mm2step(cantMm));
        while (stp.distanceToGo() != 0)
        {
            if (emergencyStopCheck())
            {
                break;
            }
            stp.run();
        }
        delay(1000);
        debugln("loop 2");
    }
    mainMenu.dirty = true;
    return proceed;
}

result calibrarPID()
{
    gfx.fillScreen(TFT_BLACK);
    gfx.setCursor(120, 60);
    gfx.setTextColor(TFT_WHITE, TFT_BLACK);
    gfx.setTextSize(2);
    gfx.drawCentreString("Peso", 120, 30, 1);
    gfx.setTextPadding(100);

    gfx.drawRect(0, 0, 240, 135, TFT_WHITE);
    // scale.tare(10);
    fuerzaSetpoint = fuerzaFinal * 1000;

    fuerzaPID.SetMode(AUTOMATIC);
    fuerzaPID.SetOutputLimits(-5000, 5000);
    fuerzaPID.SetTunings(Kp, Ki, Kd);
    int n = 0;
    stepperY.setSpeed(MICROSTEP * 250);
    stepperY.setAcceleration(accelerationX * 10);
    while (digitalRead(joySW))
    // while (1)
    {
        if (emergencyStopCheck())
        {
            break;
        }
        // unsigned long current_time = millis();
        // if (current_time - last_input_time > 100)
        // {
        //     fuerzaInput = static_cast<double>(scale.get_units(1)); // newton
        //     fuerzaPID.Compute();
        //     float speed = fuerzaOutput * MULT;
        //     stepperY.move(speed);
        //     last_input_time = current_time;
        //     gfx.drawFloat(fuerzaInput, 0, 100, 70, 1);
        //     double error = fuerzaSetpoint - fuerzaInput;
        //     debugf("error= %f\toutput= %f\tspeed= %f\n", error, fuerzaOutput, speed);
        // }
        //
        // stepperY.run();
        fuerzaInput = scale.get_units(1); // newton
        gfx.drawFloat(fuerzaInput, 0, 100, 70, 1);
        fuerzaPID.Compute();
        stepperY.setSpeed(fuerzaOutput);
        stepperY.runSpeed();

        // debugf("input= %d\toutput= %d\n", fuerzaInput, fuerzaOutput);
        // if (error < TOL)
        // {
        //     n++;
        //     if (n = 50)
        //         break;
        // }
        // else
        // {
        //     n = 0;
        // }
    }
    mainMenu.dirty = true;
    // debugf("peso %f\n", reading);
    return proceed;
}

result calibrarCelda()
{
    // TODO dar opcion para cambiar la calibracion
    gfx.fillScreen(TFT_BLACK);
    gfx.setCursor(120, 60);
    gfx.setTextColor(TFT_WHITE, TFT_BLACK);
    gfx.setTextSize(2);
    gfx.drawCentreString("Peso", 120, 30, 1);
    gfx.setTextPadding(100);

    gfx.drawRect(0, 0, 240, 135, TFT_WHITE);

    while (digitalRead(joySW))
    {
        long reading =  scale.get_units(1);
        gfx.drawFloat(reading, 0, 100, 70, 1);
    }

    gfx.fillScreen(TFT_BLACK);
    gfx.setTextPadding(0);
    gfx.setTextSize(1);
    delay(1000);
    mainMenu.dirty = true;
    return proceed;
}

void initMotors()
{
    stepperX.setMaxSpeed(maxSpeedX);
    stepperX.setAcceleration(accelerationX);

    driverX.begin();
    driverX.rms_current(600);
    driverX.en_pwm_mode(1);
    driverX.pwm_autoscale(1);
    driverX.microsteps(MICROSTEP);

    stepperY.setMaxSpeed(maxSpeedX);
    stepperY.setAcceleration(accelerationX);
    stepperY.setPinsInverted(true, false, false);

    driverY.begin();
    driverY.rms_current(600);
    driverY.en_pwm_mode(1);
    driverY.pwm_autoscale(1);
    driverY.microsteps(MICROSTEP);

    stepperX.move(50 * MICROSTEP);
    stepperY.move(-50 * MICROSTEP);
    while (stepperX.distanceToGo() != 0 && stepperY.distanceToGo() != 0)
    {
        stepperX.run();
        stepperY.run();
    }
}

void initPreferences()
{
    prefs.begin("scratch");
    bool init = prefs.isKey("init");
    if (init == false)
    {
        prefs.putFloat("Kp", Kp);
        prefs.putFloat("Ki", Ki);
        prefs.putFloat("Kd", Kd);
        prefs.putFloat("TOL", TOL);
        prefs.putFloat("MULT", MULT);
        prefs.putFloat("CALIBRATIONFACTOR", CALIBRATION_FACTOR);
        prefs.putInt("fuerzaInicial", fuerzaInicial);
        prefs.putInt("ffuerzaFinal", fuerzaFinal);
        prefs.putInt("velocidad", velocidad);
        prefs.putInt("largo", largo);
        prefs.putInt("cantVeces", cantVeces);
        prefs.putInt("cantMm", cantMm);
        prefs.putInt("pasosPorMm", pasosPorMm);
        prefs.putInt("maxSpeedX", maxSpeedX);
        prefs.putInt("accelerationX", accelerationX);
        prefs.putInt("MICROSTEP", MICROSTEP);
        prefs.putInt("buffer", buffer);
        prefs.putBool("init", true);
    }
    else
    {
        Kp = prefs.getFloat("Kp");
        Ki = prefs.getFloat("Ki");
        Kd = prefs.getFloat("Kd");
        TOL = prefs.getFloat("TOL");
        MULT = prefs.getFloat("MULT");
        CALIBRATION_FACTOR = prefs.getFloat("CALIBRATIONFACTOR");
        fuerzaInicial = prefs.getInt("fuerzaInicial");
        fuerzaFinal = prefs.getInt("fuerzaFinal");
        velocidad = prefs.getInt("velocidad");
        largo = prefs.getInt("largo");
        cantVeces = prefs.getInt("cantVeces");
        cantMm = prefs.getInt("cantMm");
        pasosPorMm = prefs.getInt("pasosPorMm");
        maxSpeedX = prefs.getInt("maxSpeedX");
        accelerationX = prefs.getInt("accelerationX");
        MICROSTEP = prefs.getInt("MICROSTEP");
        buffer = prefs.getInt("buffer");
    }
}

// ESP32 timer
void IRAM_ATTR onTimer()
{
    clickEncoder.service();
}

void IRAM_ATTR emergencyStopActivate()
{
    unsigned long now = millis();

    if (now - lastStopTime > 100)
    {
        activateEmergencyStop = true;
        lastStopTime = now;
    }
}

bool emergencyStopCheck()
{
    if (activateEmergencyStop)
    {
        emergencyStop();
        return true;
    }
    else
    {
        return false;
    }
}

void emergencyStop()
{
    int speedX = stepperX.speed();
    stepperX.setAcceleration(accelerationX * 10);
    stepperX.stop();
    int speedY = stepperY.speed();
    stepperY.setAcceleration(accelerationX * 10);
    stepperY.stop();

    stepperX.setAcceleration(accelerationX);
    stepperY.setAcceleration(accelerationX);
    if (speedX < 0)
    {
        stepperX.move(mm2step(5));
    }
    else if (speedX > 0)
    {
        stepperX.move(mm2step(-5));
    }

    if (speedY < 0)
    {
        stepperY.move(mm2step(5));
    }
    else if (speedY > 0)
    {
        stepperY.move(mm2step(-5));
    }
    activateEmergencyStop = false;
}
