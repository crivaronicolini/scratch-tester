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
#include <FastAccelStepper.h>

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

// params celda
// 500g
// #define CALIBRATION_FACTOR 50100
// 100g
// #define CALIBRATION_FACTOR 10297
// 1000g
float CALIBRATION_FACTOR = -101.805;
long reading = -1;
int numSamples = 1;
bool calibrarCelda = false;

// Setup TFT colors.  Probably stop using these and use the colors defined by ArduinoMenu
#define BACKCOLOR TFT_BLACK
#define TEXTCOLOR TFT_WHITE

// params menu
int chooseField = 1;
int menuDelayTime = 100;

// params medicion
int fuerzaInicial = 0; // N
int fuerzaFinal = 1;   // N
int velocidad = 100;   // mm x minuto
int largo = 5;         // mm
float separacion = 0.2;

// params PID
double fuerzaSetpoint, fuerzaInput, fuerzaOutput;
double Kp = 0.02, Ki = 0, Kd = 0;
double eKp = 0.185, eKi = 0.019, eKd = 0.007;
int TOL = 120;

// params calibracion
int cantVeces = 2;
int cantMm = 5;
int pasosPorMm = 1600;

double Kpmin=0, Kpmax=0, Kpinc=0, Kimin=0, Kimax=0, Kiinc=0;

// params motores
// int maxSpeedX = 25000;
int MICROSTEP = 256;
// int maxSpeedX = 10000;
float mm2step(float mm) { return mm * MICROSTEP * 100; }
float step2mm(float step) { return step / (MICROSTEP * 100); }
float mmxm2stepxs(float mmxm) { return mmxm * MICROSTEP * 100 / 60; }
float stepxs2mmxm(float stepxs) { return stepxs * 60 / (MICROSTEP * 100); }

int maxSpeedX = mmxm2stepxs(100);
int accelerationX = 4 * maxSpeedX;
int maxSpeedJog = 2 * maxSpeedX;
int accelerationJog = 4 * maxSpeedJog;

// params tiempo
int INPUT_READ_INTERVAL = 100;
unsigned long last_input_time = 0;
unsigned long lastButtonPress = 0;
unsigned long lastStopTime = 0;

// params botones
int joySW_status = 1;
bool activateEmergencyStop = false;
int buffer = 200;
int test = 44;

// init graph
float f, O, dy;

PID fuerzaPID(&fuerzaInput, &fuerzaOutput, &fuerzaSetpoint, Kp, Ki, Kd, DIRECT);

TMC2130Stepper driverX = TMC2130Stepper(CS_PINX, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
TMC2130Stepper driverY = TMC2130Stepper(CS_PINY, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperX = NULL;
FastAccelStepper *stepperY = NULL;
// Declare the clickencoder
// Disable doubleclicks in setup makes the response faster.  See: https://github.com/soligen2010/encoder/issues/6
ClickEncoder clickEncoder = ClickEncoder(encB, encA, encBtn, encSteps);
ClickEncoderStream encStream(clickEncoder, 1.5);

// TFT gfx is what the ArduinoMenu TFT_eSPIOut.h is expecting
TFT_eSPI gfx = TFT_eSPI();
TFT_eFEX fex = TFT_eFEX(&gfx);
void drawGraph(int stage);
void updatePrefs(float value, const char* key);
void updatePrefs(int value, const char* key);
void updatePrefs(double value, const char* key);
void testPrefs();
result resetearConfig();
result despejar();
result medir();
result medircte();
result mapear();
result definirOrigen();
result homing();
result calibrarMotores();
result calibrarPID();
result gridSearch();
long leerCelda();
result toggleCalibracionCelda();
void initPreferences();
void initMotors();
bool emergencyStopCheck();
void emergencyStop();
void IRAM_ATTR emergencyStopActivate();
void IRAM_ATTR onTimer(); // Start the timer to read the clickEncoder every 1 ms

#define DEBUG 0
#define GRAPH 0
#define MONITOR 1

#if MONITOR == 1
#define monitor(x) Serial.print(x)
#define monitorln(x) Serial.println(x)
#define monitorf(...) Serial.printf(__VA_ARGS__)
#define monitorw(...) Serial.write(__VA_ARGS__)
#else
#define monitor(x) do { } while(0)
#define monitorln(x) do { } while(0)
#define monitorf(...) do { } while(0)
#endif

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(...) Serial.printf(__VA_ARGS__)
#else
#define debug(x) do { } while(0)
#define debugln(x) do { } while(0)
#define debugf(...) do { } while(0)
#endif

void showPath(navRoot &root)
{
    Serial.print("nav level:");
    Serial.print(root.level);
    Serial.print(" path:[");
    for (int n = 0; n <= root.level; n++)
    {
        Serial.print(n ? "," : "");
        Serial.print(root.path[n].sel);
    }
    Serial.println("]");
}
template <typename T>
class leadsField : public menuField<T>
{
public:
    using menuField<T>::menuField;
    Used printTo(navRoot &root, bool sel, menuOut &out, idx_t idx, idx_t len, idx_t panelNr = 0) override
    {
        menuField<T>::reflex = menuField<T>::target();
        prompt::printTo(root, sel, out, idx, len);
        bool ed = this == root.navFocus;
        out.print((root.navFocus == this && sel) ? (menuField<T>::tunning ? '>' : ':') : ' ');
        out.setColor(valColor, sel, menuField<T>::enabled, ed);
        // char buffer[] = "      ";
        // sprintf(buffer, "%03d", menuField<T>::reflex);
        out.print(menuField<T>::reflex);
        out.setColor(unitColor, sel, menuField<T>::enabled, ed);
        print_P(out, menuField<T>::units(), len);

        Serial.println(menuField<T>::reflex);
        return len;
    }
};

result showEvent(const char *aver, eventMask e, navNode &nav, prompt &item)
{
    
    Serial.println();
    Serial.println(aver);
    Serial.println("========");
    Serial.print("Event for target: 0x");
    Serial.println((long)nav.target, HEX);
    Serial.println((long)&nav.target);
    showPath(*nav.root);
    Serial.print(e);
    switch (e)
    {
    case noEvent: // just ignore all stuff
        Serial.println(" noEvent");
        break;
    case activateEvent: // this item is about to be active (system event)
        Serial.println(" activateEvent");
        break;
    case enterEvent: // entering navigation level (this menu is now active)
        Serial.println(" enterEvent");
        break;
    case exitEvent: // leaving navigation level
        Serial.println(" exitEvent");
        break;
    case returnEvent: // TODO:entering previous level (return)
        Serial.println(" returnEvent");
        break;
    case focusEvent: // element just gained focus
        Serial.println(" focusEvent");
        break;
    case blurEvent: // element about to lose focus
        Serial.println(" blurEvent");
        break;
    case selFocusEvent: // TODO:child just gained focus
        Serial.println(" selFocusEvent");
        break;
    case selBlurEvent: // TODO:child about to lose focus
        Serial.println(" selBlurEvent");
        break;
    case updateEvent: // Field value has been updated
        Serial.println(" updateEvent");
        break;
    case anyEvent:
        Serial.println(" anyEvent");
        break;
    }
    return proceed;
}

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
int toggleDummy = 0;
int toggleDummyDespejar = 0;

MENU(subMenuCalibrarCelda, "Calibracion de Celda de Carga", toggleCalibracionCelda,(eventMask)(enterEvent|exitEvent), wrapStyle,
     FIELD(reading, "F:","N",-1,100000,0,0,doNothing, noEvent, noStyle),
     FIELD(CALIBRATION_FACTOR, "F calibracion:", "", 1, 200, 10, 1, doNothing, noEvent, noStyle),
     FIELD(numSamples, "N muestras:","",1,100,1,0,doNothing, noEvent, noStyle),
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

TOGGLE(toggleDummy, subMenuToggleGridSearch, "Grid Search", doNothing, noEvent, noStyle,
        VALUE("", 1, gridSearch, enterEvent),
        VALUE("", 0, doNothing, noEvent));

TOGGLE(toggleDummy, subMenuToggleCalibrarPID, "Calibrar PID", doNothing, noEvent, noStyle,
        VALUE("", 1, calibrarPID, enterEvent),
        VALUE("", 0, doNothing, noEvent));

TOGGLE(toggleDummy, subMenuToggleMedir, "Medir", doNothing, noEvent, noStyle,
        VALUE("", 1, medir, enterEvent),
        VALUE("", 0, doNothing, noEvent));

TOGGLE(toggleDummy, subMenuToggleMedirCte, "Medir F cte", doNothing, noEvent, noStyle,
        VALUE("", 1, medircte, enterEvent),
        VALUE("", 0, doNothing, noEvent));

TOGGLE(toggleDummy, subMenuToggleMapear, "Mapear", doNothing, noEvent, noStyle,
        VALUE("", 1, mapear, enterEvent),
        VALUE("", 0, doNothing, noEvent));

MENU(subMenuGridSearch, "Grid Search", doNothing, noEvent, wrapStyle,
     SUBMENU(subMenuToggleGridSearch),
     altFIELD(decPlaces<3>::menuField,Kpmin, "Kp min", "", 0, 20, 0.01, 0.001, doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField,Kpmax, "Kp max", "", 0, 20, 0.01, 0.001, doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField,Kpinc, "Kp inc", "", 0, 20, 0.01, 0.001, doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField,Kimin, "Ki min", "", 0, 20, 0.01, 0.001, doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField,Kimax, "Ki max", "", 0, 20, 0.01, 0.001, doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField,Kiinc, "Ki inc", "", 0, 20, 0.01, 0.001, doNothing, noEvent, noStyle),
     EXIT("<- Volver"));

MENU(subMenuCalibrarPID, "Calibracion de PID", doNothing, noEvent, wrapStyle,
     SUBMENU(subMenuToggleCalibrarPID),
     SUBMENU(subMenuGridSearch),
     SUBMENU(subMenuToggleMedir),
     SUBMENU(subMenuToggleMapear),
     altFIELD(decPlaces<3>::menuField, Kp, "Proporcional: ", "", -5, 5, 0.01, 0.001, doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField, Ki, "Integrador: ", "", -5, 5, 0.01, 0.001, doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField, Kd, "Derivador: ", "", -5, 5, 0.01, 0.001, doNothing, noEvent, noStyle),
     EXIT("<- Volver"));

MENU(subMenuCalibrar, "Menu de calibracion", doNothing, noEvent, wrapStyle,
     SUBMENU(subMenuCalibrarCelda),
     SUBMENU(subMenuCalibrarMotores),
     FIELD(TOL, "Tolerancia", "", 0, 1000, 10, 1, doNothing, noEvent, noStyle),
     FIELD(separacion, "Separacion", "", 0, 5, 1, 0.1, doNothing, noEvent, noStyle),
     OP("Resetear configuracion", resetearConfig, enterEvent),
     EXIT("<- Volver"));

TOGGLE(toggleDummy, subMenuToggleDefinirOrigen,"Definir origen", doNothing, noEvent, noStyle,
        VALUE(" ON", 1, definirOrigen, enterEvent),
        VALUE("", 0, doNothing, noEvent));

TOGGLE(toggleDummyDespejar, subMenuToggleDespejar,"Despejar muestra", doNothing, noEvent, noStyle,
        VALUE(" ON", 1, despejar, enterEvent),
        VALUE("", 0, doNothing, noEvent));

MENU(mainMenu, "SCRATCH TESTER 3000", doNothing, noEvent, wrapStyle,
     SUBMENU(subMenuCalibrarPID),
     FIELD(fuerzaInicial, "Fuerza inicial:", "N", 0, 200, 10, 1, doNothing, noEvent, noStyle),
     FIELD(fuerzaFinal, "Fuerza final:", "N", 0, 200, 10, 1, doNothing, noEvent, noStyle),
     FIELD(velocidad, "Velocidad:", "mm/s", 0, 200, 10, 1, doNothing, noEvent, noStyle),
     FIELD(largo, "Largo:", "mm", 0, 20, 1, 1, doNothing, noEvent, noStyle),
     SUBMENU(subMenuToggleDefinirOrigen),
     SUBMENU(subMenuToggleMedir),
     SUBMENU(subMenuToggleMedirCte),
     SUBMENU(subMenuToggleDespejar),
     // OP("Homing", homing, enterEvent),
     SUBMENU(subMenuCalibrar));

const panel panels[] MEMMODE = {{0, 0, GFX_WIDTH / fontW, GFX_HEIGHT / fontH}}; // Main menu panel
navNode *nodes[sizeof(panels) / sizeof(panel)];                                 // navNodes to store navigation status
panelsList pList(panels, nodes, sizeof(panels) / sizeof(panel));                // a list of panels and nodes
// idx_t tops[MAX_DEPTH]={0,0}; // store cursor positions for each level
idx_t eSpiTops[MAX_DEPTH] = {0};
TFT_eSPIOut eSpiOut(gfx, colors, eSpiTops, pList, fontW, fontH + 1);
idx_t serialTops[MAX_DEPTH] = {0};
serialOut outSerial(Serial, serialTops);
#if DEBUG == 1
    menuOut *constMEM outputs[] MEMMODE = {&outSerial, &eSpiOut};  // list of output devices
#else
    menuOut *constMEM outputs[] MEMMODE = {&eSpiOut};  // list of output devices
#endif
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

    clickEncoder.setAccelerationEnabled(false);
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
        if (calibrarCelda){
            reading = leerCelda();
        }
        lastMenuFrame = millis();
        nav.poll();
        // monitorf("$%f,%d",1.0);
        // monitorf("%f,%d\n",9.0, 3);
        // Serial.println(9);
    }
}

result definirOrigen()
{
    // samplea 10 veces y toma el promedio para ver el cero del joystick
    nav.poll(); // para que aparezca el ON en el menu
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
    stepperX->setAcceleration(accelerationJog);
    stepperY->setAcceleration(accelerationJog);

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
            int desired_speedX = map(joyX_value, 0, 4095, -maxSpeedJog, maxSpeedJog);
            int desired_speedY = - map(joyY_value, 0, 4095, -maxSpeedJog, maxSpeedJog);

            // Based on the input, set targets and max speed
            stepperX->setSpeedInHz(abs(desired_speedX));
            stepperY->setSpeedInHz(abs(desired_speedY));
            debugf("X %d; Y %d\n", joyX_value, joyY_value);

            if (not(joyX_value > bufferMinX && bufferMaxX > joyX_value))
            {
                if (desired_speedX < 0)
                {
                    // debugln("X negativo");
                    stepperX->runBackward();
                }
                else if (desired_speedX > 0)
                {
                    // debugln("X positivo");
                    stepperX->runForward();
                }
            }
            if (not(joyY_value > bufferMinY && bufferMaxY > joyY_value))
            {
                if (desired_speedY < 0)
                {
                    // debugln("Y negativo");
                    stepperY->runBackward();
                }
                else if (desired_speedY > 0)
                {
                    // debugln("Y positivo");
                    stepperY->runForward();
                }
                // debugf("\n");
            }
            if (joyY_value > bufferMinY && bufferMaxY > joyY_value)
            {
                stepperY->stopMove();
            }
            if (joyX_value > bufferMinX && bufferMaxX > joyX_value)
            {
                stepperX->stopMove();
            }
            last_input_time = current_time;
        }
    }
    stepperY->stopMove();
    stepperX->stopMove();
    return proceed;
}

result despejar()
{ //nueve los ejes una distancia prudente para sacar la muestra
    nav.poll();
    stepperY->setSpeedInHz(maxSpeedJog);
    stepperY->setAcceleration(accelerationJog);
    stepperY->move(mm2step(-10), true);

    toggleDummyDespejar = 0;
    return proceed;
}

result mapear()
{
    updatePrefs(velocidad,"velocidad");
    updatePrefs(largo,"largo");

    // ACERCAMIENTO
    stepperX->setCurrentPosition(0);
    stepperY->setSpeedInHz(maxSpeedX*fuerzaInicial/20);
    stepperY->setAcceleration(accelerationX * 10);

    drawGraph(1);
    drawGraph(6);
    int x = 3;

    float largoSteps = mm2step(largo);

    double altura = 0;

    stepperY->runForward();
    while (digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }
        fuerzaInput = scale.get_units(numSamples); // newton
        if (fuerzaInput>50){ stepperY->forceStop();break;}
        altura = fuerzaInput;
        unsigned long current_time = millis();
        monitorf("%d\t%d\t%d\t%f\t%f\t%f\n", current_time, stepperX->getCurrentPosition(), stepperY->getCurrentPosition(), step2mm(stepperX->getCurrentPosition()), step2mm(stepperY->getCurrentPosition()), altura);

        #if GRAPH == 1
            if (current_time - last_input_time > 50)
            {
                    gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
                    float i = map(fuerzaInput, 0, 1.2*fuerzaFinalM, 130, 18);
                    gfx.drawPixel(x, i, TFT_YELLOW);
                    x++;
                    if (x==240) {
                        x=3;
                        drawGraph(6);
                    }
                    last_input_time = current_time;
            }
        #endif

    }


    x = 3;
    int n = 0;
    int m = 0;

    // ESTABILIZACION
    stepperY->setCurrentPosition(mm2step(0.2));
    stepperY->forceStop();
    delay(100);
    float ceroCelda = scale.get_units(10);

    while (m<50 && digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }
        fuerzaInput = scale.get_units(numSamples); // newton
        altura = fuerzaInput - ceroCelda;
        unsigned long current_time = millis();
        monitorf("%d\t%d\t%d\t%f\t%f\t%f\n", current_time, stepperX->getCurrentPosition(), stepperY->getCurrentPosition(), step2mm(stepperX->getCurrentPosition()), step2mm(stepperY->getCurrentPosition()), altura);
        m++;
        #if GRAPH == 1
            if (current_time - last_input_time > 50)
            {
                    gfx.drawFloat(altura/1000.0, 3, 50, 1, 1);
                    float a = map(altura, -1000, 1000, 130, 18);
                    gfx.drawPixel(x, a, TFT_GREEN);
                    x++;
                    if (x==240) {
                        x=3;
                        drawGraph(6);
                    }
                last_input_time = current_time;
            }
        #endif

    }


    x = 3;
    n = 0;

    // MEDICION
    drawGraph(3);
    drawGraph(4);
    drawGraph(6);

    stepperX->setCurrentPosition(0);
    stepperX->setSpeedInHz(mmxm2stepxs(velocidad));
    stepperX->move(largoSteps);

    while (digitalRead(joySW) && stepperX->isRunning())
    {
        if (emergencyStopCheck())
        {
            break;
        }
        fuerzaInput = scale.get_units(numSamples); // newton
        altura = fuerzaInput - ceroCelda;
        unsigned long current_time = millis();
        monitorf("%d\t%d\t%d\t%f\t%f\t%f\n", current_time, stepperX->getCurrentPosition(), stepperY->getCurrentPosition(), step2mm(stepperX->getCurrentPosition()), step2mm(stepperY->getCurrentPosition()), altura);
        #if GRAPH == 1
            if (current_time - last_input_time > 50)
            {
                gfx.drawFloat(altura/1000.0, 3, 50, 1, 1);

                float a = map(altura, -1000, 1000, 130, 18);
                gfx.drawPixel(x, a, TFT_GREEN);
                x++;
                if (x==240) {
                    x=3;
                    drawGraph(6);
                }
                last_input_time = current_time;
            }
        #endif

    }

    if (not (stepperX->isRunning())) { toggleDummy = 0; }
    stepperX->forceStop();
    stepperY->forceStop();

    stepperX->setSpeedInHz(maxSpeedX);
    stepperY->setSpeedInHz(maxSpeedX);

    stepperY->moveTo(0);
    delay(20);
    stepperX->moveTo(0);
    nav.refresh();
    mainMenu.dirty = true;
    return proceed;
}

result stepImpulse()
{
    updatePrefs(Kp,"Kp");
    updatePrefs(Kd,"Kd");
    updatePrefs(Ki,"Ki");
    updatePrefs(velocidad,"velocidad");
    updatePrefs(fuerzaInicial,"fuerzaInicial");
    updatePrefs(fuerzaFinal,"fuerzaFinal");
    updatePrefs(largo,"largo");

    fuerzaSetpoint = fuerzaInicial * 1000;
    if (fuerzaInicial == 0) {fuerzaSetpoint=100;}

    int fuerzaFinalM = fuerzaFinal * 1000;
    int fuerzaInicialM = fuerzaInicial * 1000;

    fuerzaPID.SetMode(AUTOMATIC);
    fuerzaPID.SetOutputLimits(-maxSpeedX, maxSpeedX);
    fuerzaPID.SetSampleTime(50);
    fuerzaPID.SetTunings(Kp*(velocidad/2), Ki, Kd);

    // ACERCAMIENTO
    stepperY->setSpeedInHz(maxSpeedX*fuerzaInicial/20);
    stepperY->setAcceleration(accelerationX * 10);

    drawGraph(1);
    drawGraph(6);
    int x = 3;

    stepperY->runForward();
    while (digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }
        fuerzaInput = scale.get_units(numSamples); // newton
        if (fuerzaInput>50){ stepperY->forceStop();break;}
        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
            float i = map(fuerzaInput, 0, 1.2*fuerzaFinalM, 130, 18);
            // monitorf("%d\t%d\t%f\t%f\n",current_time,stepperX->getCurrentPosition(), fuerzaInput, 0.0);
            gfx.drawPixel(x, i, TFT_YELLOW);
            x++;
            if (x==240) {
                x=3;
                drawGraph(6);
            }
            last_input_time = current_time;
        }
    }


    x = 3;
    int n = 0;
    int m = 0;

    // ESTABILIZACION
    stepperY->setCurrentPosition(mm2step(0.5));

    drawGraph(2);
    drawGraph(4);
    drawGraph(6);

    while (m<50)
    {
        if (emergencyStopCheck())
        {
            break;
        }

        // 0.2 aver set speed y runForward
        // funciona con kp=0.02, pero converge muy lento
        // lo estoy probando denuevo, por ahi le puedo subir el kp y meterle I D
        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            fuerzaInput = scale.get_units(numSamples); // newton
            fuerzaPID.Compute();
            if (fuerzaOutput>0.2){
                stepperY->setSpeedInHz(fuerzaOutput);
                stepperY->runForward();
            } else if (fuerzaOutput<-0.2){
                stepperY->setSpeedInHz(-fuerzaOutput);
                stepperY->runBackward();
            }
            else { stepperY->stopMove();}
            double error = fuerzaSetpoint - fuerzaInput;
            monitorf("%d\t%d\t%f\t%f\t%f\t%f\n",current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint);
            gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
            gfx.drawFloat(fuerzaOutput, 0, 162, 1, 1);

            float i = map(fuerzaInput, 0, 1.2*fuerzaFinalM, 130, 18);
            float s = map(fuerzaSetpoint,0, 1.2*fuerzaFinalM, 130, 18);
            float o = map(fuerzaOutput, -100, 100, 130, 18);
            gfx.drawPixel(x, i, TFT_YELLOW);
            gfx.drawPixel(x, s, TFT_GREEN);
            gfx.drawPixel(x, o, TFT_RED);
            x++;
            m++;
            if (x==240) {
                x=3;
                drawGraph(6);
            }
            last_input_time = current_time;
        }
    }

    x = 3;
    n = 0;

    // MEDICION
    // subo el setpoint de golpe al maximo
    drawGraph(3);
    drawGraph(4);
    drawGraph(6);

    float largoSteps = mm2step(largo);
    stepperX->setCurrentPosition(0);
    stepperX->setSpeedInHz(mmxm2stepxs(velocidad));
    stepperX->move(largoSteps);

    fuerzaSetpoint = fuerzaFinalM;
    while (digitalRead(joySW) && stepperX->isRunning())
    {
        if (emergencyStopCheck())
        {
            break;
        }

        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            fuerzaInput = scale.get_units(numSamples); // newton
            fuerzaPID.Compute();
            if (fuerzaOutput>0.2){
                stepperY->setSpeedInHz(fuerzaOutput);
                stepperY->runForward();
            } else if (fuerzaOutput<-0.2){
                stepperY->setSpeedInHz(-fuerzaOutput);
                stepperY->runBackward();
            }
            else { stepperY->stopMove();}
            double error = fuerzaSetpoint - fuerzaInput;
            monitorf("%d\t%d\t%f\t%f\t%f\t%f\n",current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint);
            gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
            gfx.drawFloat(fuerzaOutput, 0, 162, 1, 1);
            // gfx.drawFloat(fuerzaSetpoint, 3, 170, 30, 1);

            float i = map(fuerzaInput, 0, 1.2*fuerzaFinalM, 130, 18);
            float s = map(fuerzaSetpoint,0, 1.2*fuerzaFinalM, 130, 18);
            float o = map(fuerzaOutput, -100, 100, 130, 18);
            float e = map(error, -1000, 1000, 130, 18);
            gfx.drawPixel(x, s, TFT_GREEN);
            gfx.drawPixel(x, i, TFT_YELLOW);
            gfx.drawPixel(x, o, TFT_RED);
            gfx.drawPixel(x, e, TFT_BLUE);
            x++;
            if (x==240) {
                x=3;
                drawGraph(6);
            }
            last_input_time = current_time;
        }

    }
    if (not (stepperX->isRunning())) { toggleDummy = 0; }
    stepperX->forceStop();
    stepperY->forceStop();

    stepperX->setSpeedInHz(maxSpeedX);
    stepperY->setSpeedInHz(maxSpeedX);

    stepperY->moveTo(0);
    delay(20);
    stepperX->moveTo(0);
    nav.refresh();
    mainMenu.dirty = true;
    return proceed;
}

result gridSearch()
{
    updatePrefs(Kpmin, "Kpmin");
    updatePrefs(Kpmax, "Kpmax");
    updatePrefs(Kpinc, "Kpinc");
    updatePrefs(Kimin, "Kimin");
    updatePrefs(Kimax, "Kimax");
    updatePrefs(Kiinc, "Kiinc");
    int i = 0;
    for (Kp=Kpmin;Kp<Kpmax;Kp+=Kpinc)
    {
        for (Ki=Kimin;Ki<Kimax;Ki+=Kiinc)
        { i++;
        }
    }
    int n = i;
    i = 0;
    for (Kp=Kpmin;Kp<Kpmax;Kp+=Kpinc)
    {
        for (Ki=Kimin;Ki<Kimax;Ki+=Kiinc)
        {
            medir();
            i++;
            gfx.fillScreen(Black);
            gfx.drawString("Kp", 1, 30, 1);
            gfx.drawFloat(Kp, 3, 40, 30, 1);
            gfx.drawString("Ki", 1, 50, 1);
            gfx.drawFloat(Ki, 3, 40, 50, 1);
            gfx.drawFloat(i, 0, 1, 70, 1);
            gfx.drawString("/", 30, 70, 1);
            gfx.drawFloat(n, 0, 45, 70, 1);
            while (stepperX->isRunning()) {
                delay(50);
            }
        }
    }
    return proceed;

}

result medircte()
{
    updatePrefs(Kp,"Kp");
    updatePrefs(Kd,"Kd");
    updatePrefs(Ki,"Ki");
    updatePrefs(velocidad,"velocidad");
    updatePrefs(fuerzaInicial,"fuerzaInicial");
    updatePrefs(fuerzaFinal,"fuerzaFinal");
    updatePrefs(largo,"largo");

    fuerzaSetpoint = -100 + fuerzaInicial * 1000;
    float speed = maxSpeedX*fuerzaInicial/40;
    if (fuerzaInicial == 0) {
        speed = 800; // si fuerza inicial es cero no le gusta al set speed
    }
    int fuerzaFinalM = fuerzaFinal * 1000;
    int fuerzaInicialM = -100 + fuerzaInicial * 1000;

    int deltaF = fuerzaFinal - fuerzaInicial;
    int deltaFM = fuerzaFinalM - fuerzaInicialM;
    float largoSteps = mm2step(largo);
    double error = 0;

    // float norm = deltaF*largo*velocidad/(5*5*20); // normalizacion: el factor es 1 para los parametros de calibracion
    float norm = 1;
    int errAbs = 0;

    fuerzaPID.SetOutputLimits(-maxSpeedX, maxSpeedX);
    fuerzaPID.SetSampleTime(50);
    fuerzaPID.SetTunings(Kp*norm, Ki*norm, Kd*norm);
    // fuerzaPID.SetTunings(eKp*norm, eKi*norm, eKd*norm);

    // ACERCAMIENTO
    stepperX->setCurrentPosition(0);

    stepperY->setSpeedInHz(speed);
    stepperY->setAcceleration(accelerationX * 10);
    drawGraph(1);
    drawGraph(6);
    int x = 3;
    // monitorln("===");
    // monitorf("deltaF= %d, Kp= %f, Kpn= %f, Ki= %f, Kd= %f, fI= %d\n",deltaF,Kp,Kp*norm,Ki, Kd, fuerzaInicial);
    // monitorf("%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n", "t", "x", "error", "fOut", "fIn", "fSet", "y", "errAcc");

    stepperY->runForward();
    while (digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }
        fuerzaInput = scale.get_units(numSamples); // newton
        if (fuerzaInput>50){ stepperY->forceStop();break;}
        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            // monitorf("%d\t%d\t%f\t%f\n",current_time,stepperX->getCurrentPosition(), fuerzaInput, 0.0);
            error = fuerzaSetpoint - fuerzaInput;
            // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t\n",
            monitorf("%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%d,%d\n",
                    current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint, stepperY->getCurrentPosition(), errAbs,
                    fuerzaInicial, deltaF, norm, Kp, Ki, Kd, largo, velocidad);
            #if GRAPH == 1
                gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
                float i = map(fuerzaInput, 0, 1.2*fuerzaFinalM, 130, 18);
                gfx.drawPixel(x, i, TFT_YELLOW);
                x++;

                if (x==240) {
                    x=3;
                    drawGraph(6);
                }
            #endif
            last_input_time = current_time;
        }
    }


    stepperY->setCurrentPosition(mm2step(0.5));
    x = 3;
    int n = 0;
    TOL = 100 + (TOL*deltaF / 5);

    // ESTABILIZACION

    drawGraph(2);
    drawGraph(4);
    drawGraph(6);

    stepperY->setSpeedInHz(speed/4);
    stepperY->runForward();
    while (digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }
        fuerzaInput = scale.get_units(numSamples); // newton
        error = fuerzaSetpoint - fuerzaInput;
        // para ver que estamos en la fuerza inicial
        // if ((-TOL < error) && (error < TOL))
        if (error < TOL) // menos Tol porque quiero que corte TOL arriba del setpoint
        {
            stepperY->forceStop();
            break;
        }

        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            // fuerzaPID.Compute();
            // if (fuerzaOutput>0){
            //     stepperY->setSpeedInHz(fuerzaOutput);
            //     stepperY->runForward();
            // } else if (fuerzaOutput<0){
            //     stepperY->setSpeedInHz(-fuerzaOutput);
            //     stepperY->runBackward();
            // }
            // else { stepperY->stopMove();}
            errAbs += abs(error)/10;
            // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\n",current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint, stepperY->getCurrentPosition(), errAbs);
            // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t\n",
            monitorf("%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%d,%d\n",
                    current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint, stepperY->getCurrentPosition(), errAbs,
                    fuerzaInicial, deltaF, norm, Kp, Ki, Kd, largo, velocidad);

            #if GRAPH == 1
                gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
                gfx.drawFloat(fuerzaOutput, 0, 162, 1, 1);

                float i = map(fuerzaInput, 0, 1.2*fuerzaFinalM, 130, 18);
                float s = map(fuerzaSetpoint,0, 1.2*fuerzaFinalM, 130, 18);
                float o = map(fuerzaOutput, -100, 100, 130, 18);
                gfx.drawPixel(x, i, TFT_YELLOW);
                gfx.drawPixel(x, s, TFT_GREEN);
                gfx.drawPixel(x, o, TFT_RED);
                x++;
                if (x==240) {
                    x=3;
                    drawGraph(6);
                }
            #endif

            last_input_time = current_time;

        }
    }
    // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t\n",
    monitorf("%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%d,%d\n",
            0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    int errAbsEstabilizacion = errAbs;
    errAbs =0;
    x = 3;
    n = 0;

    // MEDICION
    drawGraph(3);
    drawGraph(4);
    drawGraph(6);

    fuerzaPID.SetMode(AUTOMATIC);
    stepperX->setSpeedInHz(mmxm2stepxs(velocidad));
    stepperX->move(largoSteps);

    while (digitalRead(joySW) && stepperX->isRunning())
    {
        if (emergencyStopCheck())
        {
            break;
        }

        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            fuerzaInput = scale.get_units(numSamples); // newton
            fuerzaPID.Compute();
            if (fuerzaOutput>0){
                stepperY->setSpeedInHz(fuerzaOutput);
                stepperY->runForward();
            } else if (fuerzaOutput<0){
                stepperY->setSpeedInHz(-fuerzaOutput);
                stepperY->runBackward();
            }
            else { stepperY->stopMove();}
            error = fuerzaSetpoint - fuerzaInput;
            errAbs += abs(error)/10;
            // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\n",current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint, stepperY->getCurrentPosition(), errAbs);
            // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t\n",
            monitorf("%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%d,%d\n",
                    current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint, stepperY->getCurrentPosition(), errAbs,
                    fuerzaInicial, deltaF, norm, Kp, Ki, Kd, largo, velocidad);

            #if GRAPH == 1
                gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
                gfx.drawFloat(fuerzaOutput, 0, 162, 1, 1);
                // gfx.drawFloat(fuerzaSetpoint, 3, 170, 30, 1);

                float i = map(fuerzaInput, 0, 1.2*fuerzaFinalM, 130, 18);
                float s = map(fuerzaSetpoint,0, 1.2*fuerzaFinalM, 130, 18);
                float o = map(fuerzaOutput, -100, 100, 130, 18);
                float e = map(error, -1000, 1000, 130, 18);
                gfx.drawPixel(x, s, TFT_GREEN);
                gfx.drawPixel(x, i, TFT_YELLOW);
                gfx.drawPixel(x, o, TFT_RED);
                gfx.drawPixel(x, e, TFT_BLUE);
                x++;
                if (x==240) {
                    x=3;
                    drawGraph(6);
                }
            #endif
            last_input_time = current_time;
        }

    }
    // monitorf("errAbsEstabilizacion= %d, errAbs= %d\n", errAbsEstabilizacion, errAbs);
    if (not (stepperX->isRunning())) { toggleDummy = 0; }
    stepperX->forceStop();
    stepperY->forceStop();

    stepperX->setSpeedInHz(2*maxSpeedX);
    stepperY->setSpeedInHz(2*maxSpeedX);

    stepperY->moveTo(0);
    delay(20);
    stepperX->moveTo(0);
    nav.refresh();
    fuerzaPID.SetMode(MANUAL);
    mainMenu.dirty = true;
    return proceed;
}

result medir()
{
    updatePrefs(Kp,"Kp");
    updatePrefs(Kd,"Kd");
    updatePrefs(Ki,"Ki");
    updatePrefs(velocidad,"velocidad");
    updatePrefs(fuerzaInicial,"fuerzaInicial");
    updatePrefs(fuerzaFinal,"fuerzaFinal");
    updatePrefs(largo,"largo");

    fuerzaSetpoint = -100 + fuerzaInicial * 1000;
    float speed = maxSpeedX*fuerzaInicial/40;
    if (fuerzaInicial == 0) {
        speed = 800; // si fuerza inicial es cero no le gusta al set speed
    }
    int fuerzaFinalM = fuerzaFinal * 1000;
    int fuerzaInicialM = -100 + fuerzaInicial * 1000;

    float ratio = 0;
    int deltaF = fuerzaFinal - fuerzaInicial;
    int deltaFM = fuerzaFinalM - fuerzaInicialM;
    float largoSteps = mm2step(largo);
    double error = 0;

    // float norm = deltaF*largo*velocidad/(5*5*20); // normalizacion: el factor es 1 para los parametros de calibracion
    float norm = 1;
    int errAbs = 0;

    fuerzaPID.SetOutputLimits(-maxSpeedX, maxSpeedX);
    fuerzaPID.SetSampleTime(50);
    fuerzaPID.SetTunings(Kp*norm, Ki*norm, Kd*norm);
    // fuerzaPID.SetTunings(eKp*norm, eKi*norm, eKd*norm);

    // ACERCAMIENTO
    stepperX->setCurrentPosition(0);

    stepperY->setSpeedInHz(speed);
    stepperY->setAcceleration(accelerationX * 10);
    drawGraph(1);
    drawGraph(6);
    int x = 3;
    // monitorln("===");
    // monitorf("deltaF= %d, Kp= %f, Kpn= %f, Ki= %f, Kd= %f, fI= %d\n",deltaF,Kp,Kp*norm,Ki, Kd, fuerzaInicial);
    // monitorf("%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n", "t", "x", "error", "fOut", "fIn", "fSet", "y", "errAcc");

    stepperY->runForward();
    while (digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }
        fuerzaInput = scale.get_units(numSamples); // newton
        if (fuerzaInput>50){ stepperY->forceStop();break;}
        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            // monitorf("%d\t%d\t%f\t%f\n",current_time,stepperX->getCurrentPosition(), fuerzaInput, 0.0);
            error = fuerzaSetpoint - fuerzaInput;
            // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t\n",
            monitorf("%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%d,%d\n",
                    current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint, stepperY->getCurrentPosition(), errAbs,
                    fuerzaInicial, deltaF, norm, Kp, Ki, Kd, largo, velocidad);
            #if GRAPH == 1
                gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
                float i = map(fuerzaInput, 0, 1.2*fuerzaFinalM, 130, 18);
                gfx.drawPixel(x, i, TFT_YELLOW);
                x++;

                if (x==240) {
                    x=3;
                    drawGraph(6);
                }
            #endif
            last_input_time = current_time;
        }
    }


    stepperY->setCurrentPosition(mm2step(0.5));
    x = 3;
    int n = 0;
    TOL = 100 + (TOL*deltaF / 5);

    // ESTABILIZACION

    monitorf("%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%d,%d\n",
            0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    drawGraph(2);
    drawGraph(4);
    drawGraph(6);

    stepperY->setSpeedInHz(speed/4);
    stepperY->runForward();
    while (digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }
        fuerzaInput = scale.get_units(numSamples); // newton
        error = fuerzaSetpoint - fuerzaInput;
        // para ver que estamos en la fuerza inicial
        // if ((-TOL < error) && (error < TOL))
        if (fuerzaInput>fuerzaSetpoint*1.1){ stepperY->forceStop();break;}
        if (error < TOL) // menos Tol porque quiero que corte TOL arriba del setpoint
        {
            stepperY->forceStop();
            break;
        }

        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            // fuerzaPID.Compute();
            // if (fuerzaOutput>0){
            //     stepperY->setSpeedInHz(fuerzaOutput);
            //     stepperY->runForward();
            // } else if (fuerzaOutput<0){
            //     stepperY->setSpeedInHz(-fuerzaOutput);
            //     stepperY->runBackward();
            // }
            // else { stepperY->stopMove();}
            errAbs += abs(error)/10;
            // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\n",current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint, stepperY->getCurrentPosition(), errAbs);
            // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t\n",
            monitorf("%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%d,%d\n",
                    current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint, stepperY->getCurrentPosition(), errAbs,
                    fuerzaInicial, deltaF, norm, Kp, Ki, Kd, largo, velocidad);

            #if GRAPH == 1
                gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
                gfx.drawFloat(fuerzaOutput, 0, 162, 1, 1);

                float i = map(fuerzaInput, 0, 1.2*fuerzaFinalM, 130, 18);
                float s = map(fuerzaSetpoint,0, 1.2*fuerzaFinalM, 130, 18);
                float o = map(fuerzaOutput, -100, 100, 130, 18);
                gfx.drawPixel(x, i, TFT_YELLOW);
                gfx.drawPixel(x, s, TFT_GREEN);
                gfx.drawPixel(x, o, TFT_RED);
                x++;
                if (x==240) {
                    x=3;
                    drawGraph(6);
                }
            #endif

            last_input_time = current_time;

        }
    }
    // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t\n",
    monitorf("%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%d,%d\n",
            0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    int errAbsEstabilizacion = errAbs;
    errAbs =0;
    x = 3;
    n = 0;

    // MEDICION
    drawGraph(3);
    drawGraph(4);
    drawGraph(6);

    fuerzaPID.SetMode(AUTOMATIC);
    stepperX->setSpeedInHz(mmxm2stepxs(velocidad));
    stepperX->move(largoSteps);

    while (digitalRead(joySW) && stepperX->isRunning())
    {
        if (emergencyStopCheck())
        {
            break;
        }
        ratio = stepperX->getCurrentPosition() / largoSteps;
        fuerzaSetpoint = fuerzaInicialM + (deltaFM * ratio);

        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            fuerzaInput = scale.get_units(numSamples); // newton
            fuerzaPID.Compute();
            if (fuerzaOutput>0){
                stepperY->setSpeedInHz(fuerzaOutput);
                stepperY->runForward();
            } else if (fuerzaOutput<0){
                stepperY->setSpeedInHz(-fuerzaOutput);
                stepperY->runBackward();
            }
            else { stepperY->stopMove();}
            error = fuerzaSetpoint - fuerzaInput;
            errAbs += abs(error)/10;
            // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\n",current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint, stepperY->getCurrentPosition(), errAbs);
            // monitorf("%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t\n",
            monitorf("%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%d,%d\n",
                    current_time,stepperX->getCurrentPosition(), error, fuerzaOutput, fuerzaInput, fuerzaSetpoint, stepperY->getCurrentPosition(), errAbs,
                    fuerzaInicial, deltaF, norm, Kp, Ki, Kd, largo, velocidad);

            #if GRAPH == 1
                gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
                gfx.drawFloat(fuerzaOutput, 0, 162, 1, 1);
                // gfx.drawFloat(fuerzaSetpoint, 3, 170, 30, 1);

                float i = map(fuerzaInput, 0, 1.2*fuerzaFinalM, 130, 18);
                float s = map(fuerzaSetpoint,0, 1.2*fuerzaFinalM, 130, 18);
                float o = map(fuerzaOutput, -100, 100, 130, 18);
                float e = map(error, -1000, 1000, 130, 18);
                gfx.drawPixel(x, s, TFT_GREEN);
                gfx.drawPixel(x, i, TFT_YELLOW);
                gfx.drawPixel(x, o, TFT_RED);
                gfx.drawPixel(x, e, TFT_BLUE);
                x++;
                if (x==240) {
                    x=3;
                    drawGraph(6);
                }
            #endif
            last_input_time = current_time;
        }

    }
    // monitorf("errAbsEstabilizacion= %d, errAbs= %d\n", errAbsEstabilizacion, errAbs);
    if (not (stepperX->isRunning())) { toggleDummy = 0; }
    stepperX->forceStop();
    stepperY->forceStop();

    stepperX->setSpeedInHz(2*maxSpeedX);
    stepperY->setSpeedInHz(2*maxSpeedX);

    stepperY->moveTo(0);
    delay(20);
    stepperX->moveTo(0);
    nav.refresh();
    fuerzaPID.SetMode(MANUAL);
    mainMenu.dirty = true;
    return proceed;
}


result homing()
{
    // stepperX.setSpeed(maxSpeedX / 2.0);
    // stepperX.move(-mm2step(300));
    while (digitalRead(stopSW))
    {
        if (emergencyStopCheck())
        {
            return proceed;
        }
        // stepperX.runSpeed();
    }
    // stepperX.move(mm2step(300));
    while (!digitalRead(stopSW))
    {
        if (emergencyStopCheck())
        {
            return proceed;
        }
        // stepperX.runSpeed();
    }
    // stepperX.setCurrentPosition(0);
    mainMenu.dirty = true;
    return proceed;
}

result calibrarMotores()
{
    FastAccelStepper *stp;
    if (ejeACalibrar == 1)
    {
        stp = stepperX;
    }
    else
    {
        stp = stepperY;
    }
    stp->setSpeedInHz(mmxm2stepxs(velocidad));
    debugf("cant veces %d, pasosxmm %d, cantmm %d mm %d step\n", cantVeces, pasosPorMm, cantMm, mm2step(cantMm));
    debugf("velocidad %d mmxm, %f pxs\n", velocidad, mmxm2stepxs(velocidad));
    stp->setAcceleration(4 * maxSpeedX);

    for (int i = 0; i < cantVeces; i++)
    {
        stp->move(mm2step(cantMm));
        delay(1000);
        stp->move(-mm2step(cantMm));
        delay(1000);
        debugln("loop 2");
    }
    mainMenu.dirty = true;
    return proceed;
}

result stepPID()
{
    updatePrefs(Kp,"Kp");
    updatePrefs(Kd,"Kd");
    updatePrefs(Ki,"Ki");
    fuerzaSetpoint = fuerzaInicial * 1000;
    if (fuerzaInicial == 0) {fuerzaSetpoint=100;}


    fuerzaPID.SetMode(MANUAL);
    fuerzaPID.SetOutputLimits(-maxSpeedX, maxSpeedX);
    fuerzaPID.SetSampleTime(50);

    stepperY->setSpeedInHz(maxSpeedX*fuerzaFinal/20);
    stepperY->setAcceleration(accelerationX * 10);
    stepperY->setCurrentPosition(0);

    drawGraph(1);

    stepperY->runForward();
    while (digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }
        fuerzaInput = scale.get_units(numSamples); // newton
        if (fuerzaInput>50){ stepperY->forceStop();break;}
        gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
    }

    int x = 3;
    int m = 0;

    stepperY->setCurrentPosition(mm2step(0.5));
    stepperY->forceStop();
    stepperY->setAcceleration(accelerationX*10);
    stepperY->applySpeedAcceleration();
    delay(100);
    float ceroCelda = scale.get_units(10);

    while (m<100)
    {
        if (emergencyStopCheck())
        {
            break;
        }
        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            fuerzaInput = scale.get_units(numSamples); // newton
            double altura = fuerzaInput - ceroCelda;
            gfx.drawFloat(altura/1000.0, 3, 50, 1, 1);
            float a = map(altura, -1000, 1000, 130, 18);
            monitorf("%f\t%f\t%f\n", current_time/1000.0, 0.0, altura);
            gfx.drawPixel(x, a, TFT_GREEN);
            x++;
            m++;
            if (x==240) {
                x=3;
                drawGraph(6);
            }
            last_input_time = current_time;
        }

    }

    drawGraph(2);
    drawGraph(4);
    drawGraph(6);

    nav.poll(); // para que aparezca el ON en el menu
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

    while (digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }

        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            fuerzaInput = scale.get_units(numSamples);
            double altura = fuerzaInput - ceroCelda;
            int joyY_value = analogRead(joyY);
            float desired_speedY = - map(joyY_value, 0, 4095, -100, 100);
            stepperY->setSpeedInHz(abs(desired_speedY));
            if (not(joyY_value > bufferMinY && bufferMaxY > joyY_value))
            {
                if (desired_speedY < 0)
                {
                    stepperY->runBackward();
                }
                else if (desired_speedY > 0)
                {
                    stepperY->runForward();
                }
            }
            if (joyY_value > bufferMinY && bufferMaxY > joyY_value)
            {
                stepperY->stopMove();
            }
            monitorf("%f\t%f\t%f\n", current_time/1000.0, desired_speedY, altura);
            double error = fuerzaSetpoint - fuerzaInput;
            gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
            gfx.drawFloat(fuerzaOutput, 0, 162, 1, 1);

            float i = map(fuerzaInput, 0, 1.2*fuerzaFinal, 130, 18);
            float s = map(fuerzaSetpoint,0, 1.2*fuerzaFinal, 130, 18);
            float o = map(fuerzaOutput, -100, 100, 130, 18);
            gfx.drawPixel(x, s, TFT_GREEN);
            gfx.drawPixel(x, i, TFT_YELLOW);
            gfx.drawPixel(x, o, TFT_RED);
            x++;
            if (x==240) {
                x=3;
                drawGraph(6);
            }
            last_input_time = current_time;
        }

    }
    stepperY->forceStop();
    stepperY->setSpeedInHz(maxSpeedX);
    stepperY->moveTo(0);
    nav.refresh();
    mainMenu.dirty = true;
    return proceed;
}
result calibrarPID()
{
    updatePrefs(Kp,"Kp");
    updatePrefs(Kd,"Kd");
    updatePrefs(Ki,"Ki");
    fuerzaSetpoint = fuerzaInicial * 1000;
    if (fuerzaInicial == 0) {fuerzaSetpoint=100;}


    fuerzaPID.SetMode(AUTOMATIC);
    fuerzaPID.SetOutputLimits(-maxSpeedX, maxSpeedX);
    fuerzaPID.SetSampleTime(50);

    stepperY->setSpeedInHz(maxSpeedX*fuerzaFinal/20);
    stepperY->setAcceleration(accelerationX * 10);
    stepperY->setCurrentPosition(0);

    drawGraph(1);

    stepperY->runForward();
    while (digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }
        fuerzaInput = scale.get_units(numSamples); // newton
        if (fuerzaInput>50){ stepperY->forceStop();break;}
        gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
    }

    int x = 3;

    // 0.2
    stepperY->setAcceleration(accelerationX*10);
    stepperY->applySpeedAcceleration();

    drawGraph(2);
    drawGraph(4);
    drawGraph(6);

    while (digitalRead(joySW))
    {
        if (emergencyStopCheck())
        {
            break;
        }

        unsigned long current_time = millis();
        if (current_time - last_input_time > 50)
        {
            fuerzaInput = scale.get_units(numSamples);
            fuerzaPID.Compute();
            if (fuerzaOutput>0.2){
                stepperY->setSpeedInHz(fuerzaOutput);
                stepperY->runForward();
            } else if (fuerzaOutput<-0.2){
                stepperY->setSpeedInHz(-fuerzaOutput);
                stepperY->runBackward();
            }
            else { stepperY->stopMove();}
            double error = fuerzaSetpoint - fuerzaInput;
            gfx.drawFloat(fuerzaInput/1000.0, 3, 50, 1, 1);
            gfx.drawFloat(fuerzaOutput, 0, 162, 1, 1);

            float i = map(fuerzaInput, 0, 1.2*fuerzaFinal, 130, 18);
            float s = map(fuerzaSetpoint,0, 1.2*fuerzaFinal, 130, 18);
            float o = map(fuerzaOutput, -100, 100, 130, 18);
            gfx.drawPixel(x, s, TFT_GREEN);
            gfx.drawPixel(x, i, TFT_YELLOW);
            gfx.drawPixel(x, o, TFT_RED);
            x++;
            if (x==240) {
                x=3;
                drawGraph(6);
            }
            last_input_time = current_time;
        }

    }
    stepperY->forceStop();
    stepperY->setSpeedInHz(maxSpeedX);
    stepperY->moveTo(0);
    nav.refresh();
    mainMenu.dirty = true;
    return proceed;
}

long leerCelda(){
    return  scale.get_units(numSamples);
}

result toggleCalibracionCelda()
{
    if (calibrarCelda==true){
        updatePrefs(CALIBRATION_FACTOR,"CALIBRATIONFACTOR");
        updatePrefs(numSamples,"numSamples");
    }
    calibrarCelda = ~calibrarCelda;
    return proceed;
}    

void initMotors()
{
    // stepperX.setMaxSpeed(maxSpeedX);
    // stepperX.setAcceleration(accelerationX);

    engine.init();
    stepperX = engine.stepperConnectToPin(stepX);
    stepperX->setDirectionPin(dirX);
    stepperX->setSpeedInHz(maxSpeedX);
    stepperX->setAcceleration(accelerationX);

    stepperY = engine.stepperConnectToPin(stepY);
    stepperY->setDirectionPin(dirY,false);
    stepperY->setSpeedInHz(maxSpeedX);
    stepperY->setAcceleration(accelerationX);

    driverX.begin();
    driverX.rms_current(600);
    driverX.en_pwm_mode(1);
    driverX.pwm_autoscale(1);
    driverX.microsteps(MICROSTEP);

    driverY.begin();
    driverY.rms_current(600);
    driverY.en_pwm_mode(1);
    driverY.pwm_autoscale(1);
    driverY.microsteps(MICROSTEP);

    stepperX->move(mm2step(1));
    stepperX->move(mm2step(-1));
    stepperY->move(mm2step(-1));
    stepperY->move(mm2step(1));

    stepperX->setCurrentPosition(0);
    stepperY->setCurrentPosition(0);
}

void drawGraph(int stage)
{
#if GRAPH == 1
    gfx.setTextPadding(80);

    switch (stage)
    {
        case 1:
            gfx.fillScreen(TFT_BLACK);
            gfx.setTextColor(TFT_WHITE, TFT_BLACK);
            gfx.drawString("A", 1, 2, 1);
            gfx.setTextColor(TFT_YELLOW, TFT_BLACK);
            gfx.drawString("I", 25, 2, 1);
            gfx.setTextColor(TFT_WHITE, TFT_BLACK);
            break;
        case 2:
            gfx.fillScreen(TFT_BLACK);
            gfx.setTextColor(TFT_BLUE, TFT_BLACK);
            gfx.drawString("E", 1, 2, 1);
            break;
        case 3:
            gfx.fillScreen(TFT_BLACK);
            gfx.setTextColor(TFT_GREEN, TFT_BLACK);
            gfx.drawString("M", 1, 2, 1);
            break;
        case 4:
            gfx.setTextColor(TFT_YELLOW, TFT_BLACK);
            gfx.drawString("I", 25, 2, 1);
            // gfx.drawString("Error", 120, 70, 1);
            gfx.setTextColor(TFT_RED, TFT_BLACK);
            gfx.drawString("O", 140, 2, 1);
            gfx.setTextColor(TFT_WHITE, TFT_BLACK);
            // gfx.drawString("V", 120, 110, 1);
            break;
        case 5:
            gfx.fillRect(0,20,240,135,TFT_BLACK);
            f = map(fuerzaFinal, -1*fuerzaFinal, 3*fuerzaFinal, 130, 18);
            O = map(0, -1*fuerzaFinal, 3*fuerzaFinal, 130, 18);
            dy = f-O;
            gfx.drawLine(0, f, 240, f, TFT_BLUE);
            gfx.drawLine(0, O, 240, O, TFT_WHITE);
            gfx.drawLine(0, O+2*dy, 240, O+2*dy, TFT_DARKGREY);
            gfx.drawLine(0, O+3*dy, 240, O+3*dy, TFT_DARKGREY);
            gfx.drawLine(0, O-dy, 240, O-dy, TFT_DARKGREY);
            break;
        case 6: //para el medir, grafico completo
            gfx.fillRect(0,20,240,135,TFT_BLACK);
            f = map(fuerzaFinal, 0, 1.2*fuerzaFinal, 130, 18);
            O = map(0, 0, 1.2*fuerzaFinal, 130, 18);
            dy = f-O;
            gfx.drawLine(0, f, 240, f, TFT_BLUE);
            gfx.drawLine(0, O, 240, O, TFT_WHITE);
            // gfx.drawLine(0, O+2*dy, 240, O+2*dy, TFT_DARKGREY);
            // gfx.drawLine(0, O+3*dy, 240, O+3*dy, TFT_DARKGREY);
            // gfx.drawLine(0, O-dy, 240, O-dy, TFT_DARKGREY);
            gfx.drawLine(0, map(0, -100, 100, 130, 18),
                         0, map(0, -100, 100, 130, 18), TFT_BROWN);
            break;

    }
#endif
}

void initPreferences()
{
    prefs.begin("scratch");
    bool init = prefs.isKey("init");
    // init = false;
    if (init == false)
    {
        prefs.putDouble("Kp", Kp);
        prefs.putDouble("Ki", Ki);
        prefs.putDouble("Kd", Kd);
        prefs.putFloat("CALI", CALIBRATION_FACTOR);
        prefs.putFloat("separacion", separacion);
        prefs.putInt("fuerzaInicial", fuerzaInicial);
        prefs.putInt("fuerzaFinal", fuerzaFinal);
        prefs.putInt("velocidad", velocidad);
        prefs.putInt("largo", largo);
        prefs.putInt("cantVeces", cantVeces);
        prefs.putInt("cantMm", cantMm);
        prefs.putInt("pasosPorMm", pasosPorMm);
        prefs.putInt("maxSpeedX", maxSpeedX);
        prefs.putInt("accelerationX", accelerationX);
        prefs.putInt("MICROSTEP", MICROSTEP);
        prefs.putInt("buffer", buffer);
        prefs.putInt("TOL", TOL);
        prefs.putBool("init", true);
        prefs.putFloat("CALI", CALIBRATION_FACTOR);
        prefs.putDouble("eKp", eKp);
        prefs.putDouble("eKi", eKi);
        prefs.putDouble("eKd", eKd);
        prefs.putDouble("Kpmin", Kpmin);
        prefs.putDouble("Kpmax", Kpmax);
        prefs.putDouble("Kpinc", Kpinc);
        prefs.putDouble("Kimin", Kimin);
        prefs.putDouble("Kimax", Kimax);
        prefs.putDouble("Kiinc", Kiinc);
    }
    else
    {
        prefs.putInt("TOL", TOL);
        prefs.putFloat("separacion", separacion);
        Kp = prefs.getDouble("Kp");
        Ki = prefs.getDouble("Ki");
        Kd = prefs.getDouble("Kd");
        CALIBRATION_FACTOR = prefs.getFloat("CALI");
        separacion = prefs.getFloat("separacion");
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
        TOL = prefs.getInt("TOL");
        eKp = prefs.getDouble("eKp");
        eKi = prefs.getDouble("eKi");
        eKd = prefs.getDouble("eKd");
        Kpmin = prefs.getDouble("Kpmin");
        Kpmax = prefs.getDouble("Kpmax");
        Kpinc = prefs.getDouble("Kpinc");
        Kimin = prefs.getDouble("Kimin");
        Kimax = prefs.getDouble("Kimax");
        Kiinc = prefs.getDouble("Kiinc");
    }
}

void updatePrefs(int value, const char* key)
{
    debugf("guardando %s:%d", key, value);
    prefs.putInt(key,value);
}

void updatePrefs(double value, const char* key)
{
    debugf("guardando %s:%d", key, value);
    prefs.putDouble(key,value);
}

void updatePrefs(float value, const char* key)
{
    debugf("guardando %s:%d", key, value);
    prefs.putFloat(key,value);
}

void testPrefs()
{
    debugf("\nKd guardada=%f\tKd=%f\n", prefs.getFloat("Kd"), Kd);
    // updatePrefs({"Kd", "Ki"}, {Kd, Ki}, 2);
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
    // int speedX = stepperX.speed();
    // stepperX.setAcceleration(accelerationX * 10);
    // stepperX.stop();
    //
    int speedX = stepperX->getSpeedInMilliHz();
    stepperX->setAcceleration(accelerationX * 10);
    stepperX->applySpeedAcceleration();
    stepperY->stopMove();
    int speedY = stepperY->getSpeedInMilliHz();
    stepperY->setAcceleration(accelerationX * 10);
    stepperY->applySpeedAcceleration();
    stepperY->stopMove();

    // stepperX.setAcceleration(accelerationX);
    stepperY->setAcceleration(accelerationX);
    if (speedX < 0)
    {
        stepperX->move(mm2step(5));
    }
    else if (speedX > 0)
    {
        stepperX->move(mm2step(-5));
    }

    if (speedY < 0)
    {
        stepperY->move(mm2step(5));
    }
    else if (speedY > 0)
    {
        stepperY->move(mm2step(-5));
    }
    activateEmergencyStop = false;
}

result resetearConfig()
{
    prefs.remove("init");
    return proceed;
}

