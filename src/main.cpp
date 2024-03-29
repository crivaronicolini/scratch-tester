#include <Arduino.h>
#include <Preferences.h>
Preferences prefs;
#include <TFT_eFEX.h>
#include <TFT_eSPI.h>

#include <ClickEncoder.h>
#include <menu.h>
#include <menuIO/TFT_eSPIOut.h>
#include <streamFlow.h>
// Using this library: https://github.com/soligen2010/encoder.git
#include <menuIO/chainStream.h>
#include <menuIO/clickEncoderIn.h>
#include <menuIO/keyIn.h>
#include <menuIO/serialIO.h>
// For debugging the TFT_eSPIOut.h library
#include <menuIO/serialIn.h>
#include <menuIO/serialOut.h>

#include <FastAccelStepper.h>
#include <TMCStepper.h>

#include <PID_v1.h>

#include <HX711.h>

HX711 scale;

using namespace Menu;

// define menu colors --------------------------------------------------------
#define Black (uint16_t) RGB565(0, 0, 0)
#define Red (uint16_t) RGB565(255, 0, 0)
#define Green (uint16_t) RGB565(0, 255, 0)
#define Blue (uint16_t) RGB565(0, 0, 255)
#define Gray (uint16_t) RGB565(128, 128, 128)
#define LighterRed (uint16_t) RGB565(255, 60, 92)    // colores OP-1
#define LighterGreen (uint16_t) RGB565(6, 236, 150)  //
#define LighterBlue (uint16_t) RGB565(111, 132, 225) //
#define LighterGray (uint16_t) RGB565(211, 211, 211)
#define DarkerRed (uint16_t) RGB565(150, 0, 0)
#define DarkerGreen (uint16_t) RGB565(0, 150, 0)
#define DarkerBlue (uint16_t) RGB565(0, 0, 150)
#define Cyan (uint16_t) RGB565(0, 255, 255)
#define Magenta (uint16_t) RGB565(255, 0, 255)
#define Yellow (uint16_t) RGB565(255, 255, 0)
#define White (uint16_t) RGB565(255, 255, 255)
#define DarkerOrange (uint16_t) RGB565(255, 140, 0)

// TFT color table
const colorDef<uint16_t> colors[6] MEMMODE = {
    //{{disabled normal,disabled selected},{enabled normal,enabled selected,
    // enabled editing}}
    {{Black, Black}, {Black, Red, Red}},     // bgColor
    {{White, White}, {White, White, White}}, // fgColor
    {{Red, Red}, {Yellow, Yellow, Yellow}},  // valColor
    {{White, White}, {White, White, White}}, // unitColor
    {{White, Gray}, {Black, Red, White}},    // cursorColor
    {{White, Yellow}, {Black, Red, Red}},    // titleColor
};

// Define the width and height of the TFT and how much of it to take up
#define GFX_WIDTH 240
#define GFX_HEIGHT 135
#define fontW 12
#define fontH 15

// Declare pins for rotary encoder
#define encB 36 // dt
#define encA 39 // clk
#define encBtn 32
// steps per detent
#define encSteps 4

// Declare pins for joystick
#define joyY 35
#define joyX 34
#define joySW 32

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
// factor crudo = 102.01401412240335 divido por g=9.8066 para tener N
float CALIBRATION_FACTOR = 10.40258745;
long reading = -1;
int numSamples = 1;
bool calibrarCelda = false;

#define MAX_DEPTH 5
#define CENTERX 120
#define CENTERY 62

// params menu
int chooseField = 1;
int menuDelayTime = 100;

// params medicion
int fuerzaInicial = 5;    // N
int fuerzaInicialCte = 5; // N
int fuerzaInicialDin = 5; // N
int fuerzaFinal = 55;     // N
int velocidad = 5;        // mm x minuto
float largo = 5;          // mm
int direccionRayado = 1;  // negativo=hacia la derecha
int loadingRate = 10;     // N/mm
float separacion = 0.5;

bool medicionCompletada = false;

// params PID
double fuerzaSetpoint, fuerzaInput, fuerzaOutput;
double Kp = 0.02, Ki = 0, Kd = 0;
double eKp = 0.185, eKi = 0.019, eKd = 0.007;
int TOL = 120;

// params calibracion
int cantVeces = 2;
int cantMm = 5;
int pasosPorMm = 1600;

double Kpmin = 0, Kpmax = 0, Kpinc = 0, Kimin = 0, Kimax = 0, Kiinc = 0;

// params motores:
// factor = MICROSTEP * pasosmotor / pasotornillo = 256 * 200 /2 = 25600
int MICROSTEP = 256;
int factor = 25600;
float factor_um = 25.600;
float factor_t = factor / 60;

int mm2step(float mm) { return mm * factor; }
float step2um(int step) { return step / factor_um; }
float step2um(float step) { return step / factor_um; }
float step2mm(int step) { return step / factor; }
float step2mm(float step) { return step / factor; }
int mmxm2stepxs(float mmxm) { return mmxm * factor_t; }
float stepxs2mmxm(float stepxs) { return stepxs / factor_t; }

int distPuntaFoco = -1217866;
int distPanorama = 18251;

int maxSpeedX = mmxm2stepxs(100);
int accelerationX = 4 * maxSpeedX;
int maxSpeedJog = 4 * maxSpeedX;
int maxSpeedJogY = maxSpeedJog * 0.8;
int accelerationJog = 4 * maxSpeedJog;

// params tiempo
int INPUT_READ_INTERVAL = 100;
unsigned long last_input_time = 0;
unsigned long last_redraw = 0;
unsigned long lastButtonPress = 0;
unsigned long lastStopTime = 0;

// params botones
int joySW_status = 1;
// bool emergencyStopIsActive = false;
int buffer = 150;
int test = 44;

PID fuerzaPID(&fuerzaInput, &fuerzaOutput, &fuerzaSetpoint, Kp, Ki, Kd, DIRECT);

TMC2130Stepper driverX =
    TMC2130Stepper(CS_PINX, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
TMC2130Stepper driverY =
    TMC2130Stepper(CS_PINY, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperX = NULL;
FastAccelStepper *stepperY = NULL;

ClickEncoder clickEncoder = ClickEncoder(encA, encB, encBtn, encSteps);
ClickEncoderStream encStream(clickEncoder, 1.5);

// TFT gfx is what the ArduinoMenu TFT_eSPIOut.h is expecting
TFT_eSPI gfx = TFT_eSPI();
TFT_eFEX fex = TFT_eFEX(&gfx);
void updatePrefs(float value, const char *key);
void updatePrefs(int value, const char *key);
void updatePrefs(double value, const char *key);
void testPrefs();
result updateLargo();
result resetearConfig();
result despejar();
bool medir();
result mapear();
void moverMuestra();
result moverMuestraMenu();
result medirMenu();
result medirMenuCte();
result calibrarMotores();
result precisionMotores();
result calibrarPID();
result gridSearch();
result calibrarMicroscopio();
result siguienteFoto();
result anteriorFoto();
long leerCelda();
result toggleCalibracionCelda();
void initPreferences();
void initMotors();
// bool emergencyStopCheck();
// void emergencyStop();
// void IRAM_ATTR emergencyStopActivate();
void IRAM_ATTR onTimer(); // Start the timer to read the clickEncoder every 1 ms

#define DEBUG 1
#define MONITOR 1

#if MONITOR == 1
#define monitor(x) Serial.print(x)
#define monitorln(x) Serial.println(x)
#define monitorf(...) Serial.printf(__VA_ARGS__)
#define monitorw(...) Serial.write(__VA_ARGS__)
#else
#define monitor(x)                                                             \
  do {                                                                         \
  } while (0)
#define monitorln(x)                                                           \
  do {                                                                         \
  } while (0)
#define monitorf(...)                                                          \
  do {                                                                         \
  } while (0)
#endif

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(...) Serial.printf(__VA_ARGS__)
#else
#define debug(x)                                                               \
  do {                                                                         \
  } while (0)
#define debugln(x)                                                             \
  do {                                                                         \
  } while (0)
#define debugf(...)                                                            \
  do {                                                                         \
  } while (0)
#endif

//////////////////////////////////////////////////////////
// Start ArduinoMenu
//////////////////////////////////////////////////////////

result medirYMicroscopio() {
  bool canceled = medir();
  if (canceled) {
    return proceed;
  } else {
    // stepperX->setSpeedInHz(maxSpeedJog);
    // stepperX->move(-abs(distPuntaFoco));
    return proceed;
  };
}

result irAlMicro() {
  stepperX->stopMove();
  stepperX->setSpeedInHz(maxSpeedJog);
  stepperX->move(-abs(distPuntaFoco));
  // nav.doNav(navCmd(escCmd));
  return proceed;
}

result volverAOrigen() {
  stepperX->stopMove();
  stepperX->setSpeedInHz(maxSpeedJog);
  stepperX->moveTo(0);
  // nav.doNav(navCmd(escCmd));
  return proceed;
}

result updateEEPROM() {
  // writeEEPROM();
  return quit;
}

int ejeACalibrar = 1;
int toggleDummy = 0;
int constante = 0;
int toggleDummyDespejar = 0;
int toggleDummyCalibrarMicro = 0;

TOGGLE(toggleDummyCalibrarMicro, subMenuCalibrarMicroscopio, "Microscopio",
       doNothing, noEvent, noStyle,                   //
       VALUE("", 1, calibrarMicroscopio, enterEvent), //
       VALUE("", 0, doNothing, noEvent));

MENU(subMenuCalibrarCelda, "Celda de Carga", toggleCalibracionCelda,
     (eventMask)(enterEvent | exitEvent), wrapStyle,
     // FIELD(reading, "F:", "mN", -1, 100000, 0, 0, doNothing, noEvent,
     // noStyle),
     FIELD(CALIBRATION_FACTOR, "F calibracion:", "", 1, 200, 10, 1, doNothing,
           noEvent, noStyle),
     FIELD(numSamples, "N muestras:", "", 1, 100, 1, 0, doNothing, noEvent,
           noStyle),
     EXIT("<- Volver"));

TOGGLE(ejeACalibrar, subMenuToggleEjeACalibrar, "Motor a Calibrar:", doNothing,
       noEvent, noStyle,                  //
       VALUE("X", 1, doNothing, noEvent), //
       VALUE("Y", 2, doNothing, noEvent));

MENU(subMenuCalibrarMotores, "Motores", doNothing, noEvent, wrapStyle,
     OP("Calibrar", calibrarMotores, enterEvent),
     OP("Ver presicion", precisionMotores, enterEvent),
     SUBMENU(subMenuToggleEjeACalibrar),
     FIELD(cantVeces, "Cantidad de veces:", "", 0, 200, 10, 0, doNothing,
           noEvent, noStyle),
     FIELD(cantMm, "Cantidad de mm:", "", 0, 100, 10, 1, doNothing, noEvent,
           noStyle),
     FIELD(pasosPorMm, "Pasos por mm:", "", 1500, 1700, 10, 1, doNothing,
           noEvent, noStyle),
     EXIT("<- Volver"));

TOGGLE(toggleDummy, subMenuToggleGridSearch, "Grid Search", doNothing, noEvent,
       noStyle,                              //
       VALUE("", 1, gridSearch, enterEvent), //
       VALUE("", 0, doNothing, noEvent));

TOGGLE(toggleDummy, subMenuToggleCalibrarPID, "Calibrar PID", doNothing,
       noEvent, noStyle,                      //
       VALUE("", 1, calibrarPID, enterEvent), //
       VALUE("", 0, doNothing, noEvent));

MENU(subMenuMidiendoyFotos, "Microscopio", doNothing, enterEvent,
     wrapStyle,                                     //
     OP("Siguiente", siguienteFoto, enterEvent),    //
     OP("Anterior", anteriorFoto, enterEvent),      //
     OP("Ir al micro", irAlMicro, enterEvent),      //
     OP("Ir al origen", volverAOrigen, enterEvent), //
     EXIT("<- Volver"));

MENU(subMenuMidiendoyFotosCte, "Medir F cte", medirYMicroscopio, enterEvent,
     wrapStyle,                                         //
     OP("Siguiente", siguienteFoto, enterEvent),        //
     OP("Anterior", anteriorFoto, enterEvent),          //
     OP("Volver al origen", volverAOrigen, enterEvent), //
     EXIT("<- Volver"));

TOGGLE(toggleDummy, subMenuToggleMedir, "Medir", doNothing, noEvent, noStyle,
       VALUE(" ON", 1, medirMenu, enterEvent),
       VALUE("", 0, doNothing, noEvent));

TOGGLE(toggleDummy, subMenuToggleMedirCte, "Medir F cte", doNothing, noEvent,
       noStyle, VALUE(" ON", 1, medirMenuCte, enterEvent),
       VALUE("", 0, doNothing, noEvent));

TOGGLE(toggleDummy, subMenuToggleMapear, "Mapear", doNothing, noEvent, noStyle,
       VALUE("", 1, mapear, enterEvent), VALUE("", 0, doNothing, noEvent));

MENU(subMenuGridSearch, "Grid Search", doNothing, noEvent, wrapStyle,
     SUBMENU(subMenuToggleGridSearch),
     altFIELD(decPlaces<3>::menuField, Kpmin, "Kp min", "", 0, 20, 0.01, 0.001,
              doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField, Kpmax, "Kp max", "", 0, 20, 0.01, 0.001,
              doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField, Kpinc, "Kp inc", "", 0, 20, 0.01, 0.001,
              doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField, Kimin, "Ki min", "", 0, 20, 0.01, 0.001,
              doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField, Kimax, "Ki max", "", 0, 20, 0.01, 0.001,
              doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField, Kiinc, "Ki inc", "", 0, 20, 0.01, 0.001,
              doNothing, noEvent, noStyle),
     EXIT("<- Volver"));

MENU(subMenuMedir, "Medir", doNothing, noEvent, wrapStyle,
     // SUBMENU(subMenuMidiendoyFotos),
     SUBMENU(subMenuToggleMedir),
     FIELD(fuerzaInicialDin, "Fuerza inicial:", "N", 0, 200, 5, 1, updateLargo,
           enterEvent, noStyle),
     FIELD(fuerzaFinal, "Fuerza final:", "N", 0, 200, 5, 1, updateLargo,
           enterEvent, noStyle),
     altFIELD(decPlaces<1>::menuField, largo, "Largo:", "mm", 0, 20, 0, 0,
              doNothing, noEvent, noStyle),
     FIELD(velocidad, "Velocidad:", "mm/min", 0, 200, 10, 1, doNothing, noEvent,
           noStyle),
     EXIT("<- Volver"));

MENU(subMenuMedirCte, "Medir F constante", doNothing, noEvent, wrapStyle,
     SUBMENU(subMenuToggleMedirCte),
     FIELD(fuerzaInicialCte, "Fuerza:", "N", 0, 200, 5, 1, doNothing,
           enterEvent, noStyle),
     altFIELD(decPlaces<1>::menuField, largo, "Largo:", "mm", 0, 20, 1, 0.5,
              doNothing, noEvent, noStyle),
     FIELD(velocidad, "Velocidad:", "mm/s", 0, 200, 10, 1, doNothing, noEvent,
           noStyle),
     EXIT("<- Volver"));

MENU(subMenuCalibrarPID, "PID", doNothing, noEvent, wrapStyle,
     // SUBMENU(subMenuToggleCalibrarPID),
     // SUBMENU(subMenuGridSearch),
     SUBMENU(subMenuToggleMapear),
     // SUBMENU(subMenuToggleMedir),
     SUBMENU(subMenuMedir), //
     altFIELD(decPlaces<3>::menuField, Kp, "Proporcional:", "", -5, 5, 0.01,
              0.001, doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField, Ki, "Integrador:", "", -5, 5, 0.01,
              0.001, doNothing, noEvent, noStyle),
     altFIELD(decPlaces<3>::menuField, Kd, "Derivador:", "", -5, 5, 0.01, 0.001,
              doNothing, noEvent, noStyle),
     EXIT("<- Volver"));

MENU(subMenuResetearConfig, "Resetear de fabrica", doNothing, noEvent,
     wrapStyle,                                  //
     OP("Resetear", resetearConfig, enterEvent), //
     EXIT("Cancelar"));

MENU(subMenuCalibrar, "Menu de calibracion", doNothing, noEvent, wrapStyle,
     SUBMENU(subMenuCalibrarPID),         //
     SUBMENU(subMenuCalibrarCelda),       //
     SUBMENU(subMenuCalibrarMotores),     //
     SUBMENU(subMenuCalibrarMicroscopio), //
     SUBMENU(subMenuResetearConfig),      //
     FIELD(TOL, "Tolerancia", "", 0, 1000, 10, 1, doNothing, noEvent, noStyle),
     FIELD(separacion, "Separacion", "", 0, 5, 1, 0.1, doNothing, noEvent,
           noStyle),
     FIELD(loadingRate, "Tasa de carga", "N/mm", 0, 100, 5, 1, doNothing,
           noEvent, noStyle),
     EXIT("<- Volver"));

TOGGLE(toggleDummy, subMenuToggleDefinirOrigen, "Mover muestra", doNothing,
       noEvent, noStyle, //
       VALUE(" ON", 1, moverMuestraMenu, enterEvent),
       VALUE("", 0, doNothing, noEvent));

TOGGLE(toggleDummyDespejar, subMenuToggleDespejar, "Despejar muestra",
       doNothing, noEvent, noStyle,           //
       VALUE(" ON", 1, despejar, enterEvent), //
       VALUE("", 0, doNothing, noEvent));

MENU(mainMenu, "SCRATCHTESTER3000", doNothing, noEvent, wrapStyle,
     SUBMENU(subMenuToggleDefinirOrigen), //
     SUBMENU(subMenuMedir),               //
     SUBMENU(subMenuMidiendoyFotos),      //
     SUBMENU(subMenuMedirCte),            //
     SUBMENU(subMenuToggleDespejar),      //
     SUBMENU(subMenuCalibrar));

// Main menu panel
const panel panels[] MEMMODE = {{0, 0, GFX_WIDTH / fontW, GFX_HEIGHT / fontH}};
// navNodes to store navigation status
navNode *nodes[sizeof(panels) / sizeof(panel)];
// a list of panels and nodes
panelsList pList(panels, nodes, sizeof(panels) / sizeof(panel));
// idx_t tops[MAX_DEPTH]={0,0}; // store cursor positions for each level
idx_t eSpiTops[MAX_DEPTH] = {0};
TFT_eSPIOut eSpiOut(gfx, colors, eSpiTops, pList, fontW, fontH + 1);
idx_t serialTops[MAX_DEPTH] = {0};
serialOut outSerial(Serial, serialTops);
#if DEBUG == 1
// list of output devices
menuOut *constMEM outputs[] MEMMODE = {&outSerial, &eSpiOut};
// menuOut *constMEM outputs[] MEMMODE = {&eSpiOut};
#else
menuOut *constMEM outputs[] MEMMODE = {&eSpiOut};
#endif
outputsList out(outputs,
                sizeof(outputs) / sizeof(menuOut *)); // outputs list
serialIn serial(Serial);
MENU_INPUTS(in, &encStream, &serial); // &encButton,
NAVROOT(nav, mainMenu, MAX_DEPTH, in, out);

// ESP32 timer thanks to:
// http://www.iotsharing.com/2017/06/how-to-use-interrupt-timer-in-arduino-esp32.html
// and:
// https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//////////////////////////////////////////////////////////
// End Arduino Menu
//////////////////////////////////////////////////////////

void alertError(String msg) {
  gfx.fillScreen(Black);
  gfx.setTextColor(DarkerRed);
  gfx.setCursor(0, 0);
  gfx.println(msg);

  gfx.setCursor(200, 110);
  gfx.setTextColor(White, LighterRed);
  gfx.drawRightString("OK", GFX_WIDTH - 10, GFX_HEIGHT - 20, 1);
  while (digitalRead((joySW))) {
    delay(100);
  }
  nav.refresh();
}

void alertMsg(String msg) {
  gfx.fillScreen(Black);
  gfx.setTextColor(LighterBlue);
  gfx.drawCentreString(msg, CENTERX, 15, 1);
}

void setup() {
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
  gfx.setTextFont(1);
  gfx.setTextSize(2);
  debugln("done");

  nav.showTitle = true; // Show titles in the menus and submenus
  //  nav.timeOut = 60;  // Timeout after 60 seconds of inactivity
  //  nav.idleOn(); // Start with the main screen and not the menu
  nav.refresh();

  pinMode(encBtn, INPUT_PULLUP);
  pinMode(stopSW, INPUT_PULLUP);
  // attachInterrupt(stopSW, emergencyStopActivate, RISING);

  scale.begin(load, SCK);
  if (scale.wait_ready_retry(3, 500)) {
    scale.set_scale(CALIBRATION_FACTOR);
    debugln("Tare... remove any weights from the scale.");
    scale.tare(20);
    debugln("Tare done...");
  } else {
    debugln("HX711 not found.");
    alertMsg("Celda de carga desconectada\nConectar y reiniciar");
  }
  fuerzaPID.SetMode(0);
  updateLargo();
  // emergencyStopIsActive = false;
  Serial.println(factor);
  Serial.println(factor_um);
  Serial.println(factor_t);
}

void loop() {
  // Slow down the menu redraw rate
  constexpr int menuFPS = 60;
  static unsigned long lastMenuFrame = -menuFPS;
  unsigned long now = millis();

  if (now - lastMenuFrame >= menuFPS) {
    if (calibrarCelda) {
      reading = leerCelda();
      debugln(reading);
    }
    lastMenuFrame = millis();
    nav.poll();
  }
}

void moverMuestra() {
  //  samplea `cantidad` de veces y toma el promedio para ver el cero del
  //  joystick
  int suma = 0;
  int cantidad = 50;
  for (int i = 0; i < cantidad; i++) {
    suma += analogRead(joyX);
  }
  float centroX = (suma / cantidad);
  float bufferMaxX = centroX + buffer;
  float bufferMinX = centroX - buffer;
  suma = 0;
  for (int i = 0; i < cantidad; i++) {
    suma += analogRead(joyY);
  }
  float centroY = (suma / cantidad);
  float bufferMaxY = centroY + buffer;
  float bufferMinY = centroY - buffer;
  debugf("centroX: %f\n", centroX);
  debugf("buffer maxX: %f\n", bufferMaxX);
  debugf("buffer minX: %f\n", bufferMinX);
  debugf("centroY: %f\n", centroY);
  debugf("buffer maxY: %f\n", bufferMaxY);
  debugf("buffer minY: %f\n", bufferMinY);
  stepperX->setAcceleration(accelerationJog);
  stepperY->setAcceleration(accelerationJog);

  int fin = 0;
  while (digitalRead(joySW)) {
    unsigned long current_time = millis();
    if (current_time - last_input_time > INPUT_READ_INTERVAL) {
      int joyX_value = analogRead(joyX);
      int joyY_value = analogRead(joyY);
      // los menos son para que den bien las direcciones
      int desired_speedX = -map(joyX_value, 0, 4095, -maxSpeedJog, maxSpeedJog);
      int desired_speedY =
          -map(joyY_value, 0, 4095, -maxSpeedJogY, maxSpeedJogY);

      stepperX->setSpeedInHz(abs(desired_speedX));
      stepperY->setSpeedInHz(abs(desired_speedY));
      debugf("X %.0f; Y %.0f\n", joyX_value - centroX, joyY_value - centroY);
      bool joyXinDeadZone = joyX_value > bufferMinX && bufferMaxX > joyX_value;
      if (not(joyXinDeadZone)) {
        if (desired_speedX < 0) {
          stepperX->runBackward();
        } else if (desired_speedX > 0) {
          stepperX->runForward();
        }
      }
      // retiro la punta si la chocamos un poco
      if (scale.get_units(1) > 30) {
        stepperY->forceStop();
        while (scale.get_units(1) > 30) {
          stepperY->move(mm2step(-0.5));
        }
        continue;
      }
      bool joyYinDeadZone = joyY_value > bufferMinY && bufferMaxY > joyY_value;
      if (not(joyYinDeadZone)) {
        if (desired_speedY < 0) {
          stepperY->runBackward();
        } else if (desired_speedY > 0) {
          stepperY->runForward();
        }
      }
      if (joyXinDeadZone) {
        stepperX->stopMove();
      }
      if (joyYinDeadZone) {
        stepperY->stopMove();
      }
      last_input_time = current_time;
    }
  }
  stepperY->stopMove();
  stepperX->stopMove();
}

result medirMenuCte() {
  constante = 1;
  return medirMenu();
}

result medirMenu() {
  // nav.doOutput();
  debugln("nav doOutput");
  // alertMsg("DEFINIR ORIGEN");
  bool canceled = medir();
  // encStream.flush();
  // toggleDummy = 0;
  if (not canceled) {
    debugln("toggledummy");
    // delay(950);
    toggleDummy = 0;
  }
  // nav.doOutput();
  nav.refresh();
  // nav.refresh();
  debugln("nav refresh");
  return proceed;
}

result moverMuestraMenu() {
  nav.doOutput();
  // alertMsg("DEFINIR ORIGEN");
  moverMuestra();
  // toggleDummy = 0;
  nav.refresh();
  return proceed;
}

result despejar() {
  // mueve los ejes una distancia prudente para sacar la muestra
  nav.poll();
  stepperY->setSpeedInHz(maxSpeedJog);
  stepperY->setAcceleration(accelerationJog);
  stepperY->move(mm2step(-10), true);

  toggleDummyDespejar = 0;
  return proceed;
}

result mapear() {
  updatePrefs(velocidad, "velocidad");
  updatePrefs(largo, "largo");

  // ACERCAMIENTO
  stepperX->setCurrentPosition(0);
  stepperY->setSpeedInHz(maxSpeedX * fuerzaInicial / 20);
  stepperY->setAcceleration(accelerationX * 10);

  float largoSteps = mm2step(largo);

  double altura = 0;

  stepperY->runForward();
  while (digitalRead(joySW)) {
    // if (emergencyStopCheck()) {
    //   break;
    // }
    fuerzaInput = scale.get_units(numSamples); // newton
    if (fuerzaInput > 50) {
      stepperY->forceStop();
      break;
    }
    altura = fuerzaInput;
    unsigned long current_time = millis();
    monitorf("%d\t%d\t%d\t%f\t%f\t%f\n", current_time,
             stepperX->getCurrentPosition(), stepperY->getCurrentPosition(),
             step2um(stepperX->getCurrentPosition()),
             step2um(stepperY->getCurrentPosition()), altura);
  }

  int m = 0;

  // ESTABILIZACION
  stepperY->setCurrentPosition(mm2step(0.2));
  stepperY->forceStop();
  delay(100);
  float ceroCelda = scale.get_units(10);

  while (m < 50 && digitalRead(joySW)) {
    // if (emergencyStopCheck()) {
    //   break;
    // }
    fuerzaInput = scale.get_units(numSamples); // newton
    altura = fuerzaInput - ceroCelda;
    unsigned long current_time = millis();
    monitorf("%d\t%d\t%d\t%f\t%f\t%f\n", current_time,
             stepperX->getCurrentPosition(), stepperY->getCurrentPosition(),
             step2um(stepperX->getCurrentPosition()),
             step2um(stepperY->getCurrentPosition()), altura);
    m++;
  }

  // MEDICION
  stepperX->setCurrentPosition(0);
  stepperX->setSpeedInHz(mmxm2stepxs(velocidad));
  stepperX->move(largoSteps);

  while (digitalRead(joySW) && stepperX->isRunning()) {
    // if (emergencyStopCheck()) {
    //   break;
    // }
    fuerzaInput = scale.get_units(numSamples); // newton
    altura = fuerzaInput - ceroCelda;
    unsigned long current_time = millis();
    monitorf("%d\t%d\t%d\t%f\t%f\t%f\n", current_time,
             stepperX->getCurrentPosition(), stepperY->getCurrentPosition(),
             step2um(stepperX->getCurrentPosition()),
             step2um(stepperY->getCurrentPosition()), altura);
  }

  if (not(stepperX->isRunning())) {
    toggleDummy = 0;
  }
  stepperX->forceStop();
  stepperY->forceStop();

  stepperX->setSpeedInHz(maxSpeedX);
  stepperY->setSpeedInHz(maxSpeedX);

  stepperY->moveTo(0);
  delay(20);
  stepperX->moveTo(0);
  return proceed;
}

result gridSearch() {
  updatePrefs(Kpmin, "Kpmin");
  updatePrefs(Kpmax, "Kpmax");
  updatePrefs(Kpinc, "Kpinc");
  updatePrefs(Kimin, "Kimin");
  updatePrefs(Kimax, "Kimax");
  updatePrefs(Kiinc, "Kiinc");
  int i = 0;
  for (Kp = Kpmin; Kp < Kpmax; Kp += Kpinc) {
    for (Ki = Kimin; Ki < Kimax; Ki += Kiinc) {
      i++;
    }
  }
  int n = i;
  i = 0;
  for (Kp = Kpmin; Kp < Kpmax; Kp += Kpinc) {
    for (Ki = Kimin; Ki < Kimax; Ki += Kiinc) {
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

void progressBar(char step, float ratio) {
  switch (step) {
  case 'a':
    gfx.drawRoundRect(20, 65, 200, 22, 1, Gray);
    break;
  case 'e':
    gfx.drawRoundRect(20, 65, 200, 22, 1, LighterBlue);
    break;
  case 's':
    gfx.drawRoundRect(20, 65, 200, 22, 1, White);
    break;
  case 'm':
    gfx.fillRect(21, 66, ratio * 199, 19, White);
    break;
  }
}

bool medir() {
  // nav.doOutput();
  alertMsg("MIDIENDO");
  progressBar('a', .0);
  gfx.setTextColor(Gray);
  gfx.setTextSize(1);
  gfx.drawRightString("CANCELAR", GFX_WIDTH - 10, GFX_HEIGHT - 20, 1);
  gfx.setTextSize(2);

  updatePrefs(Kp, "Kp");
  updatePrefs(Kd, "Kd");
  updatePrefs(Ki, "Ki");
  updatePrefs(velocidad, "velocidad");
  updatePrefs(fuerzaInicialDin, "fInicialDin");
  updatePrefs(fuerzaInicialCte, "fInicialCte");
  updatePrefs(fuerzaFinal, "fuerzaFinal");
  updatePrefs(largo, "largo");
  updatePrefs(loadingRate, "loadingRate");
  updatePrefs(separacion, "separacion");
  updatePrefs(TOL, "TOL");

  // desacople las fuerzas iniciales de los menu con la de la medicion
  // ahora tengo que ver cual es la que aplica a esta medicion
  fuerzaInicial = constante ? fuerzaInicialCte : fuerzaInicialDin;

  fuerzaSetpoint = fuerzaInicial * 1000;

  // paso a una vel de acercamiento proporcional a la raiz de la fuerza
  // inicial. uso un parametro que lo deje igual a como era antes para 5N
  // de fI
  float speed = 2.2360 * maxSpeedX * sqrt(fuerzaInicial) / 80;
  if (fuerzaInicial == 0) {
    speed = 800; // si fuerza inicial es cero no le gusta al set speed
  }
  // valor en miliNewton
  int fuerzaFinalM = fuerzaFinal * 1000;
  int fuerzaInicialM = fuerzaInicial * 1000;

  int deltaF = fuerzaFinal - fuerzaInicial;
  int deltaFM = fuerzaFinalM - fuerzaInicialM;
  float largoSteps = direccionRayado * mm2step(largo);
  float ratio = 0.;
  float ratioOpt = 0.;
  double error = 0;

  int errAbs = 0;

  fuerzaPID.SetOutputLimits(-500, 1600);
  fuerzaPID.SetSampleTime(80);

  unsigned long current_time = millis();

  ////// ACERCAMIENTO //////

  fuerzaPID.SetMode(MANUAL);
  // la primera linea tiene los parámetros de la medicion
  debugln("fuerzaInicial, fuerzaFinal, largo, velocidad, Kp, Ki, Kd");
  monitorf("#%d,%d,%f,%d,%f,%f,%f\n", fuerzaInicial, fuerzaFinal, largo,
           velocidad, Kp, Ki, Kd);

  // luego siguen en el siguiente orden
  debugln("t, x, y, fIn, fSet, fOut, errAbs");

  stepperX->setCurrentPosition(0);
  stepperY->setSpeedInHz(speed);
  stepperY->runForward();

  while (digitalRead(joySW)) {
    fuerzaInput = scale.get_units(numSamples); // newton
    if (fuerzaInput > 30) {
      stepperY->forceStop();
      break;
    }
    monitorf("#%d,%.3f,%.3f,%.0f,%.0f,%.0f,%d\n", millis(),
             step2um(stepperX->getCurrentPosition()),
             step2um(stepperY->getCurrentPosition()), fuerzaInput,
             fuerzaSetpoint, fuerzaOutput, errAbs);
  }

  // set el cero para que la punta quede `separacion` arriba  de la muestra
  stepperY->forceStopAndNewPosition(mm2step(separacion));

  ////// ESTABILIZACION //////
  fuerzaOutput = 200;
  fuerzaPID.SetMode(AUTOMATIC);

  // anuncio las etapas: 0 = Estabilizacion, 1 = Acercamiento
  monitorln("#0,0,0,0,0,0,0");
  progressBar('e', .0);

  float TOLmod = (TOL * deltaF / 5);
  stepperY->setAcceleration(accelerationX);
  stepperY->setSpeedInHz(speed / 2);
  stepperY->runForward();

  while (digitalRead(joySW)) {
    fuerzaInput = scale.get_units(numSamples);

    error = fuerzaSetpoint - fuerzaInput;
    if (error < TOLmod || fuerzaInput > fuerzaSetpoint * 1.1) {
      debugf("estabilizo a %f\n", fuerzaInput);
      stepperY->forceStop();
      break;
    }

    current_time = millis();
    if (current_time - last_input_time > 20) {
      errAbs += abs(error) / 10;

      monitorf("#%d,%.3f,%.3f,%.0f,%.0f,%.0f,%d\n", current_time,
               step2um(stepperX->getCurrentPosition()),
               step2um(stepperY->getCurrentPosition()), fuerzaInput,
               fuerzaSetpoint, fuerzaOutput, errAbs);
      last_input_time = current_time;
    }
  }

  errAbs = 0;

  ////// MEDICION //////

  monitorln("#1,1,1,1,1,1,1");

  fuerzaPID.SetTunings(Kp, Ki, Kd);
  progressBar('s', .0);
  int stepperXPos = 0;
  stepperY->setSpeedInHz(speed);
  stepperX->setSpeedInHz(mmxm2stepxs(velocidad));
  stepperX->move(largoSteps);

  while ((digitalRead(joySW)) and (stepperX->isRunning())) {
    stepperXPos = stepperX->getCurrentPosition();
    ratio = stepperXPos / largoSteps;
    ratioOpt = constante ? 0.0 : ratio;
    fuerzaSetpoint = fuerzaInicialM + (deltaFM * ratioOpt);

    current_time = millis();
    if (current_time - last_input_time > 20) {
      fuerzaInput = scale.get_units(numSamples); // newton
      fuerzaPID.Compute();
      if (fuerzaOutput > 0) {
        stepperY->setSpeedInHz((uint32_t)fuerzaOutput);
        stepperY->runForward();
      } else if (fuerzaOutput < 0) {
        stepperY->setSpeedInHz((uint32_t)-fuerzaOutput);
        stepperY->runBackward();
      } else {
        stepperY->stopMove();
      }
      error = fuerzaSetpoint - fuerzaInput;
      // debugln(abs(error / fuerzaSetpoint));
      // if (abs(error / fuerzaSetpoint) > 0.45) {
      //   debugln("fuerza lejos del setpoint");
      //   //          -------------------- alertError("Fuerza lejos"
      //              "del setpoint");
      //   break;
      // };
      errAbs += abs(error) / 10.;
      monitorf("#%d,%.3f,%.3f,%.0f,%.0f,%.0f,%d\n", current_time,
               step2um(stepperXPos), step2um(stepperY->getCurrentPosition()),
               fuerzaInput, fuerzaSetpoint, fuerzaOutput, errAbs);
      last_input_time = current_time;
    }
    if (current_time - last_redraw > 40) {
      progressBar('m', ratio);
      last_redraw = current_time;
    }
  }
  fuerzaPID.SetMode(MANUAL);

  bool canceled = false;

  if (stepperX->isRunning()) {
    debugln("X is running, canceled");
    canceled = true;
  };

  stepperX->forceStop();
  stepperY->forceStop();
  // espero a que la cola de los motores se limpie, si no completa el movimiento
  // en X antes de hacer cualquier otra cosa
  delay(100);

  // sacar la punta y volver al origen:
  stepperY->setAcceleration(4 * accelerationX);
  stepperX->setSpeedInHz(2 * maxSpeedX);
  stepperY->setSpeedInHz(2 * maxSpeedX);

  // si la fuerza al final de la raya fue alta, retroceder la punta un poco
  // para para que no salte al retirarla
  if (fuerzaInput > 10000) {
    debugln("retreat");
    stepperX->move(-1 * direccionRayado * mm2step(0.1), true);
  }

  stepperY->moveTo(0, true);
  stepperX->moveTo(0);

  fuerzaOutput = 0.;
  medicionCompletada = true;
  constante = 0;
  return canceled;
}

result siguienteFoto() {
  // int currentX = stepperX->getCurrentPosition();
  // float largoSteps = mm2step(largo);
  // if (abs(currentX - distPuntaFoco) > largoSteps) {
  //   nav.doNav(navCmd(escCmd));
  //   return proceed;
  // }
  debugln("sig foto");
  stepperX->move(distPanorama);
  return proceed;
}

result anteriorFoto() {
  // int currentX = stepperX->getCurrentPosition();
  // float largoSteps = mm2step(largo);
  // if (abs(currentX - distPuntaFoco) > largoSteps) {
  //   return proceed;
  // }
  debugln("ant foto");
  stepperX->move(-distPanorama);
  return proceed;
}

result calibrarMicroscopio() {
  // despues de hacer una raya, da direcciones para mover la muestra hasta
  // que se vea en el microscopio. guarda la separacion entre la punta y el
  // foco del lente, y distancia entre foto y foto para hacer el panorama

  gfx.fillScreen(Black);

  // corroboramos que el origen está en el fijado por la medicion
  if (not medicionCompletada) {
    //          --------------------
    alertError("Calibrar despues de "
               "hacer una medicion");
    return proceed;
  }

  stepperY->move(mm2step(-1));

  gfx.fillScreen(Black);
  gfx.setTextColor(LighterBlue);
  gfx.setCursor(0, 0);
  //           --------------------
  gfx.println("Mover la muestra    "
              "hasta que el centro "
              "de la imagen sea el "
              "inicio de la raya");
  delay(500);

  gfx.setCursor(200, 110);
  gfx.setTextColor(Gray);
  gfx.println("OK");
  delay(500);

  moverMuestra();

  gfx.setCursor(200, 110);
  gfx.setTextColor(Black, LighterRed);
  gfx.println("OK");

  distPuntaFoco = stepperX->getCurrentPosition();
  delay(500);

  gfx.fillScreen(Black);
  gfx.setTextColor(LighterBlue);
  gfx.setCursor(0, 0);

  //           --------------------
  gfx.println("Mover la muestra    "
              "para que se solape  "
              "un 10% con la       "
              "imagen anterior");

  gfx.setCursor(200, 110);
  gfx.setTextColor(Gray);
  gfx.println("OK");

  // uso una velocidad mucho menor
  int maxSpeedJogBackup = maxSpeedJog;
  maxSpeedJog = maxSpeedJog / 4;
  moverMuestra();
  maxSpeedJog = maxSpeedJogBackup;

  gfx.setCursor(200, 110);
  gfx.setTextColor(Black, LighterRed);
  gfx.println("OK");
  delay(500);

  distPanorama = stepperX->getCurrentPosition() - distPuntaFoco;

  gfx.fillScreen(Black);
  gfx.setTextColor(LighterBlue);
  gfx.setCursor(0, 0);

  debugln("valores guardados:");
  debugf("dist punta foco %d\n", distPuntaFoco);
  debugf("dist panorama %d\n", distPanorama);
  updatePrefs(distPuntaFoco, "distPuntaFoco");
  updatePrefs(distPanorama, "distPanorama");

  //           --------------------
  gfx.println("Calibracion completa"
              "volviendo al origen");
  stepperX->setSpeedInHz(maxSpeedJog);
  stepperX->moveTo(0);
  delay(2000);
  nav.refresh();
  return proceed;
}

void medirNveces(int N) {
  int i = 0;
  while (digitalRead(joySW)) {
    fuerzaInput = scale.get_units(numSamples); // newton
    monitorf("#%d,%d,%d,%.3f\n", millis(), stepperX->getCurrentPosition(),
             stepperY->getCurrentPosition(), fuerzaInput);
    // monitorf("#%d,%.3f,%.3f,%.0f\n", millis(),
    //          step2um(stepperX->getCurrentPosition()),
    //          step2um(stepperY->getCurrentPosition()), fuerzaInput);
    if (i == N) {
      // stepperY->forceStop();
      break;
    }
    i++;
  }
};

result precisionMotores() {
  // copio de medir()
  // la idea es tocar la superficie mucho mas despacio, avanzar en x, tocar
  // denuevo y medir la fuerza que hacen distinta cantidad de microestps
  fuerzaInicial = constante ? fuerzaInicialCte : fuerzaInicialDin;
  fuerzaSetpoint = fuerzaInicial * 1000;
  float speed = 2.2360 * maxSpeedX * sqrt(5) / 80;
  if (fuerzaInicial == 0) {
    speed = 800; // si fuerza inicial es cero no le gusta al set speed
  }
  int fuerzaFinalM = fuerzaFinal * 1000;
  int fuerzaInicialM = fuerzaInicial * 1000;

  stepperX->setCurrentPosition(0);
  stepperY->setCurrentPosition(0);
  stepperY->setSpeedInHz(speed / 10);
  stepperY->runForward();

  while (digitalRead(joySW)) {
    fuerzaInput = scale.get_units(numSamples); // newton
    if (fuerzaInput > 30) {
      stepperY->forceStop();
      break;
    }
    monitorf("#%d,%d,%d,%.3f\n", millis(), stepperX->getCurrentPosition(),
             stepperY->getCurrentPosition(), fuerzaInput);
    // monitorf("#%d,%.3f,%.3f,%.0f\n", millis(),
    //          step2um(stepperX->getCurrentPosition()),
    //          step2um(stepperY->getCurrentPosition()), fuerzaInput);
  }
  // pongo el cero en la superficie y levanto la punta `separacion`
  stepperY->forceStopAndNewPosition(0);
  stepperY->move(-mm2step(separacion));
  medirNveces(30);

  // muevo x en 10um
  stepperX->moveTo(mm2step(0.01));

  // avanzo denuevo pero mas lento y mido mientras
  stepperY->setSpeedInHz(speed);
  stepperY->moveTo(0);
  medirNveces(30);

  // ahora deberia estar en la superficie aplicando una fuerza minima
  // muevo 1 step y mido
  for (int i = 0; i < 10; i++) {
    stepperY->move(1);
    monitorf("#%d,%d,0,0\n", millis(), i);
    medirNveces(50);
  };

  // muevo 5 step y mido
  for (int i = 0; i < 10; i++) {
    stepperY->move(5);
    monitorf("#%d,%d,0,0\n", millis(), i);
    medirNveces(50);
  };

  stepperY->moveTo(-mm2step(separacion));
  return proceed;
}

result calibrarMotores() {
  FastAccelStepper *stp;
  if (ejeACalibrar == 1) {
    stp = stepperX;
  } else {
    stp = stepperY;
  }
  stp->setSpeedInHz(mmxm2stepxs(velocidad));
  debugf("cant veces %d, pasosxmm %d, cantmm %d mm %d step\n", cantVeces,
         pasosPorMm, cantMm, mm2step(cantMm));
  debugf("velocidad %d mmxm, %f pxs\n", velocidad, mmxm2stepxs(velocidad));
  stp->setAcceleration(4 * maxSpeedX);

  for (int i = 0; i < cantVeces; i++) {
    stp->move(mm2step(cantMm));
    delay(1000);
    stp->move(-mm2step(cantMm));
    delay(1000);
    debugln("loop 2");
  }
  return proceed;
}

result calibrarPID() {
  updatePrefs(Kp, "Kp");
  updatePrefs(Kd, "Kd");
  updatePrefs(Ki, "Ki");
  fuerzaSetpoint = fuerzaInicial * 1000;
  if (fuerzaInicial == 0) {
    fuerzaSetpoint = 100;
  }

  fuerzaPID.SetMode(AUTOMATIC);
  // fuerzaPID.SetOutputLimits(-maxSpeedX, maxSpeedX);
  fuerzaPID.SetSampleTime(50);

  stepperY->setSpeedInHz(maxSpeedX * fuerzaFinal / 20);
  stepperY->setAcceleration(accelerationX * 10);
  stepperY->setCurrentPosition(0);

  stepperY->runForward();
  while (digitalRead(joySW)) {
    // if (emergencyStopCheck()) {
    //   break;
    // }
    fuerzaInput = scale.get_units(numSamples); // newton
    if (fuerzaInput > 50) {
      stepperY->forceStop();
      break;
    }
    gfx.drawFloat(fuerzaInput / 1000.0, 3, 50, 1, 1);
  }

  stepperY->setAcceleration(accelerationX * 10);
  stepperY->applySpeedAcceleration();

  while (digitalRead(joySW)) {
    // if (emergencyStopCheck()) {
    //   break;
    // }

    unsigned long current_time = millis();
    if (current_time - last_input_time > 50) {
      fuerzaInput = scale.get_units(numSamples);
      fuerzaPID.Compute();
      if (fuerzaOutput > 0.2) {
        stepperY->setSpeedInHz(fuerzaOutput);
        stepperY->runForward();
      } else if (fuerzaOutput < -0.2) {
        stepperY->setSpeedInHz(-fuerzaOutput);
        stepperY->runBackward();
      } else {
        stepperY->stopMove();
      }
      double error = fuerzaSetpoint - fuerzaInput;
      last_input_time = current_time;
    }
  }
  stepperY->forceStop();
  stepperY->setSpeedInHz(maxSpeedX);
  stepperY->moveTo(0);
  return proceed;
}

long leerCelda() { return scale.get_units(numSamples); }

result toggleCalibracionCelda() {
  if (calibrarCelda == true) {
    updatePrefs(CALIBRATION_FACTOR, "CALIBRATIONFACTOR");
    updatePrefs(numSamples, "numSamples");
  }
  calibrarCelda = ~calibrarCelda;
  return proceed;
}

void initMotors() {
  // Y es positivo hacia abajo
  // X es positivo hacia la izquierda
  engine.init();
  stepperX = engine.stepperConnectToPin(stepX);
  stepperX->setDirectionPin(dirX, false);
  stepperX->setSpeedInHz(maxSpeedX);
  stepperX->setAcceleration(accelerationX);

  stepperY = engine.stepperConnectToPin(stepY);
  stepperY->setDirectionPin(dirY);
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

  stepperY->move(mm2step(-0.1), true);
  stepperX->move(mm2step(-0.1), true);
  stepperX->move(mm2step(0.1));
  stepperY->move(mm2step(0.1));

  stepperX->setCurrentPosition(0);
  stepperY->setCurrentPosition(0);
}

result updateLargo() {
  // calcula el nuevo largo segun el loadingRate, y limpia el menu
  largo = (float)(fuerzaFinal - fuerzaInicialDin) / loadingRate;
  mainMenu[3].dirty = true;
  debugln("largo updated");
  return proceed;
}

void initPreferences() {
  prefs.begin("scratch");
  bool init = prefs.isKey("init");
  // init = false;
  if (init == false) {
    prefs.putFloat("CALI", CALIBRATION_FACTOR);
    prefs.putFloat("separacion", separacion);
    prefs.putInt("loadingRate", loadingRate);

    prefs.putInt("fInicialDin", fuerzaInicialDin);
    prefs.putInt("fuerzaFinal", fuerzaFinal);
    prefs.putInt("velocidad", velocidad);
    prefs.putInt("largo", largo);
    prefs.putInt("cantVeces", cantVeces);
    prefs.putInt("cantMm", cantMm);
    prefs.putInt("pasosPorMm", pasosPorMm);
    prefs.putInt("maxSpeedX", maxSpeedX);
    prefs.putInt("accelerationX", accelerationX);
    prefs.putInt("MICROSTEP", MICROSTEP);
    prefs.putInt("TOL", TOL);

    prefs.putInt("distPuntaFoco", distPuntaFoco);
    prefs.putInt("distPanorama", distPanorama);

    prefs.putBool("init", true);

    prefs.putDouble("Kp", Kp);
    prefs.putDouble("Ki", Ki);
    prefs.putDouble("Kd", Kd);
    prefs.putDouble("Kpmin", Kpmin);
    prefs.putDouble("Kpmax", Kpmax);
    prefs.putDouble("Kpinc", Kpinc);
    prefs.putDouble("Kimin", Kimin);
    prefs.putDouble("Kimax", Kimax);
    prefs.putDouble("Kiinc", Kiinc);

  } else {

    // prefs.putInt("fInicialDin", fuerzaInicialDin);
    prefs.putFloat("CALI", CALIBRATION_FACTOR);
    Kp = prefs.getDouble("Kp");
    Ki = prefs.getDouble("Ki");
    Kd = prefs.getDouble("Kd");

    CALIBRATION_FACTOR = prefs.getFloat("CALI");
    separacion = prefs.getFloat("separacion");
    loadingRate = prefs.getInt("loadingRate");

    fuerzaInicialDin = prefs.getInt("fInicialDin");
    fuerzaInicialCte = prefs.getInt("fInicialCte");
    fuerzaFinal = prefs.getInt("fuerzaFinal");
    velocidad = prefs.getInt("velocidad");
    largo = prefs.getInt("largo");
    cantVeces = prefs.getInt("cantVeces");
    cantMm = prefs.getInt("cantMm");
    pasosPorMm = prefs.getInt("pasosPorMm");
    maxSpeedX = prefs.getInt("maxSpeedX");
    accelerationX = prefs.getInt("accelerationX");
    MICROSTEP = prefs.getInt("MICROSTEP");
    TOL = prefs.getInt("TOL");

    distPuntaFoco = prefs.getInt("distPuntaFoco");
    distPanorama = prefs.getInt("distPanorama");

    Kpmin = prefs.getDouble("Kpmin");
    Kpmax = prefs.getDouble("Kpmax");
    Kpinc = prefs.getDouble("Kpinc");
    Kimin = prefs.getDouble("Kimin");
    Kimax = prefs.getDouble("Kimax");
    Kiinc = prefs.getDouble("Kiinc");

    debugln("PARAMETROS GUARDADOS");
    debugf("Kp: %f\n", Kp);
    debugf("Ki: %f\n", Ki);
    debugf("Kd: %f\n", Kd);

    debugf("CALIBRATION_FACTOR: %f\n", CALIBRATION_FACTOR);
    debugf("separacion: %f\n", separacion);
    debugf("loadingRate: %d\n", loadingRate);

    debugf("fuerzaInicialDin: %d\n", fuerzaInicialDin);
    debugf("fuerzaInicialCte: %d\n", fuerzaInicialCte);
    debugf("fuerzaFinal: %d\n", fuerzaFinal);
    debugf("velocidad: %d\n", velocidad);
    debugf("largo: %d\n", largo);
    debugf("cantVeces: %d\n", cantVeces);
    debugf("cantMm: %d\n", cantMm);
    debugf("pasosPorMm: %d\n", pasosPorMm);
    debugf("maxSpeedX: %d\n", maxSpeedX);
    debugf("accelerationX: %d\n", accelerationX);
    debugf("MICROSTEP: %d\n", MICROSTEP);
    debugf("TOL: %d\n", TOL);

    debugf("distPuntaFoco: %d\n", distPuntaFoco);
    debugf("distPanorama: %d\n", distPanorama);
  }
}

void updatePrefs(int value, const char *key) {
  debugf("guardando %s:%d\n", key, value);
  prefs.putInt(key, value);
}

void updatePrefs(double value, const char *key) {
  debugf("guardando %s:%f\n", key, value);
  prefs.putDouble(key, value);
}

void updatePrefs(float value, const char *key) {
  debugf("guardando %s:%f\n", key, value);
  prefs.putFloat(key, value);
}

void testPrefs() {
  debugf("\nKd guardada=%f\tKd=%f\n", prefs.getFloat("Kd"), Kd);
  // updatePrefs({"Kd", "Ki"}, {Kd, Ki}, 2);
}

// ESP32 timer
void IRAM_ATTR onTimer() { clickEncoder.service(); }

// void IRAM_ATTR emergencyStopActivate() {
//   unsigned long now = millis();
//
//   if (now - lastStopTime > 100) {
//     debugln("emergencyStop Activated");
//     emergencyStopIsActive = true;
//     lastStopTime = now;
//   }
// }
//
// bool emergencyStopCheck() {
//   if (emergencyStopIsActive) {
//     debugln("emergencyStopIsActive (if true):");
//     debugln(emergencyStopIsActive);
//     emergencyStop();
//     return true;
//   } else {
//     return false;
//   }
// }
//
// void emergencyStop() {
//   debugln("emergency Stop init");
//   int speedX = stepperX->getCurrentSpeedInMilliHz();
//   int speedY = stepperY->getCurrentSpeedInMilliHz();
//   stepperY->forceStop();
//   stepperX->forceStop();
//
//   debugf("Speed X= %d\n", speedX);
//   debugf("Speed Y= %d\n", speedY);
//   stepperY->setAcceleration(accelerationX);
//   if (speedX == 0) {
//     debugln("X speed cero");
//   } else if (speedX < 0) {
//     stepperX->move(mm2step(5));
//   } else if (speedX > 0) {
//     stepperX->move(mm2step(-5));
//   }
//
//   if (speedY == 0) {
//     debugln("Y speed cero");
//   } else if (speedY < 0) {
//     stepperY->move(mm2step(5));
//   } else if (speedY > 0) {
//     stepperY->move(mm2step(-5));
//   }
//   emergencyStopIsActive = false;
// }

result resetearConfig() {
  alertMsg("BORRANDOD PARAMETROS");
  prefs.remove("init");
  //        --------------------
  delay(3000);
  gfx.setCursor(0, 25);
  //           --------------------
  gfx.println("Reiniciar el equipo "
              "para ver los cambios");
  delay(3000);
  return proceed;
}
