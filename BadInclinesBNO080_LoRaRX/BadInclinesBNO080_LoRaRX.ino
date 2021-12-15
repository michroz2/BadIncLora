/* Log:

  16.11.2021  Запустить на BSFrance (не используя пока Лору) - DONE
  21.11.2021 Скопировать в LoRaRX - DONE
  15.12.2921 очищено от лишнего кода (кроме кнопки) и загрузить в приёмник

  ===============
  Цель проекта: инерционно-независимый уровень.
  Соединяем IMU BNO080 и светодиодную ленту WS2812B в одном приборе!
  Разнести датчик и индикатор и соединить их по радио через LoRa

  ===============
  Соединения:
    Лента WS2812B 13 светодиодов (задаётся в коде - в принципе любое, из-за среднего "нуля" напрашивается нечётное число):
      +5v -> 5v BSFrance LoRa32u4
      GND -> GND BSFrance LoRa32u4
      DIN (сигнал) -> D6 BSFrance LoRa32u4 (настраивается в коде)
      NOTE!!! На обратной стороне BSFrance надо перерезать перемычку "DIO10->6 default closed"
    Control button "NO"(нормально-открытые) контакты:
  (В принципе, кнопка не нужна на приёмнике; используется для проверки работы при отладке)
      1 -> GND
      2 -> D5 BSFrance LoRa32u4 (задаётся в коде)
  Функции кнопки:
    Короткое нажатие: Яркость (следующее в соответствующем списке значение глобальной яркости индикатора)
    Двойное нажатие: Чувствительность (следующая настройка чувствительности)
    Длинное нажатие (более короткое): Обнуление датчика;
    Длинное нажатие (очень длинное): Инверсия сторон индикатора
*/
//Поиск мест, где можно что-то менять - по ключевому слову: ПОДСТРОЙКА

#include <PGMWrap.h>  //для удобного использования таблицы чувствительностей, загруженной во Флэш-память (PROGMEM =)
#include <FastLED.h> //для ленты WS2812B
#include <OneButton.h> //для кнопки
#include <SPI.h>              // include LoRa required 2 libraries
#include <LoRa.h>

//ПОДСТРОЙКА: Дебагирование (вывод текстов на Serial Monitor): раскомментить следующую 1 строчку:
//#define DEBUG_ENABLE  //ЗАКОММЕНТИРОВАТЬ, когда всё отработано, перед окончательной загрузкой на BSFrance LoRa32u4
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.print(x)
#define DEBUGln(x) Serial.println(x)
#else
#define DEBUG(x)
#define DEBUGln(x)
#endif

#ifdef DEBUG_ENABLE
#define TODO(x) Serial.print(F("TODO:\t")); Serial.println(F("x"))
#else
#define TODO(x)
#endif

//#define PROC_ENABLE  //вывод на монитор входы-выходы функций. ЗАКОММЕНТИРОВАТЬ, когда отработаны последовательности переходов процедур
#ifdef PROC_ENABLE
#define PROC(x) Serial.print(x)
#define PROCln(x) Serial.println(x)
#else
#define PROC(x)
#define PROCln(x)
#endif

/*
  //= MILLIS TIMER MACRO =
  // performs the {subsequent} code once and then again after each x ms
  #define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flg = millis() - tmr >= (x);\
  if (flg) tmr = millis();\
  if (flg)
*/

//Настройки для ленты:
#define PIN_LEDS 6    //К какому дигитальному пину подключено управление ЛЕДами
#define NUM_LEDS 13   //количество ЛЕДов в ленте
#define NUM_MODES 23  //Количество вариантов свечения ленты

// PIN для кнопки:
#define PIN_CONTROL_BUTTON 5 //Button PIN (а второй NO вывод кнопки ->GND)

//Дефолтовые: 10, 9, 2 для SPI библиотеки (которая используется для связи с LoRa)
const int csPin = 8;          // LoRa radio chip select AdaFruit/BSFrance
const int resetPin = 4;       // LoRa radio reset AdaFruit/BSFrance
const int irqPin = 7;         // change for your board; must be a hardware interrupt pin AdaFruit/BSFrance

//ПОДСТРОЙКА синхронно для TX и RX в диапазоне 0-254, а также чтобы не совпадало с другими лорами
byte workAddress = 111;  // address of connection

//ПОДСТРОЙКА частоты - должна совпадать у TX и RX и желательно отличаться от других систем, хотя бы на 1E5 (100КГц)
unsigned long workFrequency = 4345E5; //working Frequency: 434.5MHz

//Команды для связи:
#define CMD_BRIGHTNESS       100 //параметр = реальная яркость 
#define CMD_SENSITIVITY      101 //параметр = curSensitivity 
#define CMD_SWITCHSIDES      102 //параметр = revers 
#define CMD_LONGPRESS        103 //параметр = 1
#define CMD_CALIBRATION      104 //параметр = 1
#define CMD_MODE             111 //параметр = curMode от 0 до NUM_MODES
#define CMD_GREETING         112 //параметр = whatever?
#define CMD_EEPROM_WRITE     113 //параметр = whatever?

//Это «рабочий» массив для ленты: заполняем его нужными цветами и вызываем show
CRGB leds [NUM_LEDS];

//ПОДСТРОЙКА: Здесь можно исправлять вручную паттерны ЛЕДов для отколонений влево.
// (паттерны для уклонов вправо задавать не надо - они получатся симметрично автоматически)
CRGB modes[NUM_MODES][NUM_LEDS] =
  //LED_FULL_BRIGHTNESS
{
  {0xff0000, 0x640800, 0x321400,        0,        0,        0,        0,        0, 0, 0, 0, 0, 0},  //L11
  {0xff0000, 0x8c0000, 0x500a00, 0x3c1400, 0x200800,        0,        0,        0, 0, 0, 0, 0, 0},  //L10
  {0xff0000, 0x780000, 0x641000, 0x321E00, 0x180E00, 0x0F1400, 0x000400,        0, 0, 0, 0, 0, 0},  //L9
  {0x0a0000, 0x780000, 0x641000, 0x321E00, 0x321E00, 0x1E2D00, 0x000C00,        0, 0, 0, 0, 0, 0},  //L8
  {0x040000, 0x080000, 0x641000, 0x501E00, 0x463200, 0x283C00, 0x001E00,        0, 0, 0, 0, 0, 0},  //L7
  {0,        0x040000, 0x080000, 0x501E00, 0x5A4600, 0x3C5000, 0x003200,        0, 0, 0, 0, 0, 0},  //L6
  {0,        0,        0x040000, 0x501E00, 0x786400, 0x506400, 0x005000,        0, 0, 0, 0, 0, 0},  //L5
  {0,        0,        0,        0x321E00, 0x786400, 0x647800, 0x007800,        0, 0, 0, 0, 0, 0},  //L4
  {0,        0,        0,        0,        0x786400, 0x647800, 0x00A000,        0, 0, 0, 0, 0, 0},  //L3
  {0,        0,        0,        0,        0x2D1E00, 0x647800, 0x00C800,        0, 0, 0, 0, 0, 0},  //L2
  {0,        0,        0,        0,        0,        0x467800, 0x00FF00,        0, 0, 0, 0, 0, 0},  //L1
  {0,        0,        0,        0,        0,        0x002020, 0x00FF00, 0x002020, 0, 0, 0, 0, 0},  //!LEVEL MODE!
};

//ПОДСТРОЙКА: здесь можно задавать яркости:
#define NUM_FADES 4   //Количество вариантов яркости ленты
uint8_t fades [NUM_FADES] = {255, 64, 32, 8}; //значение яркости при каждом варианте (255 = макс)

//Следующий паттерн («двойная радуга») загорится при длинном нажатии кнопки.
//При отпускании сразу после «радуги» вся лента загорится синим и пойдёт процесс обнуления.
CRGB modeLongPressStart [NUM_LEDS] =
{
  CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Blue, CRGB::Indigo, CRGB::Violet,
  CRGB::Indigo, CRGB::Blue, CRGB::Green, CRGB::Yellow, CRGB::Orange, CRGB::Red
};

//(Возможна ПОДСТРОЙКА :)
//Это паттерн  приветствия
//Последовательно пробежит, заполняясь слева направо от края до края,
//вот такое количество таких цветов:
#define NUM_TEST_COLORS 3
CRGB testColors [NUM_TEST_COLORS] =
{  CRGB::Red, CRGB::Green, CRGB::Blue };

//ПОДСТРОЙКА времени (в целых секундах), после которого LEDы уйдут в StandBy
unsigned long StandByTime = 30;
unsigned long nextStandByMillis;

//Define Control Button.
OneButton buttonControl(PIN_CONTROL_BUTTON, true);

byte curFade; //Текущее значение яркости
byte curMode;   //Новое значение режима (зависит от угла наклона)
byte prevMode = -1; //Предыдущее значение режима
float Roll;   //Крен, который и надо показать светодиодами
float deltaZero; //Поправка на неровность установки
boolean revers;   //Переключалка для сторон уровня
#define MAX_INDEX 8
byte rcvCmd[MAX_INDEX];      //Полученная команда
byte rcvData[MAX_INDEX];     //Полученный параметр комады
byte rcvIndex, curIndex;    //Позиция получаемой и выполняемой команды
boolean StandBy;            //Индикатор, что мы в режиме стэндбай

#define NUM_SENSITIVITIES 4                         //ПОДСТРОЙКА количества вариантов чувствительности
float_p modeRange[NUM_SENSITIVITIES][NUM_MODES - 1] PROGMEM = //ПОДСТРОЙКА границ диапазонов крена - в градусах - «0» не включать!
  //!Нужно задать левую половину граничных значений углов для каждой чувствительности, а правые написать симметрично!
{
  { 10, 5.0, 2.2, 1.6, 1.1,  0.8,  0.6,  0.4,  0.3,  0.2,  0.1,   -0.1, -0.2, -0.3, -0.4, -0.6, -0.8, -1.1, -1.6, -2.2, -5.0, -10,}, //во вторую половину записать симметрично
  { 10, 5.0, 2.8, 2.1, 1.5,  1.1,  0.8,  0.5,  0.4,  0.3,  0.2,   -0.2, -0.3, -0.4, -0.5, -0.8, -1.1, -1.5, -2.1, -2.8, -5.0, -10,}, //во вторую половину записать симметрично
  { 10, 5.0, 3.4, 2.6, 1.9,  1.4,  1.0,  0.6,  0.5,  0.4,  0.3,   -0.3, -0.4, -0.5, -0.6, -1.0, -1.4, -1.9, -2.6, -3.4, -5.0, -10,}, //во вторую половину записать симметрично
  { 12, 6.0, 4.0, 3.1, 2.3,  1.7,  1.2,  0.7,  0.6,  0.5,  0.4,   -0.4, -0.5, -0.6, -0.7, -1.2, -1.7, -2.3, -3.1, -4.0, -6.0, -12,}, //во вторую половину записать симметрично
};
byte curSensitivity = 0;    //Текущее значение чувствительности, для начала 0

#define VERY_LONG_PRESS_MS  3000  //это ПОДСТРОЙКА: длительность очень длинного нажатия (мс), при котором
//вместо калибровки произойдёт инверсия сторон индикатора

boolean startCalibrationMode = false;
uint32_t verylongPressTimer = 0;

/*
  //Моргание встроенным светодиодом - переключение каждые N циклов.
  #define BLINK_EVERY_N 50
  int blinkAlive;
  bool  blinkLED;
*/

//****************************************************************************************************
void setup() { //===========  SETUP =============

  // initialize serial port to output the debug information (if necessary)
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
#endif

  DEBUGln(F("Bad Inclines BNO080 LoRa RX ==== Setup ===="));

  initButtons();  //времено оставлено для отладки
  initMODS();
  initLEDs();
  initLoRa();
  playGreeting();
  StandByTime = StandByTime * 1000; //тупо превращаем секунды в миллисекунды
  nextStandByMillis = millis() + StandByTime;       //и устанавливаем следующее время стендбая
}

void loop() {  //===========  LOOP =============
  PROCln(F("tick()"));
  buttonControl.tick();   // keep watching the push button
  PROCln(F("/tick()"));
  processCommand();
  processLEDS();          //Обновляем (если надо) паттерн свечения светодиодов
}              //=========== /LOOP =============

void initButtons() {
  PROCln(F("initButtons()"));
  // link the CONTROL button functions.
  buttonControl.attachClick(clickControl);
  delay(20);
  buttonControl.attachDoubleClick(doubleclickControl);
  delay(20);
  buttonControl.attachLongPressStart(longPressStartControl);
  delay(20);
  buttonControl.attachLongPressStop(longPressStopControl);
  delay(20);
  buttonControl.attachDuringLongPress(longPressControl);
  delay(20);
}////initButtons()

void clickControl() {
  PROCln(F("Control Button clicked"));
  DEBUGln(F("Fade Function"));
  curFade = (curFade + 1) % NUM_FADES;
  // set master brightness control
  showBrightness(fades[curFade]);
  DEBUG(F("Current Brightness Number: "));
  DEBUG(curFade);
  DEBUG(F(",\tCurrent Brightness: "));
  DEBUGln(fades[curFade]);
}////clickControl()

void showBrightness(byte brightness) {
  FastLED.setBrightness(brightness);
  prevMode = -1;    //Делаем искусственно так, чтобы индикатор обновился, даже если не изменился угол
}////showBrightness();

void doubleclickControl() {
  PROCln(F("Control Button double-clicked"));
  curSensitivity = (curSensitivity + 1) % NUM_SENSITIVITIES;
  showSensitivity();
}////doubleclickControl()

void showSensitivity() {
  PROCln(F("showSensitivity()"));
  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = 0;
  }
  leds[curSensitivity] = modeLongPressStart[curSensitivity];
  leds[NUM_LEDS - curSensitivity - 1] = modeLongPressStart[curSensitivity];
  FastLED.show();
  delay(1000);
  prevMode = -1;
  DEBUG(F("Sensitivity: "));
  DEBUGln(curSensitivity);

}////showSensitivity()


void switchSides() {
  DEBUG(F("Switching the sides: "));
  revers = !revers;
  DEBUGln(revers);
  showSwitchSides();
  copyMode();
  prevMode = -1;
}////switchSides()

void showSwitchSides() {
  for (byte i = 0; i < NUM_LEDS / 2; i++) {
    leds[i] = CRGB::Blue;
    leds[NUM_LEDS - i - 1] = CRGB::Red;
    FastLED.show();
  }
  delay(1000);
  for (byte i = 0; i < NUM_LEDS / 2; i++) {
    leds[i] = CRGB::Red;
    leds[NUM_LEDS - i - 1] = CRGB::Blue;
  }
  FastLED.show();
  delay(1000);

}////showSwitchSides()


void longPressStartControl() {
  PROCln(F("Control Button long-press started"));
  startCalibrationMode = true;  //if release button soon, then start calibration
  verylongPressTimer = millis() + VERY_LONG_PRESS_MS;  //start timer - для определения длины нажатия
  showLongPressStart();
}////longPressStartControl()

void showLongPressStart() {
  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = modeLongPressStart[i];
  }
  FastLED.show();
  delay(500);
}//showLongPressStart()

void longPressControl() {
  PROCln(F("Long Press Control Button ..."));
  if (millis() > verylongPressTimer) {
    DEBUGln(F("VERY Long Press detected..."));
    verylongPressTimer = millis() + VERY_LONG_PRESS_MS;  //start new timer - для следующего reverse
    startCalibrationMode = false;  //too long pressed for calibration
    switchSides();
  }
}////longPressControl()


void longPressStopControl() {
  PROCln(F("Control Button long-press stopped"));
  if (startCalibrationMode) {     //only if NOT VERY long pressed
    DEBUGln(F("Starting Calibration!"));
    startCalibrationMode = false;  //just in case, seems not necessary, but...
    showCalibration();
    prevMode = -1;
  }
}////longPressStopControl()

void showCalibration() {
  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Blue;  //set Blue lights to indicate calibration
  }
  FastLED.show();
  delay(500);
}////showCalibration()



void initMODS() { //Симетрично инициализируем значения массивов ледов для отклонения вправо
  PROCln(F("initMODS()"));

  curMode = NUM_MODES / 2;  // это будет центр, то есть «LEVEL» и с него мы начнём работу
  prevMode = -1;

  for (byte i = 0; i < NUM_MODES / 2; i++) {
    for (byte j = 0; j < NUM_LEDS; j++) {
      modes[NUM_MODES - i - 1][NUM_LEDS - j - 1] = modes[i][j];
      delay(10);
    }
  }
  //Контроль правильности присвоения всем модам и всем яркостям:
#ifdef DEBUG_ENABLE
  Serial.println(F("Modes:")); //заголовок
  for (byte i = 0; i < NUM_MODES; i++) {
    for (byte j = 0; j < NUM_LEDS; j++) {
      for (byte k = 0; k < 2; k++) {
        Serial.print(String(modes[i][j][k]) + ("/")); //значения R и G единичных ледов
      }
      Serial.print(String(modes[i][j][2])); //значениe Blue единичного леда
      Serial.print(F("\t")); //разделитель отдельных значений одной моды
    }
    Serial.println();   //перевод строки - разделитель мод
  }
  Serial.println(F("------------------"));   //конец вывода мод
#endif

}////initMODS()

/*
  void initModeRanges()  { //Симметрично заполняем правую половину массива границ диапазонов чувствительностей по заданной левой
  PROCln(F("initModeRanges()>>>>>"));
  for (byte s = 0; s < NUM_SENSITIVITIES; s++) {
    DEBUG(F("Sensitivity: "));
    DEBUGln(s);
    for (byte i = 0; i < NUM_MODES - 1 ; i++) {
      modeRange[s][NUM_MODES - 2 - i] = 0 - modeRange[s][i];
      DEBUG(modeRange[s][i]);
      DEBUG(F(",\t"));
    }
    DEBUGln();
  }
  }////initModeRanges()
*/

void copyMode() {
  byte k;
  for (byte i = 0; i < NUM_LEDS; i++) {
    k = revers ? (NUM_LEDS - 1 - i) : i;
    leds[i] = modes[curMode][k] ;
  }

}////copyMode()

void processLEDS()  {
  PROCln(F("processLEDS()"));
  if (curMode != prevMode) //
  {
    copyMode();
    FastLED.show();
    prevMode = curMode;
  }
  PROCln(F("/processLEDS()"));
}  ////processLEDS()

void  initLEDs()  { //Инициализация индикаторной ленты
  PROCln(F("initLEDs()"));

  pinMode(PIN_LEDS, OUTPUT);

  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  delay(100);
  FastLED.setBrightness(fades[curFade]);
  delay(100);
}////initLEDs()

void playGreeting() {
  PROCln(F("«««««playGreeting()»»»»»"));
  for (byte j = 0; j < 3; j++) {
    for (byte i = 0; i < NUM_LEDS; i++) {
      leds[i] = testColors[j];
      delay(10);
      FastLED.show();
      delay(40);   //ПОДСТРОЙКА
    }
    prevMode = -1;    //Делаем искусственно так, чтобы индикатор обновился
    delay(100);
  }
}////playGreeting();


void showEEPROMwrite() { //Индикация того, что в ЕЕПРОМ прошла запись
  PROCln(F("***showEEPROMwrite()***"));
  byte centerLED = NUM_LEDS / 2;
  for (byte i = 0; i < NUM_LEDS; i++) { //Все ЛЕДы делаем тёмными...
    leds[i] = CRGB::Black;
  }
  for (byte j = 0; j < 3; j++) {
    leds[centerLED] = CRGB::Red;  //...а центральный - то красный,
    FastLED.show();
    delay(200);
    leds[centerLED] = CRGB::Black;  //...то тёмный
    FastLED.show();
    delay(200);
  }
  prevMode = -1;    //Делаем так, чтобы индикатор обновился, даже если не изменился угол
}////showEEPROMwrite()

void showSwitchOff() { //Индикация перехода и переход LEDов в StandBy
  PROCln(F("***showSwitchOff()***"));
  byte centerLED = NUM_LEDS / 2;
  for (byte i = 0; i < NUM_LEDS; i++) { //Все ЛЕДы делаем тёмными...
    leds[i] = CRGB::Black;
  }
  for (byte j = 0; j < 3; j++) {
    leds[centerLED] = CRGB::Blue;  //...а центральный - то красный,
    FastLED.show();
    delay(200);
    leds[centerLED] = CRGB::Black;  //...то тёмный
    FastLED.show();
    delay(200);
  }
  delay(300);
}////showSwitchOff()

void showERROR() { //Индикация того, что в ЕЕПРОМ прошла запись
  PROCln(F("***showshowERROR()***"));
  byte centerLED = NUM_LEDS / 2;
  for (byte i = 0; i < NUM_LEDS; i++) { //Все ЛЕДы делаем тёмными...
    leds[i] = CRGB::Black;
  }
  for (byte j = 0; j < 5; j++) {
    leds[centerLED] = CRGB::Red;  //...а центральный - то красный,
    FastLED.show();
    delay(400);
    leds[centerLED] = CRGB::Black;  //...то тёмный
    FastLED.show();
    delay(400);
  }
}////showshowERROR()

void initLoRa()  {
  PROCln(F("initLoRa()"));
  // override the library default CS, reset, and IRQ pins
  LoRa.setPins(csPin, resetPin, irqPin);  // set CS, reset, IRQ pin

  if (!LoRa.begin(workFrequency)) {             // initialize radio at workFrequency
    DEBUGln(F("LoRa init failed. Check your connections."));
    while (true) {
      showERROR();// Визуальное отображение ошибки
      delay(2000);
    }
  }

  /*
    //Tweak parameters for best communication
    DEBUGln(F("setLoRaParams()"));
    //Trying setting LoRa for Longest Range possible:
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);    //max
    LoRa.setSignalBandwidth(125E3);                 //..31.25E3, 41.7E3, 62.5E3, (125E3), and 250E3.
    LoRa.setSpreadingFactor(8);                    //default = 7
    LoRa.setPreambleLength(6);                    //min = 6, default = 8
    LoRa.enableCrc();                             //
    //  LoRa.setCodingRate4(5);
  */

  // Uncomment the next line to disable the default AGC and set LNA gain, values between 1 - 6 are supported
  // LoRa.setGain(6);

  // register the receive callback
  LoRa.onReceive(onReceive);

  // put the radio into receive mode
  LoRa.receive();

  DEBUGln(F("LoRa TX init success."));
  PROCln(F("/initLoRa()"));
}////initLoRa()


void onReceive(int packetSize) {
  PROCln(F("onReceive(int packetSize)"));

  if (packetSize != 3) { //нас интересуют только пакеты в 3 байта
    DEBUGln("Wrong Packet Size: " + String(packetSize));
    return;          // not our packet, return
  }

  byte rcvAddress = LoRa.read();          // received address
  if (rcvAddress != workAddress) {  //нас интересуют только пакеты с нашим адресом
    DEBUGln("Wrong Address: " + String(rcvAddress) + " - " +  String(workAddress));
    return;
  }
  rcvCmd[rcvIndex] = LoRa.read();              // received command
  rcvData[rcvIndex] = LoRa.read();             // received data
  rcvIndex = (rcvIndex + 1) % MAX_INDEX;  //Индекс ходит по кругу

  long  deltaFrequency = LoRa.packetFrequencyError();
  workFrequency = workFrequency - deltaFrequency / 2;
  LoRa.setFrequency(workFrequency);

}////onReceive(int packetSize)

void processCommand() {
  unsigned long curMillis = millis();
  if (!StandBy && (curIndex == rcvIndex))   {    //Не пришло команд, но мы ешё не в стэндбай
    //    DEBUGln(F("===No new commands==="));
    if (curMillis > nextStandByMillis) {  //Если при этом прошло много времени, переходим в стэндбай
      showSwitchOff();
      StandBy = true;
    }
    return;
  }
  while (curIndex != rcvIndex) {
    switch (rcvCmd[curIndex]) {
        /*
               //Команды для связи:
          #define CMD_BRIGHTNESS       100 //параметр = реальная яркость
          #define CMD_SENSITIVITY      101 //параметр = curSensitivity
          #define CMD_SWITCHSIDES      102 //параметр = revers
          #define CMD_LONGPRESS        103 //параметр = 1
          #define CMD_CALIBRATION      104 //параметр = 1
          #define CMD_MODE             111 //параметр = curMode от 0 до NUM_MODES
          #define CMD_GREETING         112 //параметр = whatever?
          #define CMD_EEPROM_WRITE     113 //параметр = whatever?
        */
        break;
      case CMD_MODE:
        DEBUGln(F("===CMD_MODE==="));
        curMode = rcvData[curIndex];
        break;
      case CMD_BRIGHTNESS:
        DEBUGln("===CMD_BRIGHTNESS===");
        if (fades[curFade] != rcvData[curIndex])
        showBrightness(rcvData[curIndex]);
        break;
      case CMD_SENSITIVITY:
        DEBUGln("===CMD_SENSITIVITY===");
        if (curSensitivity != rcvData[curIndex]) {
        curSensitivity = rcvData[curIndex];
        showSensitivity();
        }
        break;
      case CMD_SWITCHSIDES:
        DEBUGln(F("===CMD_SWITCHSIDES==="));
        if (revers != rcvData[curIndex])
          switchSides();
        break;
      case CMD_LONGPRESS:
        DEBUGln(F("===CMD_LONGPRESS==="));
        showLongPressStart();
        break;
      case CMD_CALIBRATION:
        DEBUGln(F("===CMD_CALIBRATION==="));
        showCalibration();
        break;
      case CMD_GREETING:
        DEBUGln(F("===CMD_GREETING==="));
        playGreeting();
        break;
      case CMD_EEPROM_WRITE:
        DEBUGln(F("===CMD_EEPROM_WRITE==="));
        showEEPROMwrite();
        break;
    }////switch
    curIndex = (curIndex + 1) % MAX_INDEX;  //Индекс ходит по кругу, догоняя rcvIndex
    StandBy = false;
    nextStandByMillis = curMillis + StandByTime; //обновляем следующее время стэндбая
  }////while
}////void processCommand()
