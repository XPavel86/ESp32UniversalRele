  #include <Arduino.h>
  #include <esp_sleep.h>
  //==========================
  
  //=========================
//#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
////==========================
//// Задаем размер дисплея 128x32
//#define SCREEN_WIDTH 128
//#define SCREEN_HEIGHT 32
//
//// Объявляем объект дисплея с заданными размерами
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
//===============================
int buttonPin = 33;  // Пин для кнопки // для esp_sleep_enable_ext0_wakeup(buttonPinGPIO, 1);  должен быть только цифровой
//int ledPin = 17;     // или D2 на wifi led Пин для светодиода

//====================================

volatile bool buttonPressed = false;
int bounceTime = 10;          // задержка для подавления дребезга
int doubleTime = 500;         // время, в течение которого нажатия можно считать двойным
int longPressTime = 5000;     // время, после которого нажатие считается длительным
int i = 0;

boolean lastReading = false;  // флаг предыдущего состояния кнопки
boolean buttonSingle = false; // флаг состояния "краткое нажатие"
boolean buttonMulti = false;  // флаг состояния "двойное нажатие"

long onTime = 0;              // переменная обработки временного интервала
long lastSwitchTime = 0;      // переменная времени предыдущего переключения состояния
long pressDuration = 0;   

boolean longPressHandled = false;

void IRAM_ATTR handleButtonPress() {
    buttonPressed = true;
}

void processButtonPress(void (*onSinglePress)(), void (*onDoublePress)(), void (*onLongPress5s)()) {
  boolean reading = digitalRead(buttonPin);
// проверка первичного нажатия
  if (reading && !lastReading) {
    onTime = millis();
    longPressHandled = false; // Сбрасываем флаг длинного нажатия
  }

  // если кнопка все еще нажата
  if (reading && (millis() - onTime >= longPressTime) && !longPressHandled) {
    (*onLongPress5s)();
    longPressHandled = true; // Устанавливаем флаг, чтобы не вызывать обработчик многократно
  }

  // если кнопка отпущена
  if (!reading && lastReading) {
    pressDuration = millis() - onTime;
    if (pressDuration > bounceTime && pressDuration < longPressTime) {
      if ((millis() - lastSwitchTime) >= doubleTime) {
        lastSwitchTime = millis();
        buttonSingle = true;
        i = 1;
      } else {
        i++;
        lastSwitchTime = millis();
        buttonSingle = false;
        buttonMulti = true;
      }
    }
  }

  lastReading = reading;

  if (buttonSingle && (millis() - lastSwitchTime) > doubleTime) {
    (*onSinglePress)();
    buttonSingle = false;
  }
  if (buttonMulti && (millis() - lastSwitchTime) > doubleTime) {
    (*onDoublePress)();
    buttonMulti = false;
  }
}

//=====================================================

void enterSleepMode() {
    Serial.println("Entering sleep mode...");
 // Преобразуем тип int в gpio_num_t
    gpio_num_t buttonPinGPIO = static_cast<gpio_num_t>(33);
    
    // Устанавливаем пробуждение по внешнему событию на пине buttonPinGPIO
    esp_sleep_enable_ext0_wakeup(buttonPinGPIO, 1); 

    WiFi.setSleep(true);
    delay(300);

    esp_deep_sleep_start();

   // esp_light_sleep_start();
}
//====================================================//
