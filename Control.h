// Сделать массив из сценариев и массив индексов реле
#include <Arduino.h>
#include <TroykaCurrent.h>
#include <ESPAsyncWebServer.h>
//#include <HCSR04.h>
#include <WiFi.h>
#include <time.h>
#include <PID_v1.h>
#include <ctype.h>

#include <AsyncDelay.h>
//#include <jsnsr04t.h>
#include <structure.h>

ESP32Time rtc;

// PID переменные
double setpoint = 25.0;  // Целевая температура, °C
double input;            // Текущая температура
double output;           // Выход PID-регулятора

// PID параметры (проверьте для вашего приложения)
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

// // для теплого пола
// double Kp = 10.0;  // Пропорциональная составляющая: увеличивает реакцию на ошибку
// double Ki = 0.1;   // Интегральная составляющая: медленно корректирует систематическую ошибку
// double Kd = 2.0;   // Дифференциальная составляющая: уменьшает резкие изменения


// Создание PID-регулятора
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// датчик температуры //
const uint8_t tempPin = 32;

#define B 3950              // B-коэффициент
#define SERIAL_R 22230      //22230 10000  10230    // сопротивление последовательного резистора, 10 кОм
#define THERMISTOR_R 10000  // номинальное сопротивления термистора, 10 кОм
#define NOMINAL_T 25        // номинальная температура (при которой TR = 10 кОм)  #define B 3950 // B-коэффициент \
                            // номинальная температура (при которой TR = 10 кОм)
//==============================

//=============================

int stringToModePin(String mode) {
  if (mode == "INPUT") {
    return INPUT;
  } else if (mode == "OUTPUT") {
    return OUTPUT;
  } else if (mode == "INPUT_PULLUP") {
    return INPUT_PULLUP;
  } else if (mode == "INPUT_PULLDOWN") {
    return INPUT_PULLDOWN;
  } else {
    return OUTPUT;  // Некорректный режим, по умолчанию возвращаем 1
  }
}

//===================================================
// Функция для настройки управления
void setupControl() {
  // Проверка, что текущий индекс устройства корректен
  if (currentDeviceIndex < 0 || currentDeviceIndex >= devices.size()) {
    Serial.println("Error: currentDeviceIndex is out of range! : " + String(currentDeviceIndex) );
    return;
  }

  // Получаем текущее устройство
  Device& currentDevice = devices[currentDeviceIndex];

  // Инициализация PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Выход PID-регулятора в диапазоне 0-255

  // Инициализация пинов для реле
  for (auto& r : currentDevice.relays) {
    delay(50);
    pinMode(r.pin, stringToModePin(r.modePin));
    Serial.printf("Set pinMode for Pin=%d, Mode_StringToModePin=%d, Mode=%s\n", r.pin, stringToModePin(r.modePin), r.modePin.c_str());
    if (r.manualMode) {
      if (r.pwmMode) {
        analogWrite(r.pin, r.pwm);
      } else {
        digitalWrite(r.pin, r.statePin ? HIGH : LOW);
      }
    }
  }
}

//void updatePins() {
//  Device& currentDevice = devices[currentDeviceIndex];
//
//   for (auto& r : currentDevice.relays) {
//    if (!r.manualMode) {
//      if (r.pwmMode) {
//        analogWrite(r.pin, r.pwm);
//      } else {
//        digitalWrite(r.pin, r.statePin ? HIGH : LOW);
//      }
//    }
//  }
//}

void updatePins() {
    Device& currentDevice = devices[currentDeviceIndex];

    static std::unordered_map<int, int> lastPwmValues; // Запоминаем последние значения PWM

    for (auto& r : currentDevice.relays) {
        if (!r.manualMode) {
            if (r.pwmMode) {
                analogWrite(r.pin, r.pwm);

                // Проверяем, изменилось ли значение PWM
                if (lastPwmValues[r.pin] != r.pwm) {
                    Serial.println("PIN: " + String(r.pin) + " -> PWM: " + String(r.pwm));
                    lastPwmValues[r.pin] = r.pwm; // Обновляем сохранённое значение
                }
            } else {
                digitalWrite(r.pin, r.statePin ? HIGH : LOW);
            }
        }
    }
}


//=========================================================
// температура при которой включается скваженный насос, если меньше насос не работает. Будет по другому когда будет компрессор
float currentTemp = 0.5;
String text = "";

//
bool debug = false;
//=========================================================

float getTemp() {
  int tt = analogRead(tempPin);
  int t = map(tt, 0, 4095, 0, 1023);
  float tr = 1023.0 / t - 1;
  tr = SERIAL_R / tr;
  float steinhart;
  steinhart = tr / THERMISTOR_R;            // (R/Ro)
  steinhart = log(steinhart);               // ln(R/Ro)
  steinhart /= B;                           // 1/B * ln(R/Ro)
  steinhart += 1.0 / (NOMINAL_T + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;              // Invert
  steinhart -= 273.15 + 1;

  steinhart = (steinhart < -200) ? 5.0 : steinhart;

  return steinhart;
}
//=========================================================

void printRelay(const Relay& relay) {
  Serial.print("Pin: ");
  Serial.print(relay.pin);
  Serial.print(", Mode Pin: ");
  Serial.print(relay.modePin);
  Serial.print(", Manual Mode: ");
  Serial.print(relay.manualMode ? "true" : "false");
  Serial.print(", State Pin: ");
  Serial.print(relay.statePin ? "true" : "false");
  Serial.print(", Description: ");
  Serial.println(relay.description);
}

//======== работа с датами=================================

String convertDateFormat(const String& inputDate) {
  // Заменяем тире на точки
  String date = inputDate;
  date.replace("-", ".");

  // Переставляем местами части даты
  String day = date.substring(8, 10);
  String month = date.substring(5, 7);
  String year = date.substring(0, 4);

  // Собираем результат в формате DD.MM.YYYY
  return day + "." + month + "." + year;
}

// Функция для проверки, состоит ли строка только из цифр
bool isNumeric(const String& str) {
  for (size_t i = 0; i < str.length(); i++) {
    if (!isdigit(str.charAt(i))) {
      return false;
    }
  }
  return true;
}

bool isValidDateTime(const String& dateTime) {
  // Находим индекс пробела, который разделяет дату и время
  int spaceIndex = dateTime.indexOf(' ');
  if (spaceIndex == -1) return false;  // Пробел отсутствует

  // Разделяем строку на дату и время
  String date = dateTime.substring(0, spaceIndex);
  String time = dateTime.substring(spaceIndex + 1);

  // Находим разделители в дате (точки)
  int dot1 = date.indexOf('.');
  int dot2 = date.lastIndexOf('.');

  // Проверяем, что разделители есть и правильно расположены
  if (dot1 == -1 || dot2 == -1 || dot1 == dot2) return false;

  // Разделяем дату на день, месяц и год
  String day = date.substring(0, dot1);
  String month = date.substring(dot1 + 1, dot2);
  String year = date.substring(dot2 + 1);

  // Проверяем, что все части даты содержат только цифры
  if (!isNumeric(day) || !isNumeric(month) || !isNumeric(year)) return false;

  // Преобразуем в числа
  int dayInt = day.toInt();
  int monthInt = month.toInt();
  int yearInt = year.toInt();

  // Проверяем диапазоны для дня, месяца и года
  if (dayInt < 1 || dayInt > 31 || monthInt < 1 || monthInt > 12 || yearInt < 1000 || yearInt > 9999) {
    return false;
  }

  // Проверяем формат времени (должно быть HH:MM, H:MM или HH:M)
  int colonIndex = time.indexOf(':');
  if (colonIndex == -1) return false;  // Нет двоеточия

  String hour = time.substring(0, colonIndex);
  String minute = time.substring(colonIndex + 1);

  // Проверяем, что оба значения содержат только цифры
  if (!isNumeric(hour) || !isNumeric(minute)) return false;

  // Преобразуем время в числа
  int hourInt = hour.toInt();
  int minuteInt = minute.toInt();

  // Проверяем диапазоны для времени
  if (hourInt < 0 || hourInt > 23 || minuteInt < 0 || minuteInt > 59) {
    return false;
  }

  return true;
}

std::pair<String, String> splitDateTime(const String& dateTime) {
  // Разделяем строку на дату и время
  int spaceIndex = dateTime.indexOf(' ');

  String date = dateTime.substring(0, spaceIndex);
  String time = dateTime.substring(spaceIndex + 1);

  // Разделим дату на день, месяц, год
  int dot1 = date.indexOf('.');
  int dot2 = date.lastIndexOf('.');

  String day = date.substring(0, dot1);           // День
  String month = date.substring(dot1 + 1, dot2);  // Месяц
  String year = date.substring(dot2 + 1);         // Год

  // Добавляем ведущий ноль, если день или месяц состоит из одной цифры
  day = day.length() == 1 ? "0" + day : day;
  month = month.length() == 1 ? "0" + month : month;

  // Формируем строку в формате YYYY-MM-DD
  String formattedDate = year + "-" + month + "-" + day;

  // Возвращаем кортеж (пару): дату и время
  return std::make_pair(formattedDate, time);
}

//=======================
// Функция для отладки и вывода информации
String debugInfo() {
  String debugString;

  // Проверка, что текущий индекс устройства корректен
  if (currentDeviceIndex < 0 || currentDeviceIndex >= devices.size()) {
    debugString += "Error: currentDeviceIndex is out of range!\n";
    return debugString;
  }

  // Получаем текущее устройство
  Device& currentDevice = devices[currentDeviceIndex];

  debugString += "\nРеле:\n";
  for (const auto& relay : currentDevice.relays) {
    debugString += "Pin: " + String(relay.pin);          // Номер пина
    debugString += ", ModePin: " + relay.modePin;        // Режим пина
    debugString += ", ManualMode: " + String(relay.manualMode);
    debugString += ", statePin: " + String(digitalRead(relay.pin));
    debugString += ", Description: " + relay.description;
    debugString += "\n";
  }

  debugString += "\nДатчики и кнопки:\n";
  debugString += "Temp Pin: " + String(tempPin) + "\n";  // Пример для датчика температуры

  if (Serial.available() > 0) {
    Serial.println(debugString);
  }
  return debugString;
}

// Функция для ручного управления
void manualWork(String arg) {
  if (arg == "?\n" || arg == "status\n") {
    debugInfo();
  }
  debug = (arg == "debug\n");
}
//=========================================================


// Функция для вывода состояния реле
void printRelayStates(void (*callFunc)(String)) {

  // Получаем текущее устройство
  Device& currentDevice = devices[currentDeviceIndex];

  // Перебираем все реле текущего устройства
  for (auto& relay : currentDevice.relays) {
    int statePin = digitalRead(relay.pin);  // Читаем текущее состояние реле

    // Проверяем, изменилось ли состояние
    if (statePin != relay.lastState) {
      String sOut = relay.description;
      sOut += ": ";
      sOut += (statePin == HIGH) ? "включено" : "отключено";  // включено / отключено
      sOut += "\n";

      callFunc(sOut);  // Вызываем переданную функцию для вывода состояния

      relay.lastState = statePin;  // Обновляем предыдущее состояние
    }
  }
}

//=================[Главный сценарий]==================//

int shiftWeekDay(int currentDay) {
  if (currentDay == 0) {  // Если воскресенье
    return 6;             // В воскресенье будет индекс 6
  } else {
    return currentDay - 1;  // Остальные дни сдвигаем на 1
  }
}

time_t convertToTimeT(const String& date) {
  tm t = {};
  sscanf(date.c_str(), "%4d-%2d-%2d", &t.tm_year, &t.tm_mon, &t.tm_mday);

  t.tm_year -= 1900;  // Годы с 1900
  t.tm_mon -= 1;      // Месяцы от 0 до 11

  // Сбрасываем время на 00:00:00
  t.tm_hour = 0;
  t.tm_min = 0;
  t.tm_sec = 0;

  return mktime(&t);
}


time_t getCurrentTimeFromRTC() {
  time_t now = rtc.getEpoch();  // Получаем текущее время из RTC
  return now;                   // Возвращаем как time_t
}

String formatDateTime(time_t rawTime) {
  struct tm timeInfo;
  localtime_r(&rawTime, &timeInfo);  // Преобразуем time_t в структуру tm

  // Форматируем строку: DD.MM.YYYY HH:MM
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%02d.%02d.%04d %02d:%02d",
           timeInfo.tm_mday,
           timeInfo.tm_mon + 1,      // Месяц в tm начинается с 0
           timeInfo.tm_year + 1900,  // Год в tm начинается с 1900
           timeInfo.tm_hour,
           timeInfo.tm_min);

  return String(buffer);
}

time_t convertOnlyDate(time_t rawTime) {
  struct tm timeInfo;
  localtime_r(&rawTime, &timeInfo);  // Преобразуем time_t в структуру tm

  // Сбрасываем время на 00:00:00
  timeInfo.tm_hour = 0;
  timeInfo.tm_min = 0;
  timeInfo.tm_sec = 0;

  // Преобразуем обратно в time_t
  return mktime(&timeInfo);
}

//=================================================

static unsigned long lastUpdate = 0;

String msg[20] = { "", "", "", "", "", "" };
String lastMsg[20] = { "", "", "", "", "", "" };

bool startTimer1 = false;
int timer1 = 0;

// Состояние реле
bool relay1State = false;


//===============================================
int convertTimeToSeconds(const String& timeStr) {
  int hours = timeStr.substring(0, 2).toInt();
  int minutes = timeStr.substring(3, 5).toInt();
  int seconds = timeStr.substring(6, 8).toInt();
  return hours * 3600 + minutes * 60 + seconds;
}

//===============================================

void controlOutputs(OutPower& outPower) {
  // Проверяем, что индекс outPower в пределах массива
 
    uint8_t relayIndex = outPower.relayIndex;
    
    // Проверяем, что relayIndex в допустимых пределах
    if (relayIndex < devices[currentDeviceIndex].relays.size()) {
      // Получаем ссылку на реле
      Relay& relay = devices[currentDeviceIndex].relays[relayIndex];

      // Копируем параметры
      relay.pwm = outPower.pwm;
      relay.statePin = outPower.statePin;
      relay.pwmMode = outPower.pwmMode;
    }
}

void controlTemperature() { 
    Temperature& tempControl = devices[currentDeviceIndex].temperature;

    if (!tempControl.isUseSetting) return;  // Выход из функции, если регулирование отключено

    uint8_t relayIndex = tempControl.relayIndex;
    if (relayIndex >= devices[currentDeviceIndex].relays.size()) return;  // Проверка границ

    Relay& relay = devices[currentDeviceIndex].relays[relayIndex];

    // Читаем текущую температуру (допустим, она где-то обновляется)
    input = currentTemp; 
    setpoint = tempControl.setTemperature;

    // Запускаем PID-регулятор
    myPID.Compute();

    if (tempControl.isSmoothly) {
        // Режим ШИМ (плавное регулирование)
        relay.pwm = constrain(output, 0, 255);
    } else {
        // Режим вкл/выкл
        bool shouldActivate = tempControl.isIncrease ? (input < setpoint) : (input > setpoint);
        relay.statePin = shouldActivate;
    }
}

//===============================================

void collectionSettingsTimer(uint8_t currentTimerIndex) { 
  Timer& currentTimer = devices[currentDeviceIndex].timer[currentTimerIndex];

  if (currentTimer.collectionSettings[0]) { // Установка мощности
    devices[currentDeviceIndex].outPower.isUseSetting = true;
  }

  if (currentTimer.collectionSettings[1]) { // активация управление температурой
    devices[currentDeviceIndex].temperature.isUseSetting = true;
  }

   if (currentTimer.collectionSettings[2]) { // активация управление графиком мощности
    devices[currentDeviceIndex].powerGraph.isUseSetting = true;
  }

   if (currentTimer.collectionSettings[3]) { // активация управление графиком температуры
    devices[currentDeviceIndex].temperatureGraph.isUseSetting = true;
  }

 if (currentTimer.collectionSettings[4]) { //  состояние реле при завершении таймера
      controlOutputs(currentTimer.endlStateRelay);
    
  }
  
}

void executeTimers(bool state = true) {
  static size_t currentTimerIndex = 0;
  static unsigned long timerStartTime = 0;  // Время запуска текущего таймера
  static bool timerRunning = false;  // Флаг, что таймер активен

  if (!devices[currentDeviceIndex].isTimersEnabled || devices[currentDeviceIndex].timer.empty()) {
    return; // Если таймеры выключены или их нет, выходим
  }

  Timer& currentTimer = devices[currentDeviceIndex].timer[currentTimerIndex];

  // Применяем начальные условия перед запуском таймера
  controlOutputs(currentTimer.initialStateRelay);
  
  // Если state = false, просто выходим, не выполняя установку конечных состояний
  if (!state) {
    return;
  }

  // Если таймер еще не запущен — инициализируем его
  if (!timerRunning) {
    timerStartTime = millis(); // Запоминаем время запуска в миллисекундах
    timerRunning = true;
  }

  // Переводим строку времени из таймера в секунды
  int timerDuration = convertTimeToSeconds(currentTimer.time);

  // Проверяем, прошло ли заданное время
  if (millis() - timerStartTime >= timerDuration * 1000) {
    // Вызываем collectionSettingsTimer, так как таймер сработал
    collectionSettingsTimer(currentTimerIndex);

    // Сбрасываем флаг работы таймера
    timerRunning = false;

    // Переход к следующему таймеру
    currentTimerIndex++;

    // Если дошли до конца списка таймеров
    if (currentTimerIndex >= devices[currentDeviceIndex].timer.size()) {
      if (devices[currentDeviceIndex].isEncyclateTimers) {
        currentTimerIndex = 0; // Запускаем таймеры заново с первого
      } else {
        // Если цикличность отключена — останавливаем таймеры
        currentTimerIndex = devices[currentDeviceIndex].timer.size(); // Выходим из цикла
      }
    }
  }
}

//===============================================================
void collectionSettings(bool start) { 
  static bool outputsUpdated = false;   // Флаг для обновления выходов
  static bool timersExecuted = false;   // Флаг для остановки таймеров

  ScheduleScenario& scenario = devices[currentDeviceIndex].scheduleScenario;

  if (start) {
    // Установка мощности
    if (scenario.collectionSettings[0]) { 
      devices[currentDeviceIndex].outPower.isUseSetting = true;
    }

    // Управление температурой
    if (scenario.collectionSettings[1]) { 
      devices[currentDeviceIndex].temperature.isUseSetting = true;
    }

    // Управление графиком мощности
    if (scenario.collectionSettings[2]) { 
      devices[currentDeviceIndex].powerGraph.isUseSetting = true;
    }

    // Управление графиком температуры
    if (scenario.collectionSettings[3]) { 
      devices[currentDeviceIndex].temperatureGraph.isUseSetting = true;
    }

    // Управление таймерами
    if (scenario.collectionSettings[4]) { 
      devices[currentDeviceIndex].isTimersEnabled = true;
    }

     if (scenario.collectionSettings[5]) { 
      controlOutputs(scenario.initialStateRelay);
    }
    
  } else {
    // Отключение мощности
    if (scenario.collectionSettings[0] && !outputsUpdated) { 
      devices[currentDeviceIndex].outPower.isUseSetting = false;
      outputsUpdated = true;
    }

    // Отключение управления температурой
    if (scenario.collectionSettings[1]) { 
      devices[currentDeviceIndex].temperature.isUseSetting = false;
    }

    // Отключение управления графиком мощности
    if (scenario.collectionSettings[2]) { 
      devices[currentDeviceIndex].powerGraph.isUseSetting = false;
    }

    // Отключение управления графиком температуры
    if (scenario.collectionSettings[3]) { 
      devices[currentDeviceIndex].temperatureGraph.isUseSetting = false;
    }

    // Отключение таймеров
    if (scenario.collectionSettings[4] && !timersExecuted) { 
      devices[currentDeviceIndex].isTimersEnabled = false;
      timersExecuted = true;
    }

    if (scenario.collectionSettings[5]) { 
      controlOutputs(scenario.endlStateRelay);
    }

  // Сбрасываем флаги при новом запуске
  if (start) {
    timersExecuted = false;
    outputsUpdated = false;
   } 
  } // end start - else
}

//===============================================
void mainScenario(void (*callFunc)(String)) { 
  if (millis() - lastUpdate >= 200) {  
    lastUpdate = millis();

    if (Serial.available() > 0) {
      String arg = Serial.readString();
      manualWork(arg);
      if (debug) {
        debugInfo();
      }
    }

    ScheduleScenario& scenario = devices[currentDeviceIndex].scheduleScenario; // Теперь без цикла

    if (scenario.isUseSetting) {
      msg[0] = "Включение сценария";
      if (msg[0] != lastMsg[0]) {
        Serial.println(msg[0]);
        lastMsg[0] = msg[0];
        callFunc(msg[0]);
      }

      time_t startDate = convertToTimeT(scenario.startDate);
      time_t endDate = convertToTimeT(scenario.endDate);
      time_t currentDateTime = getCurrentTimeFromRTC();
      time_t currentDate = convertOnlyDate(currentDateTime);

      struct tm currentTime;
      localtime_r(&currentDateTime, &currentTime);

      if (currentDate >= startDate && currentDate <= endDate) {
        msg[1] = "Текущая дата в установленном диапазоне";
        if (msg[1] != lastMsg[1]) {
          Serial.println(msg[1]);
          lastMsg[1] = msg[1];
          callFunc(msg[1]);
        }

        if (scenario.months[currentTime.tm_mon]) {  // Проверка месяца
          msg[5] = "Текущий месяц включен в массиве месяцев";
          if (msg[5] != lastMsg[5]) {
            Serial.println(msg[5]);
            lastMsg[5] = msg[5];
            callFunc(msg[5]);
          }

          if (scenario.week[shiftWeekDay(currentTime.tm_wday)]) {
            msg[2] = "Текущий день недели включен в массиве недели";
            if (msg[2] != lastMsg[2]) {
              Serial.println(msg[2]);
              lastMsg[2] = msg[2];
              callFunc(msg[2]);
            }

            int currentMinutes = currentTime.tm_hour * 60 + currentTime.tm_min;
            int startMinutes = scenario.startTime.substring(0, 2).toInt() * 60 + scenario.startTime.substring(3, 5).toInt();
            int endMinutes = scenario.endTime.substring(0, 2).toInt() * 60 + scenario.endTime.substring(3, 5).toInt();

            if (currentMinutes >= startMinutes && currentMinutes <= endMinutes) {
              msg[3] = "Текущие минуты в установленном диапазоне";
              if (msg[3] != lastMsg[3]) {
                Serial.println(msg[3]);
                lastMsg[3] = msg[3];
                callFunc(msg[3]);
              }

              msg[8] = "Запуск настроек";
              if (msg[8] != lastMsg[8]) {
                Serial.println(msg[8]);
                lastMsg[8] = msg[8];
                callFunc(msg[8]);
              }
              collectionSettings(true);

            } else {
              msg[3] = "Текущие минуты вне установленного диапазона";
              if (msg[3] != lastMsg[3]) {
                Serial.println(msg[3]);
                lastMsg[3] = msg[3];
                callFunc(msg[3]);
              }
              collectionSettings(false);
            }
          } else {
            msg[2] = "Текущий день вне массива недели";
            if (msg[2] != lastMsg[2]) {
              Serial.println(msg[2]);
              lastMsg[2] = msg[2];
              callFunc(msg[2]);
            }
            collectionSettings(false);
          }
        } else {
          msg[5] = "Текущий месяц вне массива месяцев";
          if (msg[5] != lastMsg[5]) {
            Serial.println(msg[5]);
            lastMsg[5] = msg[5];
            callFunc(msg[5]);
          }
          collectionSettings(false);
        }
      } else {
        msg[1] = "Текущая дата вне установленного диапазона";
        if (msg[1] != lastMsg[1]) {
          Serial.println(msg[1]);
          lastMsg[1] = msg[1];
          callFunc(msg[1]);
        }
        collectionSettings(false);
      }
    } else {
      msg[0] = "Выключение сценария";
      if (msg[0] != lastMsg[0]) {
        Serial.println(msg[0]);
        lastMsg[0] = msg[0];
        callFunc(msg[0]);
      }
      collectionSettings(false);
    }

    printRelayStates(callFunc);
  } 
}

//====================================================

void controlTask() {

  if (startTimer1) {
    timer1++;
  } else {
    timer1 = 0;
  }
}

//=========================================================
String sendStatus() { 
  // Проверка, что текущий индекс устройства корректен
  if (currentDeviceIndex < 0 || currentDeviceIndex >= devices.size()) {
    return "Ошибка: текущее устройство не найдено!";
  }

  // Получаем текущее устройство
  Device& currentDevice = devices[currentDeviceIndex];

  // Получаем текущий IP-адрес и SSID подключения
  String ipAddress = WiFi.localIP().toString();
  String ssid = WiFi.SSID();

  // Строка для справки с режимом работы каждого реле
  String helpText = "";
  helpText += "Текущий статус устройства: " + currentDevice.nameDevice + "\n\n";

  // Добавляем информацию о состоянии каждого реле
  helpText += "Реле:\n";
  int relayIndex = 1;  // Индекс для команд /on и /off
  for (const auto& relay : currentDevice.relays) {
    String commandOn = "/on" + String(relayIndex);
    String commandOff = "/off" + String(relayIndex);
    String mode_ = relay.manualMode ? "Ручной" : "Авто";
    String state_ = digitalRead(relay.pin) ? "Вкл" : "Выкл";
    String pwmInfo = relay.pwmMode ? " (ШИМ: " + String(relay.pwm) + ")" : "";
    helpText += commandOn + " " + commandOff + " - " + relay.description + " (" + mode_ + " / " + state_ + pwmInfo + ")\n\n";
    relayIndex++;
  }

  // Добавляем информацию о времени и температуре
  helpText += "Время на устройстве: [" + formatDateTime(getCurrentTimeFromRTC()) + "]\n";
  helpText += "Температура: " + String(currentTemp) + " °С\n\n";

  // Добавляем информацию о расписании (если есть сценарии)
  
    const ScheduleScenario& scenario = currentDevice.scheduleScenario; // Берем первый сценарий
    String tmpMsg = convertDateFormat(scenario.startDate) + " " + scenario.startTime + " по " + convertDateFormat(scenario.endDate) + " " + scenario.endTime;
    if (scenario.isUseSetting) {
      helpText += "Включено. Установлено расписание: [" + tmpMsg + "]\n";
    } else {
      helpText += "Отключено. Установлено расписание: [" + tmpMsg + "]\n";
    }
  
  helpText += "\n";

  // Добавляем команды управления
  helpText += "/startTime - включить расписание\n";
  helpText += "/stopTime - отменить расписание\n";
  helpText += "/setTime - установить расписание\n";
  helpText += "/resetManual - Сбросить в Auto\n";
  helpText += "/status - Статус и управление\n";
  helpText += "/pushOn /pushOff - Вкл/Откл уведомления\n";
  helpText += "/help - Справка\n\n";

  // Добавляем IP-адрес и SSID
  helpText += "Доступ из сети: http://" + ipAddress + "\n";
  helpText += "Имя сети Wi-Fi: " + ssid + "\n\n";

  return helpText; 
}

//===============================

String sendHelp() {
  String helpMessage = "Доступные команды:\n";

  return helpMessage;
}
