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
const int tempPin = 32;

#define B 3950              // B-коэффициент
#define SERIAL_R 22230      //22230 10000  10230    // сопротивление последовательного резистора, 10 кОм
#define THERMISTOR_R 10000  // номинальное сопротивления термистора, 10 кОм
#define NOMINAL_T 25        // номинальная температура (при которой TR = 10 кОм)  #define B 3950 // B-коэффициент \
                            // номинальная температура (при которой TR = 10 кОм)
//==============================
bool isSaveControl = false;
//=============================

struct Scenario {
  bool useSetting;           // Использовать настройку
  int temperature;           // Температура
  bool temperatureCheckbox;  // Включить температуру
  String startDate;          // Дата и время включения
  String startTime;          // Время включения
  String endDate;            // Дата и время выключения
  String endTime;            // Время выключения
  int pinRelays;
  int pinRelays2;  // Номер пина реле
  //std::vector<int> indexRelay;
  int timeInterval;
  bool week[7];
};


struct Relay {
  int pin;
  String modePin;  // INPUT, OUTPUT, INPUT_PULLUP или INPUT_PULLDOWN
  bool manualMode;
  bool statePin;
  bool lastState;
  String description;
};

struct Control {
  std::vector<Relay> relays;
  std::vector<int> pins = { 23, 22, 1, 3, 21, 19, 18, 5, 17, 16, 33, 32 };  // only input - 35, 34, 39, 36
  Scenario scenario; 
};

Control control;

// Функция для инициализации реле в структуре управления
void initializeControl() {
  // Инициализация массива реле
  control.relays = {
    { 5, "OUTPUT", false, false, false, "Реле_1" },
    { 18, "OUTPUT", false, false, false, "Реле_2" },
  };

  // Инициализация структуры сценария
  control.scenario = {
    false,                                        // Использовать настройку (например, true)
    22,                                           // Температура (например, 22°C)
    true,                                         // Включить температуру (например, true)
    "2024-12-29",                                 // Дата включения (например, "2024-12-29")
    "08:00",                                      // Время включения (например, "08:00")
    "2024-12-29",                                 // Дата выключения (например, "2024-12-29")
    "10:00",                                      // Время выключения (например, "20:00")
    5,                                            // Номер пина реле 1
    18,                                           // Номер пина реле 2
    40,                                           // Интервал времени (например, 10 минут)
    { true, true, true, true, true, true, true }  // Все дни недели активны
  };
}

//==================================
// температура при которой включается скваженный насос, если меньше насос не работает. Будет по другому когда будет компрессор
float currentTemp = 0.5;
String text = "";

//
bool debug = false;
//====================================================//

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
//===================================

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

String modePinToString(int modePin) {

  switch (modePin) {
    case INPUT: return "INPUT";                    // 1
    case OUTPUT: return "OUTPUT";                  // 3
    case INPUT_PULLUP: return "INPUT_PULLUP";      // 5
    case INPUT_PULLDOWN: return "INPUT_PULLDOWN";  // 9
    default: return "UNKNOWN";                     // Для обработки некорректных значений
  }
}

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

//======== работа с датами

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

void setupControl() {
  // Кнопки датчики //
  // Инициализация PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Выход PID-регулятора в диапазоне 0-255

  //pinMode(sensButtonPin_1, INPUT_PULLDOWN);

  for (auto& r : control.relays) {
    delay(50);
    pinMode(r.pin, stringToModePin(r.modePin));
    Serial.printf("Set pinMode for Pin=%d, Mode_StringToModePin=%d , Mode=%s\n", r.pin, stringToModePin(r.modePin), r.modePin);
    if (r.manualMode) {
      digitalWrite(r.pin, r.statePin ? HIGH : LOW);  // Устанавливаем состояние пина
      //r.lastState = r.statePin;
    }
  }
}
//======================
String debugInfo() {
  String debugString;

  debugString += "\nРеле:\n";
  for (const auto& relay : control.relays) {
    debugString += "Pin: " + String(relay.pin);          // modePin
    debugString += "ModePin: " + String(relay.modePin);  // modePin
    debugString += ", ManualMode: " + String(relay.manualMode);
    debugString += ", statePin: " + String(digitalRead(relay.pin));
    debugString += ", Description: " + relay.description;
    debugString += "\n";
  }

  debugString += "\nДатчики и кнопки:\n";
  debugString += "Temp Pin: " + String(tempPin) + "\n";

  if (Serial.available() > 0) {
    Serial.println(debugString);
  }
  return debugString;
}

void manualWork(String arg) {
  if (arg == "?\n" || arg == "status\n") {
    debugInfo();
  }
  debug = (arg == "debug\n");
}

//=====================================================



// универсальный тогл
// void toggleRelays(Relay relays[], int relayCount, unsigned long interval) {
//     static int currentRelayIndex = 0;         // Текущий индекс реле
//     static unsigned long lastToggleTime = 0; // Время последнего переключения

//     unsigned long currentTime = millis(); // Текущее время

//     // Проверяем, прошел ли временной интервал
//     if (currentTime - lastToggleTime >= interval) {
//         // Выключаем текущее реле
//         digitalWrite(relays[currentRelayIndex].pin, LOW);
//         relays[currentRelayIndex].state = false;

//         // Переходим к следующему реле
//         currentRelayIndex = (currentRelayIndex + 1) % relayCount;

//         // Включаем следующее реле
//         digitalWrite(relays[currentRelayIndex].pin, HIGH);
//         relays[currentRelayIndex].state = true;

//         // Отмечаем время переключения
//         lastToggleTime = currentTime;

//         // Выводим отчет в Serial
//         Serial.print("Relay ");
//         Serial.print(currentRelayIndex);
//         Serial.println(" ON");

//         Serial.print("Time since last toggle: ");
//         Serial.print(currentTime - lastToggleTime);
//         Serial.println(" ms");
//     }
// }



//=====================================================//

void printRelayStates(void (*callFunc)(String)) {
  for (auto& relay : control.relays) {
    int statePin = digitalRead(relay.pin);  // Читаем текущее состояние реле

    // Проверяем, изменилось ли состояние
    if (statePin != relay.lastState) {
      String sOut = relay.description;
      sOut += ": ";
      sOut += statePin ? "включено" : "отключено";  // включено / отключено
      sOut += "\n";

      callFunc(sOut);

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

void toggleRelays() {
  int relay1Pin = control.scenario.pinRelays;
  int relay2Pin = control.scenario.pinRelays2;

  if (relay1State) {

    msg[5] = "Relay2 On, Relay1 Off";
    if (msg[5] != lastMsg[5]) {
      Serial.println(msg[5]);
      lastMsg[5] = msg[5];

      digitalWrite(relay1Pin, LOW);
      digitalWrite(relay2Pin, HIGH);
      relay1State = false;
    }
  } else {

    msg[5] = "Relay2 Off, Relay1 On";
    if (msg[5] != lastMsg[5]) {
      Serial.println(msg[5]);
      lastMsg[5] = msg[5];

      digitalWrite(relay2Pin, LOW);
      digitalWrite(relay1Pin, HIGH);
      relay1State = true;
    }
  }
}

void scenarioRele(bool start) {
  int relayInterval = control.scenario.timeInterval;
  int relay1Pin = control.scenario.pinRelays;
  int relay2Pin = control.scenario.pinRelays2;

  bool manualMode1 = false;
  bool manualMode2 = false;

  // Проверяем ручной режим
  for (auto& r : control.relays) {
    if (r.pin == relay1Pin) {
      manualMode1 = r.manualMode;
    }
    if (r.pin == relay2Pin) {
      manualMode2 = r.manualMode;
    }
  }

  if (!(manualMode1 || manualMode2)) {
    if (start) {
      // Запускаем таймер
      startTimer1 = true;

      if (timer1 == 0) {
        toggleRelays();
        timer1 = 1;
      }

      if (timer1 >= relayInterval) {
        timer1 = 0;  // Сбрасываем таймер после переключения
      }
    } else {
      // Останавливаем сценарий и отключаем оба реле
      startTimer1 = false;
      digitalWrite(relay1Pin, LOW);
      digitalWrite(relay2Pin, LOW);

      msg[5] = "Relays OFF";
      if (msg[5] != lastMsg[5]) {
        Serial.println(msg[5]);
        lastMsg[5] = msg[5];
      }
    }
  }
}

//=================================================

void mainScenario(void (*callFunc)(String)) {

  if (millis() - lastUpdate >= 200) {  //delay(200);
    lastUpdate = millis();

    currentTemp = getTemp();
    setpoint = control.scenario.temperature;
    input = currentTemp;
    myPID.Compute();

    if (Serial.available() > 0) {
      String arg = Serial.readString();
      manualWork(arg);
      if (debug) {
        debugInfo();
      }
    }

    if (control.scenario.useSetting) {

      msg[0] = "Включение сценария";
      if (msg[0] != lastMsg[0]) {
        Serial.println(msg[0]);
        lastMsg[0] = msg[0];
         callFunc(msg[0]);
      }

      time_t startDate = convertToTimeT(control.scenario.startDate);
      time_t endDate = convertToTimeT(control.scenario.endDate);
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

        if (control.scenario.week[shiftWeekDay(currentTime.tm_wday)]) {
          msg[2] = "Текущий день включен в массиве недели";
          if (msg[2] != lastMsg[2]) {
            Serial.println(msg[2]);
            lastMsg[2] = msg[2];
            callFunc(msg[2]);
          }

          int currentMinutes = currentTime.tm_hour * 60 + currentTime.tm_min;
          int startMinutes = control.scenario.startTime.substring(0, 2).toInt() * 60 + control.scenario.startTime.substring(3, 5).toInt();
          int endMinutes = control.scenario.endTime.substring(0, 2).toInt() * 60 + control.scenario.endTime.substring(3, 5).toInt();

          if (currentMinutes >= startMinutes && currentMinutes <= endMinutes) {
            msg[3] = "Текущие минуты больше установленных";
            if (msg[3] != lastMsg[3]) {
              Serial.println(msg[3]);
              lastMsg[3] = msg[3];
              callFunc(msg[3]);
            }

            if (control.scenario.temperatureCheckbox) {
              if (currentTemp < control.scenario.temperature) { // currentTemp < control.scenario.temperature // output > control.scenario.temperature

                msg[4] = "Текущая температура меньше установленной";
                if (msg[4] != lastMsg[4]) {
                  Serial.println(msg[4]);
                  lastMsg[4] = msg[4];
                  callFunc(msg[4]);
                }
                scenarioRele(true);

              } else {
                msg[4] = "Текущая температура больше установленной";
                if (msg[4] != lastMsg[4]) {
                  Serial.println(msg[4]);
                  lastMsg[4] = msg[4];
                  callFunc(msg[4]);
                }
                scenarioRele(false);
              }
            } else {
              msg[8] = "Слежение за температурой отключено";
              if (msg[8] != lastMsg[8]) {
                Serial.println(msg[8]);
                lastMsg[8] = msg[8];
                callFunc(msg[8]);
              }
              scenarioRele(true);
            }
          } else {
            msg[3] = "Текущие минуты меньше установленных";
            if (msg[3] != lastMsg[3]) {
              Serial.println(msg[3]);
              lastMsg[3] = msg[3];
              callFunc(msg[3]);
            }
            scenarioRele(false);
          }
        } else {
          msg[2] = "Текущий день вне массива недели";
          if (msg[2] != lastMsg[2]) {
            Serial.println(msg[2]);
            lastMsg[2] = msg[2];
            callFunc(msg[2]);
          }
          scenarioRele(false);
        }
      } else {
        msg[1] = "Текущая дата вне установленного диапазона";
        if (msg[1] != lastMsg[1]) {
          Serial.println(msg[1]);
          lastMsg[1] = msg[1];
          callFunc(msg[1]);
        }
        scenarioRele(false);
      }
    } else {
      msg[0] = "Выключение сценария";
      if (msg[0] != lastMsg[0]) {
        Serial.println(msg[0]);
        lastMsg[0] = msg[0];
        callFunc(msg[0]);
      }
      scenarioRele(false);
    }

    printRelayStates(callFunc);

  }  // millis end
}
//====================================================

void controlTask() {

  if (startTimer1) {
    timer1++;
  } else {
    timer1 = 0;
  }
}
//======================================================
// Функция для сохранения структуры Control в SPIFFS
void saveControlToSPIFFS() {
  isSaveControl = true;
  // Открытие файла для записи
  File file = SPIFFS.open("/control.json", FILE_WRITE);
  if (!file) {
    isSaveControl = false;
    Serial.println("Не удалось открыть файл для записи");
    return;
  }

  // Создание JSON документа
  DynamicJsonDocument doc(2048);
  JsonArray relays = doc.createNestedArray("relays");

  for (auto& relay : control.relays) {
    JsonObject relayObj = relays.createNestedObject();
    relayObj["pin"] = relay.pin;
    relayObj["modePin"] = relay.modePin;
    relayObj["manualMode"] = relay.manualMode;
    relayObj["statePin"] = relay.statePin;
    relayObj["description"] = relay.description;

    Serial.printf("saveControlToSPIFFS: Pin=%d, Mode=%s, ManualMode=%d, StatePin=%d, Description=%s\n",
                  relay.pin, relay.modePin.c_str(), relay.manualMode, relay.statePin, relay.description.c_str());
  }

  // Добавляем данные из структуры Scenario
  JsonObject scenario = doc.createNestedObject("scenario");
  scenario["useSetting"] = control.scenario.useSetting;
  scenario["temperature"] = control.scenario.temperature;
  scenario["temperatureCheckbox"] = control.scenario.temperatureCheckbox;
  scenario["startDate"] = control.scenario.startDate;
  scenario["startTime"] = control.scenario.startTime;
  scenario["endDate"] = control.scenario.endDate;
  scenario["endTime"] = control.scenario.endTime;
  scenario["pinRelays"] = control.scenario.pinRelays;
  scenario["pinRelays2"] = control.scenario.pinRelays2;
  scenario["timeInterval"] = control.scenario.timeInterval;

  // Добавляем данные о днях недели
  JsonArray week = scenario.createNestedArray("week");
  for (int i = 0; i < 7; i++) {
    week.add(control.scenario.week[i]);
  }

  // Сериализация JSON документа в файл
  if (serializeJson(doc, file) == 0) {
    Serial.println("Ошибка при записи в файл");
  }

  file.close();
  isSaveControl = false;
}
//=======================================================
// Функция для чтения структуры Control из SPIFFS
void loadControlFromSPIFFS() {
  // Открытие файла для чтения
  File file = SPIFFS.open("/control.json", FILE_READ);
  if (!file) {
    Serial.println("Не удалось открыть файл для чтения");
    initializeControl();
    saveControlToSPIFFS();
    delay(100);
    return;
  }

  // Создание JSON документа
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.println("loadControl: Ошибка при чтении файла");
    delay(1000);
    initializeControl();
    delay(500);
    saveControlToSPIFFS();
    delay(1000);
    return;
  }

  // Чтение данных из JSON документа
  JsonArray relays = doc["relays"];
  control.relays.clear();  // Очистка вектора перед загрузкой новых данных
  for (JsonObject relayObj : relays) {
    Relay relay;
    relay.pin = relayObj["pin"].as<int>();
    relay.modePin = relayObj["modePin"].as<String>();
    relay.manualMode = relayObj["manualMode"].as<bool>();
    relay.statePin = relayObj["statePin"].as<bool>();
    relay.description = relayObj["description"].as<String>();
    control.relays.push_back(relay);

    Serial.printf("loadControlFromSPIFFS: Pin=%d, Mode=%s, ManualMode=%d, StatePin=%d, Description=%s\n",
                  relay.pin, relay.modePin.c_str(), relay.manualMode, relay.statePin, relay.description.c_str());
  }

  // Чтение данных из структуры Scenario
  JsonObject scenario = doc["scenario"];
  control.scenario.useSetting = scenario["useSetting"].as<bool>();
  control.scenario.temperature = scenario["temperature"].as<int>();
  control.scenario.temperatureCheckbox = scenario["temperatureCheckbox"].as<bool>();
  control.scenario.startDate = scenario["startDate"].as<String>();
  control.scenario.startTime = scenario["startTime"].as<String>();
  control.scenario.endDate = scenario["endDate"].as<String>();
  control.scenario.endTime = scenario["endTime"].as<String>();
  control.scenario.pinRelays = scenario["pinRelays"].as<int>();
  control.scenario.pinRelays2 = scenario["pinRelays2"].as<int>();
  control.scenario.timeInterval = scenario["timeInterval"].as<int>();


  // Чтение данных о днях недели
  JsonArray week = scenario["week"];
  for (int i = 0; i < 7; i++) {
    control.scenario.week[i] = week[i].as<bool>();
  }

  Serial.printf("loadControlFromSPIFFS Scenario: UseSetting=%d, Temperature=%d, TemperatureCheckbox=%d, StartDate=%s, StartTime=%s, EndDate=%s, EndTime=%s, PinRelays=%d, Week(monday=%d, tuesday=%d, wednesday=%d, thursday=%d, friday=%d, saturday=%d, sunday=%d)\n",
                control.scenario.useSetting, control.scenario.temperature, control.scenario.temperatureCheckbox, control.scenario.startDate.c_str(),
                control.scenario.startTime.c_str(), control.scenario.endDate.c_str(), control.scenario.endTime.c_str(), control.scenario.pinRelays,
                control.scenario.week[0], control.scenario.week[1], control.scenario.week[2], control.scenario.week[3],
                control.scenario.week[4], control.scenario.week[5], control.scenario.week[6]);

  file.close();
}

//============

String sendStatus() {
  // Получаем текущий IP-адрес и SSID подключения
  String ipAddress = WiFi.localIP().toString();
  String ssid = WiFi.SSID();

  // Строка для справки с режимом работы каждого реле
  String helpText = "";
  // Добавляем информацию о состоянии каждого реле
  helpText += "Текущий статус:\n";
  int relayIndex = 1;  // Индекс для команд /on и /off
  for (const auto& relay : control.relays) {
    String commandOn = "/on" + String(relayIndex);
    String commandOff = "/off" + String(relayIndex);
    String mode_ = relay.manualMode ? "Manual" : "Auto";
    String state_ = digitalRead(relay.pin) ? "On" : "Off";
    helpText += String(commandOn + " " + commandOff + " - " + relay.description + " (" + mode_ + " / " + state_ + ")\n\n");
    relayIndex++;
  }

  helpText += "Дата и время на устройстве: " + formatDateTime(getCurrentTimeFromRTC()) + "\n";
  helpText += "Температура: " + String(currentTemp) + " °С\n";
  helpText += "\n";

  String tmpMsg = convertDateFormat(control.scenario.startDate) + " " + control.scenario.startTime + " по " + convertDateFormat(control.scenario.endDate) + " " + control.scenario.endTime + "\n";
  if (control.scenario.useSetting) {
    helpText += "Включено. Установленное расписание: " + tmpMsg;
  } else {
    helpText += "Отключено. Установленное расписание: " + tmpMsg;
  }

  helpText += "Установить расписание /setTime\n";
  helpText += "Включить /startTime\n";
  helpText += "Отменить /stopTime\n";

  helpText += "\n";

  helpText += "/resetManual - Сбросить в Auto\n";
  helpText += "/status - Статус и управление \n";
  helpText += "/pushOn /pushOff - Включить/Отключить уведомления\n";
  helpText += "/help - Справка \n\n";

  // Добавляем IP-адрес и SSID
  helpText += "Доступ из сети: http://" + ipAddress + "\n";
  helpText += "Имя сети Wi-Fi: " + ssid + "\n\n";

  return helpText;
}

//===============================

String sendHelp() {
  String helpMessage = "Доступные команды:\n";
  helpMessage += "Задать интервал переключения (в сек. ) /setTimeInterval 1180\n";

  return helpMessage;
}
