#include <Arduino.h>
#include <TroykaCurrent.h>
#include <ESPAsyncWebServer.h>
//#include <HCSR04.h>
#include <WiFi.h>
#include <time.h>

#include <ctype.h>

#include <AsyncDelay.h>
//#include <jsnsr04t.h>

ESP32Time rtc;

// датчик температуры //
const int tempPin = 32;

#define B 3950              // B-коэффициент
#define SERIAL_R 10000      // сопротивление последовательного резистора, 10 кОм
#define THERMISTOR_R 22000  // номинальное сопротивления термистора, 10 кОм
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

    String day = date.substring(0, dot1);   // День
    String month = date.substring(dot1 + 1, dot2);  // Месяц
    String year = date.substring(dot2 + 1); // Год

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

bool startTimer1 = false;
int timer1 = 0;

// Состояние реле
bool relay1State = false;

void toggleRelays() {

  int relay1Pin = control.scenario.pinRelays;
  int relay2Pin = control.scenario.pinRelays2;

  if (relay1State) {
    delay(100);
    // Если реле1 включено, выключаем его и включаем реле2
    digitalWrite(relay1Pin, LOW);
    digitalWrite(relay2Pin, HIGH);
    relay1State = false;
    Serial.println("Relay1 OFF, Relay2 ON");
    
  } else {
    delay(100);
    // Если реле2 включено, выключаем его и включаем реле1
    digitalWrite(relay2Pin, LOW);
    digitalWrite(relay1Pin, HIGH);
    relay1State = true;
    Serial.println("Relay2 OFF, Relay1 ON");
  }
}

// struct Relay {
//     int pin;
//     bool state; // Состояние реле: true - включено, false - выключено
// };

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


void scenarioRele(bool start) {
  int relayInterval = control.scenario.timeInterval;
  int relay1Pin = control.scenario.pinRelays;
  int relay2Pin = control.scenario.pinRelays2;

  bool manualMode1;
  bool manualMode2;

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
      startTimer1 = true;

      if (timer1 >= relayInterval) {
        delay(0);
        toggleRelays();
        timer1 = 0;
      }
    } else {
      startTimer1 = false;
      digitalWrite(relay1Pin, LOW);
      digitalWrite(relay2Pin, LOW);
    }
  }
}

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

time_t convertToTimeT(const String& date, const String& time) {
  tm t = {};
  sscanf(date.c_str(), "%4d-%2d-%2d", &t.tm_year, &t.tm_mon, &t.tm_mday);
  sscanf(time.c_str(), "%2d:%2d", &t.tm_hour, &t.tm_min);

  t.tm_year -= 1900;  // Годы с 1900
  t.tm_mon -= 1;      // Месяцы от 0 до 11

  return mktime(&t);
}

time_t getCurrentTimeFromRTC() {
  time_t now = rtc.getEpoch();  // Получаем текущее время из RTC
  return now;                   // Возвращаем как time_t
}

//=================================================

void mainScenario(void (*callFunc)(String)) {

  delay(200);

  if (Serial.available() > 0) {
    String arg = Serial.readString();
    manualWork(arg);
    if (debug) {
      debugInfo();
    }
  }

  // Проверяем, включено ли использование настройки
  if (control.scenario.useSetting) {

    // Преобразуем строки даты и времени в `time_t`
    time_t startDateTime = convertToTimeT(control.scenario.startDate, control.scenario.startTime);
    time_t endDateTime = convertToTimeT(control.scenario.endDate, control.scenario.endTime);
    time_t currentDateTime = getCurrentTimeFromRTC();  // Текущее время
    struct tm currentTime;
    localtime_r(&currentDateTime, &currentTime);  // Получаем текущий день недели

    // Проверяем, попадает ли текущая дата/время в заданный диапазон
    if (currentDateTime >= startDateTime && currentDateTime <= endDateTime) {

      // Проверяем, включен ли текущий день в массиве недели
      if (control.scenario.week[shiftWeekDay(currentTime.tm_wday)]) {

        struct tm* currentTime = localtime(&currentDateTime);  // Преобразуем в структуру tm
        int currentHour = currentTime->tm_hour;                // Часы
        int currentMinute = currentTime->tm_min;               // Минуты

        // Время начала и окончания
        int startHour = control.scenario.startTime.substring(0, 2).toInt();
        int startMinute = control.scenario.startTime.substring(3, 5).toInt();
        int endHour = control.scenario.endTime.substring(0, 2).toInt();
        int endMinute = control.scenario.endTime.substring(3, 5).toInt();

        // Преобразуем время начала и окончания в количество минут с начала дня
        int startMinutes = startHour * 60 + startMinute;
        int endMinutes = endHour * 60 + endMinute;
        int currentMinutes = currentHour * 60 + currentMinute;

        // Сравниваем время в минутах
        if (currentMinutes >= startMinutes && currentMinutes <= endMinutes) {

          // Если установлена галочка температуры
          if (control.scenario.temperatureCheckbox) {
            // Если текущая температура меньше заданной, включаем реле
            if (currentTemp < control.scenario.temperature) {
              //scenarioRele(true, control.scenario.pinRelays , control.scenario.pinRelays2, control.scenario.timeInterval );
              scenarioRele(true);
            } else {
              //scenarioRele(false, control.scenario.pinRelays , control.scenario.pinRelays2, control.scenario.timeInterval );
              scenarioRele(false);
            }
          } else {
            //scenarioRele(true, control.scenario.pinRelays , control.scenario.pinRelays2, control.scenario.timeInterval );
            scenarioRele(true);
          }
        } else {
          // scenarioRele(false, control.scenario.pinRelays , control.scenario.pinRelays2, control.scenario.timeInterval );
          scenarioRele(false);
        }
      } else {
        // scenarioRele(false, control.scenario.pinRelays , control.scenario.pinRelays2, control.scenario.timeInterval ); // Если текущий день недели не активен
        scenarioRele(false);
      }
    } else {
      //scenarioRele(false, control.scenario.pinRelays , control.scenario.pinRelays2, control.scenario.timeInterval ); // Если текущее время не в пределах начала и конца даты
      scenarioRele(false);
    }
  } else {
    // Если настройка отключена
    //scenarioRele(false, control.scenario.pinRelays , control.scenario.pinRelays2, control.scenario.timeInterval );
    scenarioRele(false);
  }

  printRelayStates(callFunc);  // передача состояний
}

////====================================================

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

  helpText += "Температура: " + String(currentTemp) + " °С\n";
  helpText += "\n";

  String tmpMsg = convertDateFormat(control.scenario.startDate) + " " + control.scenario.startTime + " по " + convertDateFormat(control.scenario.endDate) + " " + control.scenario.endTime + "\n";
  if (control.scenario.useSetting) {
    helpText += "Включено. Установленное расписание " + tmpMsg;
  } else {
    helpText += "Отключено. Установленное расписание " + tmpMsg;
  }

  helpText += "Установить расписание /setTime\n";
  helpText += "Включить /startTime\n";
  helpText += "Отменить /stopTime\n";

  helpText += "\n";


  helpText += "/resetManual - Сбросить в Auto\n";
  helpText += "/status - Статус и управление \n";
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
