#include <Arduino.h>
#include <TroykaCurrent.h>
#include <ESPAsyncWebServer.h>
//#include <HCSR04.h>
#include <WiFi.h>
#include <time.h>

#include <AsyncDelay.h>
//#include <jsnsr04t.h>


ESP32Time rtc;

//====================================
// датчик температуры //

const int tempPin = 32;   

#define B 3950              // B-коэффициент
#define SERIAL_R 10000      // сопротивление последовательного резистора, 10 кОм
#define THERMISTOR_R 22000  // номинальное сопротивления термистора, 10 кОм
#define NOMINAL_T 25        // номинальная температура (при которой TR = 10 кОм)  #define B 3950 // B-коэффициент
                            // номинальная температура (при которой TR = 10 кОм)
//==============================
bool isSaveControl = false;
//=============================

struct Scenario {
    bool useSetting;         // Использовать настройку
    int temperature;         // Температура
    bool temperatureCheckbox; // Включить температуру
    String startDate;        // Дата и время включения
    String startTime;        // Время включения
    String endDate;          // Дата и время выключения
    String endTime;          // Время выключения
    int pinRelays;           // Номер пина реле
    bool week[7];
};


struct Relay {
  int pin;
  String modePin; // INPUT, OUTPUT, INPUT_PULLUP или INPUT_PULLDOWN
  bool manualMode;
  bool statePin;
  bool lastState;
  String description;
};

struct Control {
  std::vector<Relay> relays;
  std::vector<int> pins = {23, 22, 1, 3, 21, 19, 18, 5, 17, 16, 33, 32 }; // only input - 35, 34, 39, 36
  Scenario scenario;
  int getRelayCount() const {
        return relays.size();
    }
};

Control control;

// Функция для инициализации реле в структуре управления
void initializeControl() {
  // Инициализация массива реле
  control.relays = {
    { 5, "OUTPUT", false, false, false, "Реле_1" },
    { 18, "OUTPUT", false, false, false, "Реле_2" },
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
       case INPUT: return "INPUT"; // 1
       case OUTPUT: return "OUTPUT";  // 3
       case INPUT_PULLUP: return "INPUT_PULLUP"; // 5
       case INPUT_PULLDOWN: return "INPUT_PULLDOWN"; // 9
       default: return "UNKNOWN"; // Для обработки некорректных значений
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
        return OUTPUT; // Некорректный режим, по умолчанию возвращаем 1
    }
}

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
    debugString += "Pin: " + String(relay.pin); // modePin
    debugString += "ModePin: " + String(relay.modePin); // modePin
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

//=====================================================//

void printRelayStates(void (*callFunc)(String)) {
  for (auto& relay : control.relays) {
    int statePin = digitalRead(relay.pin);  // Читаем текущее состояние реле

    // Проверяем, изменилось ли состояние
    if (statePin != relay.lastState) {
      String sOut = relay.description;
      sOut += ": ";
      sOut += statePin ? "включено" : "отключено"; // включено / отключено
      sOut += "\n";

      callFunc(sOut);

      relay.lastState = statePin;  // Обновляем предыдущее состояние
    }
  }
}

//=================[Главный сценарий]==================//


time_t convertToTimeT(const String& date, const String& time) {
    tm t = {};
    sscanf(date.c_str(), "%4d-%2d-%2d", &t.tm_year, &t.tm_mon, &t.tm_mday);
    sscanf(time.c_str(), "%2d:%2d", &t.tm_hour, &t.tm_min);

    t.tm_year -= 1900; // Годы с 1900
    t.tm_mon -= 1;     // Месяцы от 0 до 11

    return mktime(&t);
}

bool lastStatePin = LOW;  // Переменная для хранения последнего состояния реле

void controlRelay(bool state) {
    int relayPin = control.scenario.pinRelays;

    // Проверяем, нужно ли менять состояние реле
    if (state != lastStatePin) {
        if (state) {
            digitalWrite(relayPin, HIGH);  // Включаем реле
        } else {
            digitalWrite(relayPin, LOW);   // Выключаем реле
        }

        lastStatePin = state;  // Обновляем последнее состояние
    } 
}

time_t getCurrentTimeFromRTC() {
    time_t now = rtc.getEpoch(); // Получаем текущее время из RTC
    return now; // Возвращаем как time_t
}


//=================================================

 void mainScenario(void (*callFunc)(String)) {  

  if (Serial.available() > 0) {
    String arg = Serial.readString();
    manualWork(arg);
    if (debug) {
      debugInfo();
    }
  }

// Проверяем, включено ли использование настройки
if (control.scenario.useSetting) {
    // Преобразование строк даты и времени в `time_t`
    time_t startDateTime = convertToTimeT(control.scenario.startDate, control.scenario.startTime);
    time_t endDateTime = convertToTimeT(control.scenario.endDate, control.scenario.endTime);
    time_t currentDateTime = getCurrentTimeFromRTC(); // Текущее время

    // Проверяем, попадает ли текущая дата/время в заданный диапазон
    if (currentDateTime >= startDateTime && currentDateTime <= endDateTime) {
       // Serial.println("Current time is within the scenario's range.");

        // Если установлена галочка температуры
        if (control.scenario.temperatureCheckbox){
            // Если текущая температура меньше заданной, включаем реле
            if (currentTemp < control.scenario.temperature) {
                controlRelay(true);
            } else {
                controlRelay(false);
            }
        } else {
            controlRelay(true);
        }
    } else {
        controlRelay(false);
    }
} else {
    // Если настройка отключена
    controlRelay(false);
}
//      String say = "Сценарий трубы остановлен - этап #5\n";
//      callFunc(say);

    printRelayStates(callFunc); // запускать с задержкой 

  }

  ////=======================================================

   void controlTask() {

        delay(10);

      //Timer for Rele2 тайемер если насос давления работает более 30 минут +
//      if (startTimerRele2) {
//        TimerRele2++;
//      } else {
//        TimerRele2 = 0;
//      }

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
    //helpMessage += "/on1 /off1 - Скважинный насос\n";
   
    return helpMessage;
  }
