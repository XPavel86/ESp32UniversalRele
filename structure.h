

#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <vector>

bool isSaveControl = false;

struct Relay {
  uint8_t pin;
  String modePin;  // INPUT, OUTPUT, INPUT_PULLUP или INPUT_PULLDOWN
  bool manualMode;
  bool statePin;
  bool lastState;
  bool pwmMode;
  uint16_t pwm;
  uint16_t lastPwm;
  String description;
};

struct OutPower {
  bool isUseSetting; 
  uint8_t relayIndex;
  uint16_t pwm; 
  bool statePin;
  bool pwmMode;
  String description;
};

struct ScheduleScenario {
  bool isUseSetting;           // Использовать настройку
  bool collectionSettings[12]; // действия во время выполнения сценария: pwmOut, timers, PIDTemperature, easyAjustTemperature, PowerGraph, TemperatureGraph, pinsOut...
  String startDate;          // Дата и время включения
  String startTime;          // Время включения
  String endDate;            // Дата и время выключения
  String endTime;            // Время выключения
  bool week[7];
  bool months[12];
  OutPower initialStateRelay; // начальное состояния реле перед запуском сценария
  OutPower endlStateRelay; //  конечное состояние реле после выполнения сценария
};

struct Temperature {
  bool isUseSetting; 
  uint8_t relayIndex;
  int setTemperature;
  bool isSmoothly;
  bool isIncrease;
};

struct Timer {
  bool isUseSetting; 
  String time; 
  bool collectionSettings[12];
  OutPower initialStateRelay; // начальное состояния реле перед запуском сценария
  OutPower endlStateRelay; 
};

struct CalibrationMap {
  bool isUseSetting;
  uint8_t forRelayIndex;
  String name;
  float currentValues[256];
  float upperLimitValues[256];
  float lowerLimitValues[256];
  float upperLimitValue;
  float lowerLimitValue;
  bool isUsedUpperLimit;
  bool isUsedLowerLimit;
  bool isTurnOffPower;
  bool isSay;
};

struct SmoothStart {
    bool isUseSetting;
    uint8_t forRelayIndex;
    std::vector<std::vector<unsigned long>> timePower;  // Двумерный вектор
};

struct PowerGraph {
    bool isUseSetting;
    uint8_t forRelayIndex;
    std::vector<std::vector<unsigned long>> timePower;  // Двумерный вектор
};

struct TemperatureGraph {
    bool isUseSetting;
    uint8_t forRelayIndex;
    std::vector<std::vector<unsigned long>> timeTemp;  // Двумерный вектор
    bool isHeating;
};

struct Device {
  String nameDevice;
  bool isSelected;
  std::vector<Relay> relays;
  std::vector<uint8_t> pins = { 23, 22, 1, 3, 21, 19, 18, 5, 17, 16, 33, 32 };  // only input - 35, 34, 39, 36
  ScheduleScenario scheduleScenario; 
  Temperature temperature;
  OutPower outPower;
  std::vector<Timer> timer;
  bool isTimersEnabled;
  bool isEncyclateTimers;
  CalibrationMap calibrationMap;
  PowerGraph powerGraph;
  TemperatureGraph temperatureGraph;
  SmoothStart smoothStart;
};

std::vector<Device> devices;
uint8_t currentDeviceIndex = 0;

// Функция для инициализации устройства
void initializeDevice(String name, bool activ) {
    devices.emplace_back();
    Device& newDevice = devices.back();

    newDevice.nameDevice = name;
    newDevice.isSelected = activ;

    newDevice.relays = {
        {5, "OUTPUT", false, false, false, false, 0, 0, "Реле_1"}
    };

    newDevice.scheduleScenario = {
        false,
        {true, true, true, true, true, true, true, true, true, true, true, true},
        "2024-12-29",
        "08:00",
        "2024-12-29",
        "10:00",
        {true, true, true, true, true, true, true},
        {true, true, true, true, true, true, true, true, true, true, true, true},
        {false, 0, 0, false, false, "Initial State"},
        {false, 0, 0, false, false, "End State"}
    };

    newDevice.temperature = {false, 0, 22, true, true};
    newDevice.outPower = {false, 0, 0, false, false, "Output Power"};

    newDevice.timer.push_back({
        false, "00:30:00", {true, false, false, false, false, false, false, false, false, false, false, false},
        {false, 0, 0, false, false, "Initial Timer Power"},
        {false, 0, 0, false, false, "End Timer Power"}
    });

    newDevice.isTimersEnabled = false;
    newDevice.isEncyclateTimers = false;

    newDevice.calibrationMap = {
        false,
        0,
        "Calibration_1",
        {0}, {0}, {0},
        100.0f, 0.0f,
        true, true,
        true, true
    };

    SmoothStart smoothStart = {false, 0, {{0}}};
    PowerGraph powerGraph = {false, 0, {{0}}};
    TemperatureGraph temperatureGraph = {false, 0, {{0}}, false};

    Serial.println("Памяти в куче: ");
    Serial.print(esp_get_free_heap_size());
}

// Функция для сериализации устройства в JSON
String serializeDevice(const Device& device) {
    StaticJsonDocument<2048> doc;

    doc["nameDevice"] = device.nameDevice;
    doc["isSelected"] = device.isSelected;

    JsonArray relays = doc.createNestedArray("relays");
    for (const auto& relay : device.relays) {
        JsonObject relayObj = relays.createNestedObject();
        relayObj["pin"] = relay.pin;
        relayObj["modePin"] = relay.modePin;
        relayObj["manualMode"] = relay.manualMode;
        relayObj["statePin"] = relay.statePin;
        relayObj["lastState"] = relay.lastState;
        relayObj["pwmMode"] = relay.pwmMode;
        relayObj["pwm"] = relay.pwm;
        relayObj["lastPwm"] = relay.lastPwm;
        relayObj["description"] = relay.description;
    }

    JsonArray pins = doc.createNestedArray("pins");
    for (const auto& pin : device.pins) {
        pins.add(pin);
    }

    JsonObject scheduleScenario = doc.createNestedObject("scheduleScenario");
    scheduleScenario["isUseSetting"] = device.scheduleScenario.isUseSetting;
    JsonArray collectionSettings = scheduleScenario.createNestedArray("collectionSettings");
    for (const auto& setting : device.scheduleScenario.collectionSettings) {
        collectionSettings.add(setting);
    }
    scheduleScenario["startDate"] = device.scheduleScenario.startDate;
    scheduleScenario["startTime"] = device.scheduleScenario.startTime;
    scheduleScenario["endDate"] = device.scheduleScenario.endDate;
    scheduleScenario["endTime"] = device.scheduleScenario.endTime;
    JsonArray week = scheduleScenario.createNestedArray("week");
    for (const auto& day : device.scheduleScenario.week) {
        week.add(day);
    }
    JsonArray months = scheduleScenario.createNestedArray("months");
    for (const auto& month : device.scheduleScenario.months) {
        months.add(month);
    }
    JsonObject initialStateRelay = scheduleScenario.createNestedObject("initialStateRelay");
    initialStateRelay["isUseSetting"] = device.scheduleScenario.initialStateRelay.isUseSetting;
    initialStateRelay["relayIndex"] = device.scheduleScenario.initialStateRelay.relayIndex;
    initialStateRelay["pwm"] = device.scheduleScenario.initialStateRelay.pwm;
    initialStateRelay["statePin"] = device.scheduleScenario.initialStateRelay.statePin;
    initialStateRelay["pwmMode"] = device.scheduleScenario.initialStateRelay.pwmMode;
    initialStateRelay["description"] = device.scheduleScenario.initialStateRelay.description;
    JsonObject endlStateRelay = scheduleScenario.createNestedObject("endlStateRelay");
    endlStateRelay["isUseSetting"] = device.scheduleScenario.endlStateRelay.isUseSetting;
    endlStateRelay["relayIndex"] = device.scheduleScenario.endlStateRelay.relayIndex;
    endlStateRelay["pwm"] = device.scheduleScenario.endlStateRelay.pwm;
    endlStateRelay["statePin"] = device.scheduleScenario.endlStateRelay.statePin;
    endlStateRelay["pwmMode"] = device.scheduleScenario.endlStateRelay.pwmMode;
    endlStateRelay["description"] = device.scheduleScenario.endlStateRelay.description;

    JsonObject temperature = doc.createNestedObject("temperature");
    temperature["isUseSetting"] = device.temperature.isUseSetting;
    temperature["relayIndex"] = device.temperature.relayIndex;
    temperature["setTemperature"] = device.temperature.setTemperature;
    temperature["isSmoothly"] = device.temperature.isSmoothly;
    temperature["isIncrease"] = device.temperature.isIncrease;

    JsonObject outPower = doc.createNestedObject("outPower");
    outPower["isUseSetting"] = device.outPower.isUseSetting;
    outPower["relayIndex"] = device.outPower.relayIndex;
    outPower["pwm"] = device.outPower.pwm;
    outPower["statePin"] = device.outPower.statePin;
    outPower["pwmMode"] = device.outPower.pwmMode;
    outPower["description"] = device.outPower.description;

    JsonArray timers = doc.createNestedArray("timer");
    for (const auto& timer : device.timer) {
        JsonObject timerObj = timers.createNestedObject();
        timerObj["isUseSetting"] = timer.isUseSetting;
        timerObj["time"] = timer.time;
        JsonArray collectionSettings = timerObj.createNestedArray("collectionSettings");
        for (const auto& setting : timer.collectionSettings) {
            collectionSettings.add(setting);
        }
        JsonObject initialStateRelay = timerObj.createNestedObject("initialStateRelay");
        initialStateRelay["isUseSetting"] = timer.initialStateRelay.isUseSetting;
        initialStateRelay["relayIndex"] = timer.initialStateRelay.relayIndex;
        initialStateRelay["pwm"] = timer.initialStateRelay.pwm;
        initialStateRelay["statePin"] = timer.initialStateRelay.statePin;
        initialStateRelay["pwmMode"] = timer.initialStateRelay.pwmMode;
        initialStateRelay["description"] = timer.initialStateRelay.description;
        JsonObject endlStateRelay = timerObj.createNestedObject("endlStateRelay");
        endlStateRelay["isUseSetting"] = timer.endlStateRelay.isUseSetting;
        endlStateRelay["relayIndex"] = timer.endlStateRelay.relayIndex;
        endlStateRelay["pwm"] = timer.endlStateRelay.pwm;
        endlStateRelay["statePin"] = timer.endlStateRelay.statePin;
        endlStateRelay["pwmMode"] = timer.endlStateRelay.pwmMode;
        endlStateRelay["description"] = timer.endlStateRelay.description;
    }

    doc["isTimersEnabled"] = device.isTimersEnabled;
    doc["isEncyclateTimers"] = device.isEncyclateTimers;

    JsonObject calibrationMap = doc.createNestedObject("calibrationMap");
    calibrationMap["isUseSetting"] = device.calibrationMap.isUseSetting;
    calibrationMap["forRelayIndex"] = device.calibrationMap.forRelayIndex;
    calibrationMap["name"] = device.calibrationMap.name;
    JsonArray currentValues = calibrationMap.createNestedArray("currentValues");
    for (const auto& value : device.calibrationMap.currentValues) {
        currentValues.add(value);
    }
    JsonArray upperLimitValues = calibrationMap.createNestedArray("upperLimitValues");
    for (const auto& value : device.calibrationMap.upperLimitValues) {
        upperLimitValues.add(value);
    }
    JsonArray lowerLimitValues = calibrationMap.createNestedArray("lowerLimitValues");
    for (const auto& value : device.calibrationMap.lowerLimitValues) {
        lowerLimitValues.add(value);
    }
    calibrationMap["upperLimitValue"] = device.calibrationMap.upperLimitValue;
    calibrationMap["lowerLimitValue"] = device.calibrationMap.lowerLimitValue;
    calibrationMap["isUsedUpperLimit"] = device.calibrationMap.isUsedUpperLimit;
    calibrationMap["isUsedLowerLimit"] = device.calibrationMap.isUsedLowerLimit;
    calibrationMap["isTurnOffPower"] = device.calibrationMap.isTurnOffPower;
    calibrationMap["isSay"] = device.calibrationMap.isSay;

    JsonObject powerGraph = doc.createNestedObject("powerGraph");
    powerGraph["isUseSetting"] = device.powerGraph.isUseSetting;
    powerGraph["forRelayIndex"] = device.powerGraph.forRelayIndex;
    JsonArray timePower = powerGraph.createNestedArray("timePower");
    for (const auto& vec : device.powerGraph.timePower) {
        JsonArray innerArray = timePower.createNestedArray();
        for (const auto& value : vec) {
            innerArray.add(value);
        }
    }

    JsonObject temperatureGraph = doc.createNestedObject("temperatureGraph");
    temperatureGraph["isUseSetting"] = device.temperatureGraph.isUseSetting;
    temperatureGraph["forRelayIndex"] = device.temperatureGraph.forRelayIndex;
    JsonArray timeTemp = temperatureGraph.createNestedArray("timeTemp");
    for (const auto& vec : device.temperatureGraph.timeTemp) {
        JsonArray innerArray = timeTemp.createNestedArray();
        for (const auto& value : vec) {
            innerArray.add(value);
        }
    }
    temperatureGraph["isHeating"] = device.temperatureGraph.isHeating;

    JsonObject smoothStart = doc.createNestedObject("smoothStart");
    smoothStart["isUseSetting"] = device.smoothStart.isUseSetting;
    smoothStart["forRelayIndex"] = device.smoothStart.forRelayIndex;
    JsonArray smoothTimePower = smoothStart.createNestedArray("smoothTimePower");
    for (const auto& vec : device.smoothStart.timePower) {
        JsonArray innerArray = smoothTimePower.createNestedArray();
        for (const auto& value : vec) {
            innerArray.add(value);
        }
    }

    String output;
    serializeJson(doc, output);
    return output;
}

// Функция для десериализации устройства из JSON
bool deserializeDevice(const String& json, Device& device) {
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, json);
    if (error) {
        Serial.print("Ошибка десериализации: ");
        Serial.println(error.c_str());
        return false;
    }

    device.nameDevice = doc["nameDevice"].as<String>();
    device.isSelected = doc["isSelected"].as<bool>();

    JsonArray relays = doc["relays"];
    for (JsonObject relayObj : relays) {
        Relay relay;
        relay.pin = relayObj["pin"];
        relay.modePin = relayObj["modePin"].as<String>();
        relay.manualMode = relayObj["manualMode"];
        relay.statePin = relayObj["statePin"];
        relay.lastState = relayObj["lastState"];
        relay.pwmMode = relayObj["pwmMode"];
        relay.pwm = relayObj["pwm"];
        relay.lastPwm = relayObj["lastPwm"];
        relay.description = relayObj["description"].as<String>();
        device.relays.push_back(relay);
    }

    JsonArray pins = doc["pins"];
    for (JsonVariant pin : pins) {
        device.pins.push_back(pin.as<uint8_t>());
    }

    JsonObject scheduleScenario = doc["scheduleScenario"];
    device.scheduleScenario.isUseSetting = scheduleScenario["isUseSetting"];
    JsonArray collectionSettings = scheduleScenario["collectionSettings"];
    for (int i = 0; i < 12; i++) {
        device.scheduleScenario.collectionSettings[i] = collectionSettings[i];
    }
    device.scheduleScenario.startDate = scheduleScenario["startDate"].as<String>();
    device.scheduleScenario.startTime = scheduleScenario["startTime"].as<String>();
    device.scheduleScenario.endDate = scheduleScenario["endDate"].as<String>();
    device.scheduleScenario.endTime = scheduleScenario["endTime"].as<String>();
    JsonArray week = scheduleScenario["week"];
    for (int i = 0; i < 7; i++) {
        device.scheduleScenario.week[i] = week[i];
    }
    JsonArray months = scheduleScenario["months"];
    for (int i = 0; i < 12; i++) {
        device.scheduleScenario.months[i] = months[i];
    }
    JsonObject initialStateRelay = scheduleScenario["initialStateRelay"];
    device.scheduleScenario.initialStateRelay.isUseSetting = initialStateRelay["isUseSetting"];
    device.scheduleScenario.initialStateRelay.relayIndex = initialStateRelay["relayIndex"];
    device.scheduleScenario.initialStateRelay.pwm = initialStateRelay["pwm"];
    device.scheduleScenario.initialStateRelay.statePin = initialStateRelay["statePin"];
    device.scheduleScenario.initialStateRelay.pwmMode = initialStateRelay["pwmMode"];
    device.scheduleScenario.initialStateRelay.description = initialStateRelay["description"].as<String>();
    JsonObject endlStateRelay = scheduleScenario["endlStateRelay"];
    device.scheduleScenario.endlStateRelay.isUseSetting = endlStateRelay["isUseSetting"];
    device.scheduleScenario.endlStateRelay.relayIndex = endlStateRelay["relayIndex"];
    device.scheduleScenario.endlStateRelay.pwm = endlStateRelay["pwm"];
    device.scheduleScenario.endlStateRelay.statePin = endlStateRelay["statePin"];
    device.scheduleScenario.endlStateRelay.pwmMode = endlStateRelay["pwmMode"];
    device.scheduleScenario.endlStateRelay.description = endlStateRelay["description"].as<String>();

    JsonObject temperature = doc["temperature"];
    device.temperature.isUseSetting = temperature["isUseSetting"];
    device.temperature.relayIndex = temperature["relayIndex"];
    device.temperature.setTemperature = temperature["setTemperature"];
    device.temperature.isSmoothly = temperature["isSmoothly"];
    device.temperature.isIncrease = temperature["isIncrease"];

    JsonObject outPower = doc["outPower"];
    device.outPower.isUseSetting = outPower["isUseSetting"];
    device.outPower.relayIndex = outPower["relayIndex"];
    device.outPower.pwm = outPower["pwm"];
    device.outPower.statePin = outPower["statePin"];
    device.outPower.pwmMode = outPower["pwmMode"];
    device.outPower.description = outPower["description"].as<String>();

    JsonArray timers = doc["timer"];
    for (JsonObject timerObj : timers) {
        Timer timer;
        timer.isUseSetting = timerObj["isUseSetting"];
        timer.time = timerObj["time"].as<String>();
        JsonArray collectionSettings = timerObj["collectionSettings"];
        for (int i = 0; i < 12; i++) {
            timer.collectionSettings[i] = collectionSettings[i];
        }
        JsonObject initialStateRelay = timerObj["initialStateRelay"];
        timer.initialStateRelay.isUseSetting = initialStateRelay["isUseSetting"];
        timer.initialStateRelay.relayIndex = initialStateRelay["relayIndex"];
        timer.initialStateRelay.pwm = initialStateRelay["pwm"];
        timer.initialStateRelay.statePin = initialStateRelay["statePin"];
        timer.initialStateRelay.pwmMode = initialStateRelay["pwmMode"];
        timer.initialStateRelay.description = initialStateRelay["description"].as<String>();
        JsonObject endlStateRelay = timerObj["endlStateRelay"];
        timer.endlStateRelay.isUseSetting = endlStateRelay["isUseSetting"];
        timer.endlStateRelay.relayIndex = endlStateRelay["relayIndex"];
        timer.endlStateRelay.pwm = endlStateRelay["pwm"];
        timer.endlStateRelay.statePin = endlStateRelay["statePin"];
        timer.endlStateRelay.pwmMode = endlStateRelay["pwmMode"];
        timer.endlStateRelay.description = endlStateRelay["description"].as<String>();
        device.timer.push_back(timer);
    }

    device.isTimersEnabled = doc["isTimersEnabled"];
    device.isEncyclateTimers = doc["isEncyclateTimers"];

    JsonObject calibrationMap = doc["calibrationMap"];
    device.calibrationMap.isUseSetting = calibrationMap["isUseSetting"];
    device.calibrationMap.forRelayIndex = calibrationMap["forRelayIndex"];
    device.calibrationMap.name = calibrationMap["name"].as<String>();
    JsonArray currentValues = calibrationMap["currentValues"];
    for (int i = 0; i < 256; i++) {
        device.calibrationMap.currentValues[i] = currentValues[i];
    }
    JsonArray upperLimitValues = calibrationMap["upperLimitValues"];
    for (int i = 0; i < 256; i++) {
        device.calibrationMap.upperLimitValues[i] = upperLimitValues[i];
    }
    JsonArray lowerLimitValues = calibrationMap["lowerLimitValues"];
    for (int i = 0; i < 256; i++) {
        device.calibrationMap.lowerLimitValues[i] = lowerLimitValues[i];
    }
    device.calibrationMap.upperLimitValue = calibrationMap["upperLimitValue"];
    device.calibrationMap.lowerLimitValue = calibrationMap["lowerLimitValue"];
    device.calibrationMap.isUsedUpperLimit = calibrationMap["isUsedUpperLimit"];
    device.calibrationMap.isUsedLowerLimit = calibrationMap["isUsedLowerLimit"];
    device.calibrationMap.isTurnOffPower = calibrationMap["isTurnOffPower"];
    device.calibrationMap.isSay = calibrationMap["isSay"];

    JsonObject powerGraph = doc["powerGraph"];
    device.powerGraph.isUseSetting = powerGraph["isUseSetting"];
    device.powerGraph.forRelayIndex = powerGraph["forRelayIndex"];
    JsonArray timePower = powerGraph["timePower"];
    for (JsonArray innerArray : timePower) {
        std::vector<unsigned long> vec;
        for (JsonVariant value : innerArray) {
            vec.push_back(value.as<unsigned long>());
        }
        device.powerGraph.timePower.push_back(vec);
    }

    JsonObject temperatureGraph = doc["temperatureGraph"];
    device.temperatureGraph.isUseSetting = temperatureGraph["isUseSetting"];
    device.temperatureGraph.forRelayIndex = temperatureGraph["forRelayIndex"];
    JsonArray timeTemp = temperatureGraph["timeTemp"];
    for (JsonArray innerArray : timeTemp) {
        std::vector<unsigned long> vec;
        for (JsonVariant value : innerArray) {
            vec.push_back(value.as<unsigned long>());
        }
        device.temperatureGraph.timeTemp.push_back(vec);
    }
    device.temperatureGraph.isHeating = temperatureGraph["isHeating"];

    JsonObject smoothStart = doc["smoothStart"];
    device.smoothStart.isUseSetting = smoothStart["isUseSetting"];
    device.smoothStart.forRelayIndex = smoothStart["forRelayIndex"];
    JsonArray smoothTimePower = smoothStart["smoothTimePower"];
    for (JsonArray innerArray : smoothTimePower) {
        std::vector<unsigned long> vec;
        for (JsonVariant value : innerArray) {
            vec.push_back(value.as<unsigned long>());
        }
        device.smoothStart.timePower.push_back(vec);
    }

    return true;
}

// Функция записи вектора устройств в файл
bool writeDevicesToFile(const std::vector<Device>& devices, const char* filename) {
    File file = SPIFFS.open(filename, "w");
    if (!file) {
        Serial.println("Ошибка открытия файла для записи");
        return false;
    }

    JsonArray jsonArray;
    for (const auto& device : devices) {
        String json = serializeDevice(device);
        file.println(json);
    }

    file.close();
    return true;
}

// Функция чтения вектора устройств из файла (исправлена)
bool readDevicesFromFile(std::vector<Device>& devices, const char* filename) {
    File file = SPIFFS.open(filename, "r");
    if (!file) {
        Serial.println("Ошибка открытия файла для чтения");
        return false;
    }

    devices.clear(); // Очищаем вектор устройств
    while (file.available()) {
        String json = file.readStringUntil('\n');
        Device device;
        if (deserializeDevice(json, device)) {
            devices.push_back(device);
        }
        yield(); // Даём системе время для обработки других задач
    }

    file.close();
    return true;
}



// Вывод информации об устройствах
// Вывод информации об устройствах и индекса выбранного устройства
void printDevices(const std::vector<Device>& devices) {
    if (devices.empty()) {
        Serial.println("Устройства не найдены.");
        return;
    }

    bool foundSelected = false; // Флаг для отслеживания найденного выбранного устройства

    for (size_t i = 0; i < devices.size(); ++i) {
        const auto& device = devices[i];
        Serial.println("Устройство #" + String(i));
        Serial.println("  Имя: " + device.nameDevice);
        Serial.println("  Выбрано: " + String(device.isSelected ? "Да" : "Нет"));
        Serial.println("  Количество реле: " + String(device.relays.size()));
        Serial.println("  Количество таймеров: " + String(device.timer.size()));
        Serial.println("-----------------------------");

        // Проверяем, выбрано ли устройство
        if (device.isSelected) {
            Serial.println("Выбранное устройство имеет индекс: " + String(i));
            foundSelected = true;
        }
    }

    // Если ни одно устройство не выбрано
    if (!foundSelected) {
        Serial.println("Нет выбранного устройства.");
    }
}

int getSelectedDeviceIndex(const std::vector<Device>& devices) {
    for (size_t i = 0; i < devices.size(); ++i) {
        if (devices[i].isSelected) {
            return i; // Возвращаем индекс первого найденного выбранного устройства
        }
    }
    return -1; // Если ни одно устройство не выбрано
}

void deviceInit() {
// Инициализация устройства, если файл не существует
    if (!SPIFFS.exists("/device.json")) {
        initializeDevice("MyDevice1", true);

        writeDevicesToFile(devices, "/device.json");
        Serial.println("Устройство инициализировано и сохранено в файл.");
    } else {

    // Чтение устройств из файла
    if (readDevicesFromFile(devices, "/device.json")) {
        Serial.println("Устройства успешно загружены из файла:");
       // printDevices(devices); // Вывод информации об устройствах
       currentDeviceIndex = getSelectedDeviceIndex(devices);
    } else {
        Serial.println("Ошибка загрузки устройств из файла.");
    } 
}
    }
