#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <HTTPClient.h>
#include "ESP32Time.h"
#include <time.h>
#include <ArduinoJson.h>
#include <UniversalTelegramBot.h>
#include <SPIFFS.h>
#include <esp_task_wdt.h>
#include <vector>
#include <Ticker.h>

#include "func.h"
#include "Control.h"
//================================
Ticker ticker;
//=================================

//=================================
bool isConnectionAttempts = true;
int totalNetworks = 0;

//================================
String scannedNetworks = "[]";
//==================================
String oldBotId = "";
//================================
uint64_t tiktak1 = 0;
uint64_t tiktak2 = 0;
uint64_t tiktak3 = 0;

//==================================
bool res = false;
//bool isUpdate = false;
bool isNewToken = false;
bool isScan = false;
//===================================
const unsigned long BOT_MTBS = 1000;
unsigned long bot_lasttime;

bool isBotToken = false;
String CHAT_ID;
//===================================
int idSetting = 0;
//===================================

struct TelegramUserID {
  String id;
  bool reading;
  bool writing;
};

struct TelegramSettings {
  bool isTelegramOn;
  String botId;
  std::vector<TelegramUserID> telegramUsers;  // Использование vector для динамического массива
  String lastMessage;
};

struct NetworkSetting {
  String ssid;
  String password;
  bool useStaticIP;
  IPAddress staticIP;
  IPAddress staticGateway;
  IPAddress staticSubnet;
  IPAddress staticDNS;
};

struct WiFiSettings {
  bool isWifiTurnedOn;
  int currentIdNetworkSetting;
  std::vector<NetworkSetting> networkSettings;  // Использование vector для динамического массива
  String ssidAP;
  String passwordAP;
  IPAddress ipAddressAP;
  bool isAP;
  TelegramSettings telegramSettings;
};

WiFiSettings settings;

//===================================

WiFiClientSecure client;
UniversalTelegramBot* bot;

//====================================

const char* defaultSsidAP = "KolibriAP";
const char* defaultPasswordAP = "";
IPAddress local_IP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

unsigned long startTime;
const unsigned long CONNECTION_TIMEOUT = 5000;  // Время ожидания подключения

bool isConnectedWiFi = false;
bool isStartedAP = false;
bool isCertificate = true;
bool isInternetConnected = false;

AsyncWebServer server(80);


//===================================
struct DateTime {
  int setHour;
  int setMinute;
  int setDay;
  int setMonth;
  int setYear;

  String getNowDateTime() {
    return String(setDay) + "." + String(setMonth) + "." + String(setYear) + " " + String(setHour) + ":" + String(setMinute);
  }
};

DateTime dateTime;

ESP32Time rtc;
//==============================
const size_t MAX_LOG_MESSAGES = 100;

struct LogList {
  String message;
  String timestamp;
  bool isSay;
};

LogList logList[MAX_LOG_MESSAGES];
size_t currentIndex = 0;
size_t logCount = 0;
//=============================
// Функция для получения текущего времени из RTC в формате строки
String getFormattedTimeFromRTC() {
  time_t now = rtc.getEpoch();
  struct tm* timeinfo = localtime(&now);
  char timeString[20];
  strftime(timeString, sizeof(timeString), "%d.%m.%Y %H:%M:%S", timeinfo);
  return String(timeString);
}

// Функция для добавления сообщения в лог с датой и временем
void addLog(String message) {
  String timestamp = getFormattedTimeFromRTC();

  logList[currentIndex] = { message, timestamp, false };
  currentIndex = (currentIndex + 1) % MAX_LOG_MESSAGES;
  if (logCount < MAX_LOG_MESSAGES) {
    logCount++;
  }
}

// Функция для получения всего списка сообщений как одной строки с разделителем \n
String getLog() {
  String result;
  size_t index = currentIndex;  // Начинаем с текущего индекса

  if (logCount < MAX_LOG_MESSAGES) {
    index = 0;
  }

  for (size_t i = 0; i < logCount; ++i) {
    // Формируем строку лог-сообщения с временной меткой и сообщением
    String logMessage = "[" + logList[(index + i) % MAX_LOG_MESSAGES].timestamp + "] - " + logList[(index + i) % MAX_LOG_MESSAGES].message;
    result += logMessage;

    if (i < logCount - 1) {
      result += "\n";  // Добавляем перенос строки только между сообщениями
    }
  }
  return result;
}

//=============================
void sendPendingMessages() {

  int messageSent;
  for (size_t index = 0; index < logCount; ++index) {

    if (!logList[index].isSay && logList[index].message.length() > 0) {
      //String sendStr = "[" + logList[index].timestamp + "] - " + logList[index].message;
      String sendStr = logList[index].message;
      for (const auto& user : settings.telegramSettings.telegramUsers) {
        if (!user.id.isEmpty() && user.reading && settings.telegramSettings.isTelegramOn) {
          messageSent = bot->sendMessage(user.id, sendStr, "");
        }
      }

      if (messageSent == 1) {
        logList[index].isSay = true;
      } else {
        //Serial.println("Failed to send message");
        break;
      }
    }
  }
}
//=============================

void addTelegramUser(const String& id, bool reading, bool writing) {
  TelegramUserID user = { id, reading, writing };
  settings.telegramSettings.telegramUsers.push_back(user);
}

void removeTelegramUser(const String& id) {
  auto& users = settings.telegramSettings.telegramUsers;
  users.erase(std::remove_if(users.begin(), users.end(),
                             [&id](const TelegramUserID& user) {
                               return user.id == id;
                             }),
              users.end());
}

void addNetwork(const String& ssid, const String& password, bool useStaticIP,
                const IPAddress& staticIP, const IPAddress& staticGateway,
                const IPAddress& staticSubnet, const IPAddress& staticDNS) {
  NetworkSetting network = { ssid, password, useStaticIP, staticIP, staticGateway, staticSubnet, staticDNS };
  settings.networkSettings.push_back(network);
}

void removeNetwork(const String& ssid) {
  auto& networks = settings.networkSettings;
  networks.erase(std::remove_if(networks.begin(), networks.end(),
                                [&ssid](const NetworkSetting& network) {
                                  return network.ssid == ssid;
                                }),
                 networks.end());
}

//===============================
String getSettingsJson() {
  DynamicJsonDocument doc(4096);  // Увеличиваем размер, чтобы вместить все данные

  JsonObject root = doc.to<JsonObject>();

  root["isWifiTurnedOn"] = settings.isWifiTurnedOn;
  root["currentIdNetworkSetting"] = settings.currentIdNetworkSetting;
  root["ssidAP"] = settings.ssidAP.isEmpty() ? "KalibriAP" : settings.ssidAP;
  root["passwordAP"] = settings.passwordAP;
  root["ipAddressAP"] = settings.ipAddressAP.toString();
  root["isAP"] = settings.isAP;
  root["connected"] = isConnectedWiFi;

  // Добавляем информацию о Telegram настройках
  JsonObject telegramSettingsObj = root.createNestedObject("telegramSettings");
  telegramSettingsObj["isTelegramOn"] = settings.telegramSettings.isTelegramOn;
  telegramSettingsObj["botId"] = settings.telegramSettings.botId;
  telegramSettingsObj["lastMessage"] = settings.telegramSettings.lastMessage;

  // Добавляем информацию о пользователях Telegram
  JsonArray usersArray = telegramSettingsObj.createNestedArray("telegramUsers");
  for (const auto& user : settings.telegramSettings.telegramUsers) {
    JsonObject userObj = usersArray.createNestedObject();
    userObj["id"] = user.id;
    userObj["reading"] = user.reading;
    userObj["writing"] = user.writing;
  }

  // Добавляем информацию о сетевых настройках
  JsonArray networkSettingsArray = root.createNestedArray("networkSettings");
  for (const auto& network : settings.networkSettings) {
    JsonObject networkSettingObj = networkSettingsArray.createNestedObject();
    networkSettingObj["ssid"] = network.ssid;
    networkSettingObj["password"] = network.password;
    networkSettingObj["useStaticIP"] = network.useStaticIP;
    networkSettingObj["staticIP"] = network.staticIP.toString();
    networkSettingObj["staticGateway"] = network.staticGateway.toString();
    networkSettingObj["staticSubnet"] = network.staticSubnet.toString();
    networkSettingObj["staticDNS"] = network.staticDNS.toString();
  }

  String output;
  serializeJson(doc, output);
  return output;
}

//============================

void initializeWiFiSettings(WiFiSettings& settings) {
  settings.isWifiTurnedOn = true;
  settings.currentIdNetworkSetting = 0;  // Устанавливаем текущий ID сетевой настройки
  settings.ssidAP = "KolibriAP";
  settings.passwordAP = "";
  settings.ipAddressAP = IPAddress(192, 168, 1, 1);
  settings.isAP = true;

    NetworkSetting network;
    network.ssid = "My home Wi-Fi";
    network.password = "12345678w";
    network.useStaticIP = false;
    network.staticIP = IPAddress(192, 168, 1, 101 );
    network.staticGateway = IPAddress(192, 168, 1, 1);
    network.staticSubnet = IPAddress(255, 255, 255, 0);
    network.staticDNS = IPAddress(192, 168, 1, 1);
    settings.networkSettings.push_back(network);
}

//===========================
bool saveSettings(bool wdt = false) {
  if (wdt) {
    // esp_task_wdt_add(NULL);
  };

  if (isUpdate) {
    return false;
  }

  DynamicJsonDocument doc(4096);

  // Создаем JSON-массив для хранения всех сетевых настроек
  JsonArray networkSettingsArray = doc.createNestedArray("networkSettings");

  doc["isWifiTurnedOn"] = settings.isWifiTurnedOn;
  doc["currentIdNetworkSetting"] = settings.currentIdNetworkSetting;
  doc["ssidAP"] = settings.ssidAP;
  doc["passwordAP"] = settings.passwordAP;
  doc["ipAddressAP"] = settings.ipAddressAP.toString();
  doc["isAP"] = settings.isAP;

  for (const auto& network : settings.networkSettings) {
    JsonObject networkSetting = networkSettingsArray.createNestedObject();
    networkSetting["ssid"] = network.ssid;
    networkSetting["password"] = network.password;
    networkSetting["useStaticIP"] = network.useStaticIP;
    networkSetting["staticIP"] = network.staticIP.toString();
    networkSetting["staticGateway"] = network.staticGateway.toString();
    networkSetting["staticSubnet"] = network.staticSubnet.toString();
    networkSetting["staticDNS"] = network.staticDNS.toString();
  }

  // Добавляем информацию о Telegram настройках
  JsonObject telegramSettingsObj = doc.createNestedObject("telegramSettings");
  telegramSettingsObj["isTelegramOn"] = settings.telegramSettings.isTelegramOn;
  telegramSettingsObj["botId"] = settings.telegramSettings.botId;
  telegramSettingsObj["lastMessage"] = settings.telegramSettings.lastMessage;

  // Добавляем информацию о пользователях Telegram
  JsonArray usersArray = telegramSettingsObj.createNestedArray("telegramUsers");
  for (const auto& user : settings.telegramSettings.telegramUsers) {
    JsonObject userObj = usersArray.createNestedObject();
    userObj["id"] = user.id;
    userObj["reading"] = user.reading;
    userObj["writing"] = user.writing;
  }

  File file = SPIFFS.open("/settings.json", FILE_WRITE);

  delay(500);
  if (!file) {
    delay(400);
    SPIFFS.end();
    delay(400);

    if (!SPIFFS.begin(true)) {
      Serial.println("Failed to mount SPIFFS on retry");
      if (wdt) {
        // esp_task_wdt_reset();  // Сброс таймера WDT
        // esp_task_wdt_delete(NULL);
      }
      return false;
    }
  }

  serializeJson(doc, file);
  file.close();
  Serial.println("\nSettings saved");

  if (wdt) {
    // esp_task_wdt_reset();  // Сброс таймера WDT
    // esp_task_wdt_delete(NULL);
  }

  return true;
}

//==============================
bool loadSettings(bool wdt = false) {
  delay(500);

  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount file system");
    return false;
  }

  File file = SPIFFS.open("/settings.json", "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return false;
  }

  size_t size = file.size();
  std::unique_ptr<char[]> buf(new char[size]);
  file.readBytes(buf.get(), size);
  DynamicJsonDocument doc(4096);  // Увеличиваем размер документа
  DeserializationError error = deserializeJson(doc, buf.get());

  if (error) {
    Serial.println("Failed to parse settings file");
    initializeWiFiSettings(settings);

    return false;
  }

  // Чтение сетевых настроек
  settings.networkSettings.clear();
  JsonArray networkSettingsArray = doc["networkSettings"];
  for (JsonObject networkSetting : networkSettingsArray) {
    NetworkSetting network;
    network.ssid = networkSetting["ssid"].as<String>();
    network.password = networkSetting["password"].as<String>();
    network.useStaticIP = networkSetting["useStaticIP"].as<bool>();
    network.staticIP.fromString(networkSetting["staticIP"].as<String>());
    network.staticGateway.fromString(networkSetting["staticGateway"].as<String>());
    network.staticSubnet.fromString(networkSetting["staticSubnet"].as<String>());
    network.staticDNS.fromString(networkSetting["staticDNS"].as<String>());
    settings.networkSettings.push_back(network);
  }

  // Чтение остальных настроек
  settings.isWifiTurnedOn = doc["isWifiTurnedOn"].as<bool>();
  settings.currentIdNetworkSetting = doc["currentIdNetworkSetting"].as<int>();
  settings.ssidAP = doc["ssidAP"].as<String>();
  settings.passwordAP = doc["passwordAP"].as<String>();
  settings.ipAddressAP.fromString(doc["ipAddressAP"].as<String>());
  settings.isAP = doc["isAP"].as<bool>();

  JsonObject telegramSettingsObj = doc["telegramSettings"];
  settings.telegramSettings.isTelegramOn = telegramSettingsObj["isTelegramOn"].as<bool>();
  settings.telegramSettings.botId = telegramSettingsObj["botId"].as<String>();
  //settings.telegramSettings.lastMessage = telegramSettingsObj["lastMessage"].as<String>();
  settings.telegramSettings.lastMessage = telegramSettingsObj["lastMessage"].isNull() ? "" : telegramSettingsObj["lastMessage"].as<String>();

  settings.telegramSettings.telegramUsers.clear();
  JsonArray usersArray = telegramSettingsObj["telegramUsers"];
  for (JsonObject user : usersArray) {
    TelegramUserID telegramUser;
    telegramUser.id = user["id"].as<String>();
    telegramUser.reading = user["reading"].as<bool>();
    telegramUser.writing = user["writing"].as<bool>();
    settings.telegramSettings.telegramUsers.push_back(telegramUser);
  }

  file.close();

  idSetting = settings.currentIdNetworkSetting;

  return true;
}

//===========================
// Проверка, если свободной памяти меньше критического порога
void checkMemory() {
  if (esp_get_free_heap_size() < 1024) {
    String timestamp = getFormattedTimeFromRTC();

    String s = timestamp + "\n";
    s += "Critical memory level detected!\n";
    s += "Restarting due to critical memory level.\n";

    settings.telegramSettings.lastMessage = s;
    saveSettings();
    delay(1000);
    res = true;
  }
}

//=============================
bool wasAPMode = false;

struct NetworkInfo {
  String ssid;
  int rssi;
};

NetworkInfo* ssidList = nullptr;  // Объявление массива как глобальной переменной и инициализация его нулевым указателем

String scanNetworks(bool isState = true) {

  if (isUpdate) {
    return "[]";
  }

  isScan = true;

  // Сохранение начального состояния WiFi
  wasAPMode = WiFi.getMode() == WIFI_AP;

  int inc = 0;
  int numNetworks = 0;

  while (numNetworks <= 0 && inc < 5) {
    numNetworks = WiFi.scanNetworks();
    Serial.println("\nScan networks\n " + String(numNetworks));

    if (numNetworks > 0) {
      break;
    }

    if (inc >= 3) {
      WiFi.disconnect(true);
      delay(300);
      WiFi.mode(WIFI_OFF);
      delay(300);
      WiFi.mode(WIFI_STA);
      delay(500);
    }

    inc++;
    delay(1000);  // Ожидание перед следующей попыткой
  }

  if (numNetworks <= 0) {
    // Если после 3 попыток сети не найдены, присваиваем numNetworks = 0
    Serial.println("No networks found after 3 attempts.");
    return "[]";
  }

  // Выделение памяти под массив структур NetworkInfo
  ssidList = new NetworkInfo[numNetworks];

  // Сохранение информации о сетях в массив
  for (int i = 0; i < numNetworks; ++i) {
    ssidList[i].ssid = WiFi.SSID(i);
    ssidList[i].rssi = WiFi.RSSI(i);
  }

  // Сортировка массива по убыванию силы сигнала (RSSI)
  for (int i = 0; i < numNetworks - 1; ++i) {
    for (int j = i + 1; j < numNetworks; ++j) {
      if (ssidList[i].rssi < ssidList[j].rssi) {
        NetworkInfo temp = ssidList[i];
        ssidList[i] = ssidList[j];
        ssidList[j] = temp;
      }
    }
  }

  // Формируем JSON строку
  String result = "[";
  for (int i = 0; i < numNetworks; ++i) {
    if (i) result += ",";
    result += "{";
    result += "\"ssid\":\"" + ssidList[i].ssid + "\",";
    result += "\"rssi\":" + String(ssidList[i].rssi);
    result += "}";
  }
  result += "]";

  // Освобождение памяти, если массив уже был выделен
  if (ssidList != nullptr) {
    delete[] ssidList;
    ssidList = nullptr;
  }

  if (isState) {
    if (wasAPMode) {
      WiFi.mode(WIFI_AP);
    } else {
      WiFi.mode(WIFI_STA);
    }
  }

  isScan = false;
  Serial.println("\nStop scan networks\n");
  return result;
}


//=============================
void getTimeFromRTC() {
  dateTime.setMinute = rtc.getMinute();
  dateTime.setHour = rtc.getHour(true);
  dateTime.setDay = rtc.getDay();
  dateTime.setMonth = rtc.getMonth() + 1;
  dateTime.setYear = rtc.getYear();
}

void getNowDateTime() {
  // Задаем часовой пояс прямо в configTime
  configTime(3 * 3600, 0, "pool.ntp.org");  // UTC+3 для Москвы

  struct tm timeinfo;

  if (getLocalTime(&timeinfo)) {
    time_t now = time(nullptr);

    rtc.setTime(now);
    getTimeFromRTC();

    Serial.println("Time set in RTC: " + rtc.getTime("%A, %d %B %Y %H:%M:%S"));
  } else {
    Serial.println("Failed to obtain time");
  }
}

//================================
void updateBotId() {

  //=========================================================
  if (settings.telegramSettings.botId != oldBotId) {

    if (bot != nullptr) {  // Проверка, что bot инициализирован
      bot->updateToken(settings.telegramSettings.botId);
      isBotToken = true;
    } else {
      Serial.println("Error: bot is not initialized.");
    }

    oldBotId = settings.telegramSettings.botId;  // Обновление старого значения botId
  }
}

//================================
void bot_setup() {

  const String commands = F("["
                            "{\"command\":\"status\",  \"description\":\"Текущий статус устройства\"},"
                            "{\"command\":\"reset\",  \"description\":\"Перезагрузить устойство\"},"
                            "{\"command\":\"update\",  \"description\":\"Обновить прошивку или файлы\"},"
                            "{\"command\":\"newtoken\",  \"description\":\"Установить новый бот токен\"}"
                            "]");
  botSetCommands(settings.telegramSettings.botId, commands);
  //bot->setMyCommands(commands);
}

//===============================

void startTelegramMessage() {
#ifdef ESP32
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  isCertificate = true;
#endif

#ifdef ESP8266
  configTime(0, 0, "pool.ntp.org");
  client.setTrustAnchors(&cert);
  isCertificate = true;
#endif

  Serial.print("BOT Send\n");

  for (const auto& user : settings.telegramSettings.telegramUsers) {
    if (!user.id.isEmpty() && user.reading && isBotToken && settings.telegramSettings.isTelegramOn) {

      String sendStr = "Hello\n";
      sendStr += "Доступ из локальной сети: http://" + WiFi.localIP().toString() + "\n";
      sendStr += "Подключено: " + WiFi.SSID() + "\n";

      sendStr += settings.telegramSettings.lastMessage;

      int messageSent = bot->sendMessage(user.id, sendStr, "");

      if (settings.telegramSettings.lastMessage.length() > 1) {
        settings.telegramSettings.lastMessage = "";
        saveSettings();
      }

      //deleteBotCommands(settings.telegramSettings.botId);

      bot_setup();

      if (messageSent == 1) {
        isCertificate = true;
        Serial.print("\nBotCode: " + String(messageSent) + "\n");

        return;
      } else {
        isCertificate = false;
        Serial.print("\nBotCode: " + String(messageSent) + "\n");
      }
    }
  }
}

//=====================

bool checkInternetConnection() {
  WiFiClient client;
  const char* urlList[] = { "api.telegram.org", "nc.ru" };
  int arraySize = sizeof(urlList) / sizeof(urlList[0]);

  if (isConnectedWiFi && !isStartedAP && !isUpdate) {
    for (int i = 0; i < arraySize; ++i) {
      Serial.print("Attemp connecting to: " + String(urlList[i]) + "\n");
      if (client.connect(urlList[i], 443)) {
        delay(300);  // Добавлено для устойчивости соединения
        isInternetConnected = true;
        Serial.print("\nInternet is connected\n");
        getNowDateTime();

        updateBotId();

        return true;
      }
    }
  }
  // Если не удалось установить соединение ни с одним из хостов
  Serial.print("\nNo internet connection\n");
  isInternetConnected = false;
  getTimeFromRTC();
  return false;
}

//======================================================================

int getTotalNetworks() {
  int count = 0;
  for (const auto& network : settings.networkSettings) {
    if (!network.ssid.isEmpty()) {
      count++;
    }
  }
  return count;
}

// Обновленная функция для подключения к Wi-Fi
bool connectToWiFi(const char* ssid, const char* password, int idSetting) {

  WiFi.mode(WIFI_STA);
  delay(100);

  if (settings.networkSettings[idSetting].useStaticIP) {
    WiFi.config(
      settings.networkSettings[idSetting].staticIP,
      settings.networkSettings[idSetting].staticGateway,
      settings.networkSettings[idSetting].staticSubnet,
      settings.networkSettings[idSetting].staticDNS);
  }

  WiFi.begin(ssid, password);
  Serial.println("Begin status WIFI: " + String(WiFi.status()));
  //unsigned long startTime = millis();
  int attempts = 0;
  const int maxAttempts = 10;
  const unsigned long connectionTimeout = 8000;

  Serial.print("Connect to: " + String(ssid) + " Index network: " + String(idSetting));
  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(1000);
    Serial.print(".");
    attempts++;
    // Serial.println(ESP.getFreeHeap() / 1024);
    if (attempts > 6) {
      WiFi.disconnect();
      Serial.println("\nFailed to connect");
      isConnectedWiFi = false;
      return false;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to " + String(ssid));
    IPAddress IP = WiFi.localIP();
    Serial.print("IP address: ");
    Serial.println(IP);

    isConnectedWiFi = true;
    isStartedAP = false;

    //analogWrite(ledPin, 255);  // Полная яркость

    // Проверка подключения к интернету и отправка сообщений
    checkInternetConnection();
    if (isInternetConnected && !isCertificate) {
      startTelegramMessage();
    }

    if (!settings.networkSettings[idSetting].useStaticIP) {
      settings.networkSettings[idSetting].staticIP = WiFi.localIP();
      settings.networkSettings[idSetting].staticGateway = WiFi.gatewayIP();
      settings.networkSettings[idSetting].staticSubnet = WiFi.subnetMask();
      settings.networkSettings[idSetting].staticDNS = WiFi.dnsIP();
    }

    return true;
  }
  Serial.println("\nFailed to connect end");
}

//===============================================================

String startAccessPoint(const char* ssid = nullptr, const char* password = nullptr, const char* ipAddress = nullptr) {

  // Проверяем параметры на наличие и устанавливаем дефолтные значения при необходимости
  ssid = ssid ? ssid : defaultSsidAP;
  password = password ? password : defaultPasswordAP;
  ipAddress = ipAddress ? ipAddress : local_IP.toString().c_str();

  // Запускаем точку доступа
  if (password && *password) {
    WiFi.softAP(ssid, password);
  } else {
    WiFi.softAP(ssid);
  }
  WiFi.softAPConfig(local_IP, gateway, subnet);

  // Получаем IP-адрес точки доступа
  IPAddress IP = WiFi.softAPIP();
  String ipStr = IP.toString();

  // Проверяем, что IP-адрес доступен
  if (IP != IPAddress(0, 0, 0, 0)) {
    Serial.println("Access Point IP: " + ipStr);
    isStartedAP = true;
    isConnectedWiFi = false;
    //analogWrite(ledPin, 100);  // Половина яркости
    return "Access Point set to SSID: " + String(ssid) + " with IP: " + ipStr;

  } else {
    Serial.println("Failed to start Access Point");
    isStartedAP = false;
    //analogWrite(ledPin, 50);  // Половина яркости
    return "Failed to start Access Point";
  }
}
//===============================================================

bool connectToBestAvailableNetwork() {
  Serial.println("connectToBestAvailableNetwork");
  bool connected = false;
  int networksFound = 0;

  String networksJson = scanNetworks(true);  // Отсканируем доступные сети

  if (networksJson.length() < 3) {
    isConnectionAttempts = false;
    return false;
  }

  const size_t capacity = JSON_ARRAY_SIZE(20) + 20 * JSON_OBJECT_SIZE(2) + 1024;
  DynamicJsonDocument doc(capacity);

  DeserializationError error = deserializeJson(doc, networksJson);
  if (error) {
    Serial.println("Failed to parse JSON");

    connected = false;
    isConnectionAttempts = false;
  }

  if (!error) {
    JsonArray networks = doc.as<JsonArray>();
    networksFound = networks.size();  // Получаем количество найденных сетей
    Serial.println(networksJson);
    //Serial.println(networksFound);

    int i = 0;

    while (!connected && i < networksFound) {
      JsonObject network = networks[i];
      String ssid = network["ssid"].as<String>();

      int networks = getTotalNetworks();
      totalNetworks = networks;
      // Перебираем настройки сети и пытаемся подключиться
      for (int j = 0; j < networks; ++j) {
        if (ssid == settings.networkSettings[j].ssid) {
          Serial.println("Trying to connect to saved network: " + ssid);
          bool result = connectToWiFi(ssid.c_str(), settings.networkSettings[j].password.c_str(), j);

          if (WiFi.status() == WL_CONNECTED) {
            settings.currentIdNetworkSetting = j;
            //saveSettings();
            connected = true;
            return result;  // Успешно подключились
          } else {
            Serial.println("Failed to connect to " + ssid);
          }
        }
      }

      i++;
    }
  }

  // Если все сети проверены и не удалось подключиться
  if (!connected) {
    isConnectionAttempts = false;
    Serial.println("Failed to connect to any saved network, switching to AP Mode");
    startAccessPoint(settings.ssidAP.c_str(), settings.passwordAP.c_str(), settings.ipAddressAP.toString().c_str());
  }

  return false;
}

//==================================

void sendInitialMenu() {
  for (const auto& user : settings.telegramSettings.telegramUsers) {
    if (!user.id.isEmpty() && user.reading && isBotToken && settings.telegramSettings.isTelegramOn) {
      String sendStr = "Telegram bot is ready!\n";
      // sendStr += "Доступ из локальной сети: http://" + WiFi.localIP().toString() + "\n";
      // sendStr += "Подключено: "+ WiFi.SSID() + "\n";

      int messageSent = bot->sendMessage(user.id, sendStr, "");
    }
  }
}
//======================================

void changeBotId(String newToken) {
  isBotToken = false;
  delay(0);
  if (newToken.length() > 20) {
    // Освобождаем память, занятую текущим объектом бота
    if (bot != nullptr) {
      delete bot;
      bot = nullptr;
    }

    // Создаем новый объект бота с новым токеном
    bot = new UniversalTelegramBot(newToken, client);
    Serial.println("\nBot token changed");

    //      if (isInternetConnected) {
    //      bool responseCode = bot->getMe();
    //
    //      if (!responseCode) {
    //        Serial.println("Error: Unable to connect to Telegram server");
    //        Serial.print("Response code: ");
    //        Serial.println(responseCode);
    //
    //      } else {
    //        isBotToken = true;
    //        Serial.println("Successfully connected to Telegram server");
    //        sendInitialMenu();
    //      }
    //     }

    isBotToken = true;

    Serial.print("New Bot Token: ");
    Serial.println(newToken);
    Serial.println("\nBot initialized");


  } else {
    Serial.println("New token is empty. Token not changed.");
    isBotToken = false;
  }
}

//=================================
void deleteNetwork(String ssid) {
  auto it = std::find_if(settings.networkSettings.begin(), settings.networkSettings.end(),
                         [&ssid](const NetworkSetting& network) {
                           return network.ssid == ssid;
                         });

  if (it != settings.networkSettings.end()) {
    settings.networkSettings.erase(it);

    // Обновление текущего индекса сети
    if (settings.networkSettings.empty()) {
      settings.currentIdNetworkSetting = -1;  // Сброс текущего индекса, если нет ни одной сети
    } else {
      int index = std::distance(settings.networkSettings.begin(), it);
      if (settings.currentIdNetworkSetting == index) {
        settings.currentIdNetworkSetting = 0;  // Если удаляемый элемент был текущим выбранным, устанавливаем на первый элемент
      } else if (settings.currentIdNetworkSetting > index) {
        settings.currentIdNetworkSetting--;  // Если текущий выбранный элемент находится после удаленного, смещаем индекс на один назад
      }
    }
  }
}

//================================= r.manualMode = true;r.manualMode = true;

void serverProcessingControl() {
 

  server.on("/relay", HTTP_POST, [](AsyncWebServerRequest* request) {
    // Проверяем наличие параметров
    if (!request->hasParam("relay", true) || !request->hasParam("action", true)) {
      request->send(400, "text/plain", "Missing parameters");
      return;
    }

    // Извлечение параметров
    String relay = request->getParam("relay", true)->value();
    String action = request->getParam("action", true)->value();

    Serial.printf("Received Relay: %s, Action: %s\n", relay.c_str(), action.c_str());

    bool relayFound = false;

    for (auto& r : control.relays) {
      if (r.description == relay) {
        relayFound = true;
        r.statePin = (action == "on");
      
        Serial.printf("Setting Relay Pin %d to %s\n", r.pin, r.statePin ? "HIGH" : "LOW");
        r.manualMode = true;
        digitalWrite(r.pin, r.statePin ? HIGH : LOW);
        isSaveControl = true;
        request->send(200, "text/plain", "Command executed");
        break;
      }
    }

    if (!relayFound) {
      request->send(404, "text/plain", "Relay not found");
    }
  });

  server.on("/resetManual", HTTP_POST, [](AsyncWebServerRequest* request) {
    Serial.println("Resetting all relays to Auto mode");

    bool anyRelayUpdated = false;

    for (auto& r : control.relays) {
      if (r.manualMode) {
        r.manualMode = false;  // Устанавливаем режим в Auto
        r.statePin = false;
        digitalWrite(r.pin, LOW);
        anyRelayUpdated = true;

        Serial.printf("Relay %s set to Auto mode\n", r.description.c_str());
      }
    }

    if (anyRelayUpdated) {
      isSaveControl = true;
      request->send(200, "text/plain", "All relays reset to Auto");
    } else {
      request->send(200, "text/plain", "No relays to reset");
    }
  });

  server.on("/relayStates", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = "{";
    json += "\"relays\":[";
    for (const auto& relay : control.relays) {
        json += "{";
        json += "\"description\":\"" + relay.description + "\",";
        json += "\"state\":" + String(digitalRead(relay.pin) ? "true" : "false") + ",";
        json += "\"mode\":\"" + String(relay.manualMode ? "Manual" : "Auto") + "\"";
        json += "},";
    }
    if (!control.relays.empty()) {
        json.remove(json.length() - 1);  // Удалить последнюю запятую
    }
    json += "],";
    json += "\"temp\":" + String(Temp);
    json += "}";
    request->send(200, "application/json", json);
    
    //Serial.println(json);
});

//========= настройки control
server.on("/getRelaySettings", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Создаем список доступных пинов
    std::vector<int> availablePins = control.pins;

    // Формируем JSON-ответ
    String response = "{";
    response += "\"relays\":[";

    bool firstRelay = true;
    for (const auto &relay : control.relays) {
        // Проверяем, что у реле есть описание
        if (!relay.description.isEmpty()) {
            if (!firstRelay) {
                response += ","; // Добавляем запятую между объектами
            } else {
                firstRelay = false;
            }
            
            Serial.print("load relay.statePin: ");
            Serial.println(relay.modePin); // Логируем значение modePin
            
            // Формируем объект для текущего реле
            response += "{";
            response += "\"pin\":" + String(relay.pin) + ",";
            response += "\"modePin\":\"" + relay.modePin + "\",";
            response += "\"manualMode\":" + String(relay.manualMode ? "true" : "false") + ",";
            response += "\"statePin\":" + String(relay.statePin ? "true" : "false") + ",";
            response += "\"description\":\"" + relay.description + "\"";
            response += "}";
        }
    }
    response += "],";

    // Добавляем список доступных пинов
    response += "\"availablePins\":[";
    for (size_t i = 0; i < availablePins.size(); ++i) {
        response += String(availablePins[i]);
        if (i < availablePins.size() - 1) {
            response += ",";
        }
    }
    response += "]";

    response += "}";

    // Отправляем ответ клиенту
    request->send(200, "application/json", response);
});

// Обработчик для сохранения настроек реле
server.on("/saveRelaySettings", HTTP_POST, [](AsyncWebServerRequest* request) {
    Serial.println("Received POST request to /saveRelaySettings");

    printRequestParameters(request) ;

    String requestBody;
    if (request->hasParam("relaySettings", true)) {
        requestBody = request->getParam("relaySettings", true)->value();
        Serial.println(requestBody);
    } else {
        Serial.println(requestBody);
        request->send(400, "application/json", "{\"error\":\"Missing relaySettings parameter\"}");
        return;
    }

    // Обработка тела запроса
    DynamicJsonDocument doc(2048); // Увеличьте размер, если необходимо
    DeserializationError error = deserializeJson(doc, requestBody);

    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        return;
    }

    // Проверка, является ли JSON массивом
    if (!doc.is<JsonArray>()) {
        Serial.println(F("Invalid JSON: Expected an array"));
        request->send(400, "application/json", "{\"error\":\"Expected a JSON array\"}");
        return;
    }

    JsonArray relays = doc.as<JsonArray>();
    control.relays.clear(); // Очищаем существующий массив реле

    for (JsonVariant v : relays) {
        if (!v.is<JsonObject>()) {
            Serial.println(F("Invalid JSON object in array"));
            request->send(400, "application/json", "{\"error\":\"Invalid JSON structure\"}");
            return;
        }

        Relay relay;
        JsonObject relayObj = v.as<JsonObject>();

        // Заполняем данные реле с проверкой
        relay.pin = relayObj.containsKey("pin") ? relayObj["pin"].as<int>() : 5;
        relay.modePin = relayObj.containsKey("modePin") ? relayObj["modePin"].as<String>() : "OUTPUT";
        relay.manualMode = relayObj.containsKey("manualMode") ? relayObj["manualMode"].as<bool>() : false;
        relay.statePin = relayObj.containsKey("statePin") ? relayObj["statePin"].as<bool>() : false;
        relay.description = relayObj.containsKey("description") ? relayObj["description"].as<String>() : "Relay_1";

        control.relays.push_back(relay);

        // Вывод информации о реле в консоль
        Serial.printf("Relay added: Pin=%d, Mode=%s, ManualMode=%d, StatePin=%d, Description=%s\n",
                      relay.pin, relay.modePin.c_str(), relay.manualMode, relay.statePin, relay.description.c_str());
    }

    request->send(200, "application/json", "{\"status\":\"Success\"}");
});

// Обработчик для добавления нового реле
server.on("/addRelay", HTTP_POST, [](AsyncWebServerRequest* request) {
    Serial.println("Received POST request to /addRelay");
    printRequestParameters(request);
     // Проставляем значения по умолчанию
    String pin = request->hasParam("pin", true) ? request->getParam("pin", true)->value() : "";
    String modePin = request->hasParam("modePin", true) ? request->getParam("modePin", true)->value() : "OUTPUT";
    String manualMode = request->hasParam("manualMode", true) ? request->getParam("manualMode", true)->value() : "false";
    String statePin = request->hasParam("statePin", true) ? request->getParam("statePin", true)->value() : "false";
    String description = request->hasParam("description", true) ? request->getParam("description", true)->value() : "Relay_1";

   // Проверяем, что все необходимые параметры присутствуют
    if (pin.isEmpty() || modePin.isEmpty()) {
        request->send(400, "application/json", "{\"error\":\"Missing required fields\"}");
        return;
    }

    // Создаем объект реле
    Relay relay;
    relay.pin = pin.toInt();
    relay.modePin = modePin;
    relay.manualMode = (manualMode == "on");
    relay.statePin = (statePin == "true");
    relay.description = description;
    
    // Добавляем реле в список
    control.relays.push_back(relay);

    // Логируем информацию о добавленном реле
    Serial.printf("New relay added: Pin=%d, Mode=%s, ManualMode=%d, StatePin=%d, Description=%s\n",
                  relay.pin, relay.modePin, relay.manualMode, relay.statePin, relay.description.c_str());

    // Отправляем успешный ответ
    request->send(200, "application/json", "{\"status\":\"Success\"}");
});


}
//========================================


void serverProcessing() {
  //if (isConnectedWiFi || isStartedAP) {
  Serial.println("serverProcessing");

  // Настройка маршрутов для загрузки HTML страницы из SPIFFS
  // Настройка сервера
  server.on("/", HTTP_GET, handleRoot);

  //=======================

    // Обработчик для получения списка файлов
    server.on("/fileList", HTTP_GET, [](AsyncWebServerRequest *request) {
        File root = SPIFFS.open("/");
        if (!root || !root.isDirectory()) {
            request->send(500, "application/json", "{\"error\":\"Failed to access SPIFFS\"}");
            return;
        }

        String fileList = "[";
        File file = root.openNextFile();
        while (file) {
            if (fileList.length() > 1) {
                fileList += ",";
            }
            fileList += "\"" + String(file.name()) + "\"";
            file.close();
            file = root.openNextFile();
        }
        fileList += "]";
        request->send(200, "application/json", fileList);
    });

    // Обработчик для скачивания конкретного файла
    server.on("/downloadFile", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("filename")) {
            request->send(400, "text/plain", "Missing 'filename' parameter");
            return;
        }

        String filename = request->getParam("filename")->value();
        if (!SPIFFS.exists(filename)) {
            request->send(404, "text/plain", "File not found");
            return;
        }

        AsyncWebServerResponse *response = request->beginResponse(SPIFFS, filename, "application/octet-stream");
        response->addHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
        request->send(response);
    });
  // Обработчик для получения настроек
  server.on("/getsettings", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = getSettingsJson();
    request->send(200, "application/json", json);
  });

  server.on("/scan", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!isScan) {
      isBotToken = false;
      isScan = true;
      request->send(200, "application/json", "{\"success\":true,\"message\":\"Scan started\"}");
    } else {
      request->send(400, "application/json", "{\"success\":false,\"message\":\"Scan already in progress\"}");
    }
    isBotToken = true;
  });

  server.on("/getNetworks", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (isScan) {
      request->send(200, "application/json", "[]");
      Serial.println(scannedNetworks);
    } else {
      request->send(200, "application/json", scannedNetworks);
    }
  });

  server.on("/applySettings", HTTP_POST, [](AsyncWebServerRequest* request) {
    String mode = request->getParam("mode", true)->value();
    bool isAP = request->hasParam("is_ap", true);  // Проверка наличия параметра is_ap

    if (mode == "client") {
      settings.isAP = false;
      //settings.ssidAP = ""; // Очищаем поле SSID точки доступа

      if (request->hasParam("ssid", true) && request->hasParam("password", true) && request->hasParam("ip_mode", true)) {
        String newSSID = request->getParam("ssid", true)->value();
        String newPassword = request->getParam("password", true)->value();
        String ipMode = request->getParam("ip_mode", true)->value();
        int selectedNetworkIndex = -1;

        // Проверяем, существует ли уже SSID в векторе
        for (size_t i = 0; i < settings.networkSettings.size(); ++i) {
          if (settings.networkSettings[i].ssid == newSSID) {
            selectedNetworkIndex = i;
            break;
          }
        }

        // Если SSID не найден, добавляем новый элемент в вектор
        if (selectedNetworkIndex == -1) {
          // Добавляем новый элемент в вектор
          settings.networkSettings.push_back(NetworkSetting());
          selectedNetworkIndex = settings.networkSettings.size() - 1;

          // Инициализируем новый элемент
          settings.networkSettings[selectedNetworkIndex].ssid = newSSID;
          settings.networkSettings[selectedNetworkIndex].password = newPassword;
        } else {
          // Обновляем существующую сеть
          settings.networkSettings[selectedNetworkIndex].password = newPassword;
        }

        NetworkSetting& selectedNetwork = settings.networkSettings[selectedNetworkIndex];

        if (ipMode == "static") {
          selectedNetwork.useStaticIP = true;
          selectedNetwork.staticIP.fromString(request->getParam("static_ip", true)->value());
          selectedNetwork.staticGateway.fromString(request->getParam("gateway", true)->value());
          selectedNetwork.staticSubnet.fromString(request->getParam("subnet", true)->value());
          selectedNetwork.staticDNS.fromString(request->getParam("dns", true)->value());
        } else {
          selectedNetwork.useStaticIP = false;
        }

        settings.currentIdNetworkSetting = selectedNetworkIndex;  // Устанавливаем текущий ID сети
        settings.isWifiTurnedOn = true;                           // Предполагаем, что Wi-Fi включен после настройки
      }
    } else if (mode == "is_ap" && isAP) {
      settings.isAP = true;
      settings.ssidAP = request->getParam("ap_ssid", true)->value();

      if (request->hasParam("ap_password", true)) {
        settings.passwordAP = request->getParam("ap_password", true)->value();
      }

      settings.ipAddressAP.fromString(request->getParam("ap_ip", true)->value());
      settings.isWifiTurnedOn = true;  // Предполагаем, что Wi-Fi включен после настройки
    }

    // Обработка настроек Telegram
    if (request->hasParam("botId", true)) {
      String botId = request->getParam("botId", true)->value();
      settings.telegramSettings.botId = botId;

      updateBotId();

      String isTelegramOnStr = request->getParam("isTelegramOn", true)->value();
      settings.telegramSettings.isTelegramOn = (isTelegramOnStr == "true");

      // Очищаем текущий список пользователей
      settings.telegramSettings.telegramUsers.clear();

      int userIndex = 0;
      if (request->hasParam("users", true)) {
        String usersParam = request->getParam("users", true)->value();
        DynamicJsonDocument doc(2048);
        DeserializationError error = deserializeJson(doc, usersParam);

        if (error) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.c_str());
          request->send(400, "application/json", "{\"success\":false,\"message\":\"Invalid JSON\"}");
          return;
        }

        JsonArray usersArray = doc.as<JsonArray>();

        for (JsonObject userObj : usersArray) {
          if (userIndex < settings.telegramSettings.telegramUsers.capacity()) {
            settings.telegramSettings.telegramUsers.push_back({ userObj["id"].as<String>(),
                                                                userObj["reading"].as<bool>(),
                                                                userObj["writing"].as<bool>() });
            userIndex++;
          }
        }
      }
    }

    bool isResult = saveSettings();
    
    isSaveControl = true;


    if (isResult) {
      request->send(200, "application/json", "{\"success\":true,\"message\":\"Settings applied successfully\"}");
    } else {
      request->send(500, "application/json", "{\"success\":false,\"message\":\"Failed to apply settings\"}");
    }
  });

  server.on("/deleteNetwork", HTTP_POST, [](AsyncWebServerRequest* request) {
    printRequestParameters(request);
    if (request->hasParam("ssid", true)) {
      String ssid = request->getParam("ssid", true)->value();

      deleteNetwork(ssid);

      // Отправляем успешный ответ клиенту
      request->send(200, "application/json", "{\"success\":true,\"message\":\"Network deleted successfully\"}");
    } else {
      // Отправляем ошибку клиенту, если параметр "ssid" отсутствует в запросе
      request->send(400, "application/json", "{\"success\":false,\"message\":\"SSID parameter is missing\"}");
    }
  });

  //========== Загрузка файла =========
  server.on("/uploadFile", HTTP_POST, [](AsyncWebServerRequest* request) {
      // Обработчик для отправки ответа после загрузки
      request->send(200, "text/plain", "File Uploaded Successfully");
    },
    [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* fileData, size_t len, bool final) {
      handleFileUpload(request, filename, index, fileData, len, final);
    });

  //=============Рестарт===============
  // Обработчик для перезагрузки устройства
  server.on("/restart", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", "{\"message\":\"Устройство перезагружается...\"}");
    // Задержка перед перезагрузкой, чтобы клиент получил ответ
    delay(500);
    ESP.restart();
  });

  server.on("/format", HTTP_POST, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", "{\"message\":\"Устройство форматируется...\"}");
    // Задержка перед перезагрузкой, чтобы клиент получил ответ
    delay(500);
    SPIFFS.format();
    request->send(200, "application/json", "{\"success\":true,\"message\":\"Formated successfully\"}");
    delay(500);
    ESP.restart();
  });

  // Настройка сервера для изменения botId
  server.on("/set_bot_id", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("botId", true)) {
      String newBotId = request->getParam("botId", true)->value();
      settings.telegramSettings.botId = newBotId;

      Serial.println(newBotId);

      String botName = "";
      if (isInternetConnected) {
        botName = checkNewToken(newBotId);
      }

      updateBotId();

      // Отправляем ответ с именем бота
      String response = "{\"success\":true,\"message\":\"Bot ID applied successfully\", \"botName\":\"" + botName + "\"}";
      request->send(200, "application/json", response);
    } else {
      request->send(400, "application/json", "{\"message\":\"Invalid request\"}");
    }
  });

  // Обработчик для добавления пользователя
  server.on("/add_user", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("user_id", true) && request->hasParam("reading", true) && request->hasParam("writing", true)) {
      String userId = request->getParam("user_id", true)->value();
      bool reading = request->getParam("reading", true)->value() == "true";
      bool writing = request->getParam("writing", true)->value() == "true";

      // Добавляем нового пользователя в вектор
      settings.telegramSettings.telegramUsers.push_back({ userId, reading, writing });

      request->send(200, "application/json", "{\"success\":true,\"message\":\"User added successfully\"}");
    } else {
      request->send(400, "application/json", "{\"success\":false,\"message\":\"Required parameters missing\"}");
    }
  });
  

  server.on("/sysStatus", HTTP_GET, [](AsyncWebServerRequest* request) {
    String status = getSystemStatus();
    request->send(200, "text/plain", status);
  });

    // Обработчик для удаления пользователя
    server.on("/delete_user", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("user_id", true)) {
        String userId = request->getParam("user_id", true)->value();
        auto& users = settings.telegramSettings.telegramUsers;

        // Ищем пользователя по userId
        auto it = std::find_if(users.begin(), users.end(), [&](const auto& user) {
            return user.id == userId;
        });

        if (it != users.end()) {
            // Удаляем пользователя из вектора
            users.erase(it);
            request->send(200, "application/json", "{\"success\":true,\"message\":\"User deleted successfully\"}");
        } else {
            request->send(404, "application/json", "{\"message\":\"User not found\"}");
        }
        Serial.println(userId);

    } else {
        request->send(400, "application/json", "{\"message\":\"Invalid request\"}");
    }
 });

  server.begin();
}

//===================================
String parseNetworkSettings(const WiFiSettings& settings) {
  String result;
  for (const NetworkSetting& netSetting : settings.networkSettings) {
    if (!netSetting.ssid.isEmpty()) {
      result += "SSID: " + netSetting.ssid + "\n";
      result += "Password: " + netSetting.password + "\n";
      result += "DHCP: " + String(netSetting.useStaticIP ? "FALSE" : "TRUE") + "\n";
      result += "Static IP: " + netSetting.staticIP.toString() + "\n";
      result += "Static Gateway: " + netSetting.staticGateway.toString() + "\n";
      result += "Static Subnet: " + netSetting.staticSubnet.toString() + "\n";
      result += "Static DNS: " + netSetting.staticDNS.toString() + "\n";
      result += "\n";
    }
  }
  return result;
}

//================================
// Функция для поиска пользователя по его идентификатору
const TelegramUserID* getUserById(const String& userId) {
  for (const auto& user : settings.telegramSettings.telegramUsers) {
    if (user.id == userId) {
      return &user;
    }
  }
  return nullptr;  // Пользователь не найден
}
//==================================

void updateStatus() {
  if (previousStatus != statusUpdate) {
    bot->sendMessage(CHAT_ID, statusUpdate, "");
    previousStatus = statusUpdate;
  }
}

//==================================
void sendMessageWithMarkdown(String chat_id, String message) {
  // Создаем JSON-объект
  DynamicJsonDocument doc(1024);
  JsonObject payload = doc.to<JsonObject>();

  // Заполняем JSON-объект
  payload["chat_id"] = chat_id;
  payload["text"] = message;
  payload["disable_web_page_preview"] = true;  // Отключаем предварительный просмотр
  payload["parse_mode"] = "Markdown";          // Указываем использование Markdown "HTML"

  // Отправляем сообщение
  bool result = bot->sendPostMessage(payload, false);  // false означает, что не требуется редактирование сообщения
}

//==================================

bool isValidTokenFormat(const String& token) {
  int colonIndex = token.indexOf(':');

  if (colonIndex == -1 || colonIndex == 0 || colonIndex == token.length() - 1) {
    return false;
  }

  String userIdPart = token.substring(0, colonIndex);
  for (int i = 0; i < userIdPart.length(); i++) {
    if (!isDigit(userIdPart[i])) {
      return false;
    }
  }

  String secretPart = token.substring(colonIndex + 1);
  if (secretPart.length() < 20) {
    return false;
  }

  return true;
}

//==================================

File sendFileToTg;

bool isMoreDataAvailable() {
  return sendFileToTg.available();
}

byte getNextByte() {
  return sendFileToTg.read();
}


//================================
bool handleTelegramCommand(String chat_id, String command) {
  command.trim();  // Удалить лишние пробелы в начале и конце строки

  // Проверка и обработка команды "/on" и "/off"
  if (command.startsWith("/on") || command.startsWith("/off")) {
    bool turnOn = command.startsWith("/on");
    int relayNumber = command.substring(turnOn ? 3 : 4).toInt();  // Извлечь номер реле

    // Проверка, что relayNumber в диапазоне 1-4
    if (relayNumber >= 1 && relayNumber <= control.relays.size()) {
      int relayIndex = relayNumber - 1;
      auto& r = control.relays[relayIndex];
      r.statePin = turnOn;
      r.manualMode = true;
      digitalWrite(r.pin, turnOn ? HIGH : LOW);
      Serial.printf("Relay %d turned %s\n", r.pin, turnOn ? "ON" : "OFF");
      isSaveControl = true;  // Сохраняем состояние после изменения
      return true;
    } else {
      Serial.println("Invalid relay number");
      bot->sendMessage(chat_id, "Invalid relay number", "");
      return false;
    }
  }
  // Проверка команды "/statusControl" 
  else if (command == "/status") {
    String sOut = sendStatus();
    bot->sendMessage(chat_id, sOut, "");
    return true;
  }

  // Проверка команды "/resetManual"
  else if (command == "/resetManual") {
    String sOut;
    bool anyRelayUpdated;
    
    for (auto& r : control.relays) {
      if (r.manualMode) {
        r.manualMode = false;  // Устанавливаем режим в Auto
        r.statePin = false;
        digitalWrite(r.pin, LOW);
        anyRelayUpdated = true;

        sOut += r.description + String(" set to Auto mode\n");
      }
    }
    if (anyRelayUpdated) {
      isSaveControl = true;  // Сохраняем состояние после изменения

      bot->sendMessage(chat_id, sOut, "");
      Serial.print(sOut);
    }
    return true;
  }
  // Проверка и обработка команды "/setLimitWorkPressurePump"
//  else if (command.startsWith("/setLimitWorkPressurePump")) {
//    int value = command.substring(25).toInt();  // Извлечь значение
//    control.limitWorkPressurePump = value;
//    Serial.printf("limitWorkPressurePump set to %d\n", value);
//    isSaveControl = true;
//    return true;
//  }
  //===============
  
   else if (command.startsWith("/help")) {
    bool value = command.substring(5).equalsIgnoreCase("true");  // Извлечь значение
    String res = sendHelp();
    bot->sendMessage(chat_id, res, "");
    return true;
  } else if (command.startsWith("/resetFull")) {
    bool value = command.substring(10).equalsIgnoreCase("true");  // Извлечь значение
    initializeControl();
    bot->sendMessage(chat_id, "Все параметры сброшены по умолчанию", "");
    isSaveControl = true;
    return true;
  } else if (command.startsWith("/debug")) {
    bool value = command.substring(6).equalsIgnoreCase("true");  // Извлечь значение
    String outStr = debugInfo();
    bot->sendMessage(chat_id, outStr, "");

    return true;
  } else if (command.startsWith("/log")) {
    bool value = command.substring(4).equalsIgnoreCase("true");  // Извлечь значение
    String outStr = getLog();
    bot->sendMessage(chat_id, outStr, "");

    return true;
  }
  // Неизвестная команда
  else {
    Serial.println("Unknown command. Please use /help for help.");
    return false;
  }
}


//=================================

void handleNewMessages(int numNewMessages) {
  delay(10);
  //Serial.println(String(numNewMessages));

  for (int i = 0; i < numNewMessages; i++) {

    // Идентификатор чата запроса
    String chat_id = String(bot->messages[i].chat_id);
    CHAT_ID = chat_id;

    int message_id = bot->messages[i].message_id;
    String text = bot->messages[i].text;

    if (text.length() > 512) {
      bot->sendMessage(chat_id, "Ваше сообщение слишком длинное и было проигнорировано.", "");
      continue;
    }

    const TelegramUserID* user = getUserById(chat_id);
    if (user == nullptr) {
      bot->sendMessage(chat_id, "Access is denied, user does not exist", "");
      continue;
    }


    // Проверка доступа на чтение и запись
    if (!user->reading) {
      bot->sendMessage(chat_id, "Read access denied", "");
      continue;
    }

    if (!user->writing) {
      bot->sendMessage(chat_id, "Write access denied", "");
      continue;
    }

    Serial.println(text);

    //======КНОПКИ====
    if (handleTelegramCommand(chat_id, text)) {
    } else

      if (text == "/statusSystem") {
      bot->sendMessage(chat_id, getSystemStatus(), "");
    } else

      if (text == "/start") {
      bot->sendMessage(chat_id, "Hello", "");
    } else

      if (text == "/reset") {
      bot->sendMessage(chat_id, "Перезагрузка", "");
      delay(1000);
      res = true;
      delay(500);
    } else

      if (text == "/update") {
      isUpdate = true;

      String message = "Выберете файл прошивки .bin, контента .html или настроек .json📎. [Скачать прошивку](https://cloud.mail.ru/public/KtJ5/WgxbfXTrP)";
      sendMessageWithMarkdown(chat_id, message);

    } else if (isUpdate) {

      size_t file_size = bot->messages[i].file_size;
      String file_path = bot->messages[i].file_path;
      String file_name = bot->messages[i].file_name;

      size_t totalBytes = SPIFFS.totalBytes();
      size_t usedBytes = SPIFFS.usedBytes();
      size_t freeBytes = totalBytes - usedBytes;

      //      Serial.println(file_path);
      //      Serial.println(file_name);
      //      Serial.println(file_size);

      String extension = file_name.substring(file_name.lastIndexOf('.') + 1);

      bool sizeOk = freeBytes > file_size || extension == "bin";
      if (!sizeOk) {
        bot->sendMessage(chat_id, "Превышен размер файла. Свободно: " + String(freeBytes / 1024) + " KB. Размер файла: " + String(file_size / 1024) + " KB.", "");
      }

      if (sizeOk && file_size > 0 && file_path.length() > 10) {

        bot->sendMessage(chat_id, "Файл принят: " + file_name, "");

        if (extension == "bin") {
          file_name = "firmware.bin";  // Переименовываем файл так как длинные имена и с подчеркиванием все ломают
        }

        TaskParameters* params = new TaskParameters;
        params->filePath = file_path;  // Передаем путь к файлу
        params->fileName = file_name;  // Передаем имя файла

        delay(200);

        BaseType_t resultOta = xTaskCreate(
          otaUpdateTask,  // Функция задачи
          "OTA Update",   // Имя задачи
          8192,           // Размер стека
          params,         // Параметры задачи
          1,              // Приоритет
          NULL            // Дескриптор задачи
        );

        if (resultOta != pdPASS) {
          Serial.println("Failed to create OTA update task.");
          isUpdate = false;
          bot->sendMessage(chat_id, "Повторите попытку", "");
        }

        return;
      }
      //isUpdate = false;  // Сброс флага обновления перенесен в otaUpdateTask

    } else

      if (text == "/newtoken") {
      isNewToken = true;
      bot->sendMessage(chat_id, "Отправьте новый токен бота. Для создания бота: @BotFather", "");

    } else

      // не работает пока
      if (text.startsWith("/sendfile ")) {
        String fileNameSend = text.substring(10);
        String filePath = "/" + fileNameSend;

        sendFileToTg = SPIFFS.open(filePath, FILE_READ);
        if (!sendFileToTg) {
          Serial.println("Не удалось открыть файл для чтения");
          return;
        }

        String response = bot->sendMultipartFormDataToTelegram("sendDocument", "document", sendFileToTg.name(), "txt", chat_id, sendFileToTg.size(), isMoreDataAvailable, getNextByte, nullptr, nullptr);

        Serial.println("Ответ сервера: " + response);
        sendFileToTg.close();

      }

      else if (isNewToken) {
        text.trim();
        String botName = checkNewToken(text);
        bool isCheck = (botName != "") && isValidTokenFormat(text);

        if (isCheck) {
          String outStr = "Новый токен успешно применен, перейдите в новый телеграм бот: " + String("@") + botName;
          bot->sendMessage(chat_id, outStr, "");

          bot->updateToken(text);
          settings.telegramSettings.botId = text;

          saveSettings();

        } else {
          bot->sendMessage(chat_id, "Введеный токен не корректный или бот не существует", "");
        }

        isNewToken = false;
      } else

        if (text == "/format") {
        bot->sendMessage(chat_id, "Форматирование файловой системы", "");
        SPIFFS.format();
      } else

        if (text == "/list") {
        String listF = "Список файлов:\n";
        listF += listFiles();

        bot->sendMessage(chat_id, listF, "");
      } else if (text == "/networks") {
        String result = parseNetworkSettings(settings);
        bot->sendMessage(chat_id, result, "");
      }

    //=== END КНОПКИ =====//
    //return;
  }
  // return;
}
//==================

// Задача тиктака

void tikTak() {

  //if (!isUpdate) checkMemory();

  controlTask();

  if ((!settings.isAP || !isStartedAP) && settings.isWifiTurnedOn && !isUpdate) {
    tiktak1++;
  } else {
    tiktak1 = 0;
  }

  if (!isConnectionAttempts) {
    tiktak2++;
  } else {
    tiktak2 = 0;
  }

  if (settings.isWifiTurnedOn && !isStartedAP && WiFi.status() == WL_DISCONNECTED && isConnectionAttempts) {
    tiktak3++;
  } else {
    tiktak3 = 0;
  }
}
