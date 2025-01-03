
#include <Arduino.h>
#include "esp_system.h"
//======================
#include "Settings.h"
#include "Devices.h"

//======================

void setup() {

  Serial.begin(115200);

  //====================================
  //pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLDOWN); // INPUT_PULLDOWN  INPUT_PULLUP// нужно подтянуть к плючсу так как прерывание сравботает при п

  pinMode(tempPin, INPUT);


 unsigned long buttonPressStartTime = 0;

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    delay(2000);
    if (!SPIFFS.begin(true)) {
      Serial.println("Format SPIFFS");
      SPIFFS.format();
      delay(500);
      ESP.restart();
    }
    return;
  }

// Проверяем состояние кнопки и начинаем отсчет времени, если кнопка нажата
  // if (digitalRead(buttonPin) == HIGH) {
  //   buttonPressed = true;
  //   buttonPressStartTime = millis();
  // } else {
  //    buttonPressed = false;
  //   }
//
//  // Цикл ожидания для проверки длительности нажатия // пересекает с пинов выхода из сна
//  while (buttonPressed) {
//    unsigned long currentTime = millis();
//
//    // Если кнопка удерживается более 5 секунд
//    if (currentTime - buttonPressStartTime > 5000) {
//      Serial.println("Button was pressed for more than 5 seconds. Formatting SPIFFS.");
//      SPIFFS.format();
//
//      delay(1000);
//      ESP.restart();
//      //buttonPressed = false; // Сброс состояния кнопки после форматирования
//    }
//    // Если кнопка отпущена до истечения 5 секунд
//    else if (digitalRead(buttonPin) == LOW) {
//      buttonPressed = false;  // Сброс состояния кнопки
//    }
//    delay(100);  
//  }
//  delay(500);
  
//===================================
WiFi.persistent(true);

loadControlFromSPIFFS();
setupControl();
//=====================================

  bot = new UniversalTelegramBot("9960429846:AAHsKFC6CDjV_F9pZ7c81Oppdm580CVv5F4", client);

  if (!loadSettings()) {
    Serial.println("Failed to load settings, applying default settings\n");

    initializeWiFiSettings(settings);
    saveSettings();

    startAccessPoint();
  } else if (settings.isWifiTurnedOn) {
    if (settings.isAP) {
      startAccessPoint(settings.ssidAP.c_str(), settings.passwordAP.c_str(), settings.ipAddressAP.toString().c_str());
    } else {
      if (!settings.networkSettings[idSetting].ssid.isEmpty()) {
        bool isSuccess = connectToWiFi(settings.networkSettings[idSetting].ssid.c_str(), settings.networkSettings[idSetting].password.c_str(), idSetting);
        delay(1000);
        if (!isSuccess) {
          connectToBestAvailableNetwork();
        }
      } else {
        startAccessPoint(settings.ssidAP.c_str(), settings.passwordAP.c_str(), settings.ipAddressAP.toString().c_str());
      }
    }
  }

  //========= ТЕЛЕГРАМ==============// перенести до соединения с  wifi

  if (!settings.telegramSettings.botId.isEmpty()) {
    changeBotId(settings.telegramSettings.botId);
  }
  if (isInternetConnected) {
    startTelegramMessage();
  }

  serverProcessingControl();
  serverProcessing();
  

  delay(1000); 
  //==========================

  ticker.attach(1.0, tikTak);

//    xTaskCreate(
//    controlTask,        // Функция задачи
//    "controlTask",      // Имя задачи
//    4096,          // Размер стека
//    NULL,           // Параметры задачи
//    1,              // Приоритет
//    NULL  // Дескриптор задачи (указатель на указатель)
//  );

  //=================================

  // Прерывание на кнопке
  //attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, LOW);  //RISING, FALLING, или CHANGE). 
} // END SETUP

//=======================

void handleSinglePress() {
  Serial.println("handleSinglePress\n");
}

void handleDoublePress() {
  if (WiFi.getMode() == WIFI_OFF) {

    settings.isAP = true;
    settings.isWifiTurnedOn = true;
    startAccessPoint(settings.ssidAP.c_str(), settings.passwordAP.c_str(), settings.ipAddressAP.toString().c_str());
    saveSettings();

  } else if (WiFi.getMode() == WIFI_AP) {
    settings.isAP = false;
    settings.isWifiTurnedOn = true;
    bool isConnect = connectToWiFi(settings.networkSettings[idSetting].ssid.c_str(), settings.networkSettings[idSetting].password.c_str(), idSetting);
    if (!isConnect) { connectToBestAvailableNetwork(); }
    saveSettings();

  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    settings.isAP = false;
    settings.isWifiTurnedOn = false;
    saveSettings();
  }

}

void handleLongPress5s() {

  if (isUpdate) {
    return;
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  
  ticker.detach();

  delay(1000);

  //enterSleepMode();
}

//========================

void loop() {
  delay(0);
  //esp_task_wdt_reset();
  
  if (settings.telegramSettings.isTelegramOn && isBotToken && isInternetConnected) {
    if (millis() - bot_lasttime > BOT_MTBS)
  {
    int numNewMessages = bot->getUpdates(bot->last_message_received + 1);

    while (numNewMessages)
    {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot->getUpdates(bot->last_message_received + 1);
    }
    updateStatus();
    sendPendingMessages();

    bot_lasttime = millis();
  }
}
//============================
  
 if(!isUpdate) {
    mainScenario(addLog);
 }

    if (isSaveControl) {
      saveControlToSPIFFS();
    }
  //============================================
//  wifi_mode_t currentWifiMode = WiFi.getMode();
//  wl_status_t currentWifiStatus = WiFi.status();
//
//  if (currentWifiMode != previousWifiMode || currentWifiStatus != previousWifiStatus) {
//    previousWifiMode = currentWifiMode;
//    previousWifiStatus = currentWifiStatus;
//
//    if (currentWifiStatus == WL_CONNECTED) {
//      analogWrite(ledPin, 255);  // Максимальная яркость
//    } else if (currentWifiMode == WIFI_AP) {
//      analogWrite(ledPin, 100);  // Яркость 100
//    } else if (currentWifiMode == WIFI_OFF) {
//      analogWrite(ledPin, 30);  // Яркость 30
//    }
//  }

  //============================================
  if (isScan) {
    scannedNetworks = scanNetworks(false);
  }
  //============ ОБРАБОТКА ДЕВАЙСОВ ============
  if (digitalRead(buttonPin) == LOW) {
    Serial.print("buttonPin ");
    Serial.println(digitalRead(buttonPin));
    delay(300);
    buttonPressed = false; 
    enterSleepMode();
    // Сброс флага прерывания
    //processButtonPress(handleSinglePress, handleDoublePress, handleLongPress5s);
  } 
  //===========================
  
  if (!isScan && tiktak1 > 120) {
        checkInternetConnection();
       tiktak1 = 0;
    }
   
  // каждые 30 секунд проверям подключение если не AP
  if (!isScan && tiktak3 > 30) {
    connectToBestAvailableNetwork();
    tiktak3 = 0;
  }

  if (!isScan && tiktak2 > 300) {
    isConnectionAttempts = true;
    tiktak2 = 0;

    connectToBestAvailableNetwork();
  }

  if (res && millis() > 60000) {
    delay(500);
    ESP.restart();
  } else {
    res = false;
  }

}  // END LOOP
