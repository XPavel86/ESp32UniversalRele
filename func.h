#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <SPIFFS.h>
#include <FS.h>
#include <ESP32Time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//===============
bool isUpdate = false;

String statusUpdate = "";
String previousStatus = "";
bool statusUpdateChanged = false;

//================

String getFileNameFromUrl(String url) {
    int index = url.lastIndexOf('/');
    if (index != -1) {
        return url.substring(index + 1);
    }
    return "";
}

void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *fileData, size_t len, bool final) {
 // deleteAllFilesExceptHtmlJson() ;
    static File file;
    static bool isUpdateFlash = false;

    // Определение расширения файла
    String extension = filename.substring(filename.lastIndexOf('.') + 1);

    if (!index) {  // начало нового файла
        if (extension != "bin") {
            String path = "/" + filename;
            file = SPIFFS.open(path, FILE_WRITE);
            if (!file) {
                Serial.println("Failed to open file for writing");
                return;
            }
        } else if (extension == "bin") {
            isUpdateFlash = true;
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {  // начать обновление
                Update.printError(Serial);
                return;  
            }
        }
    }

    if (len) {  // есть данные для записи
        if (extension != "bin") {
            file.write(fileData, len);
        } else if (extension == "bin" && isUpdateFlash) {
            if (Update.write(fileData, len) != len) {
                Update.printError(Serial);
                 return;  
            }
        }
    }

    if (final) {  // последний пакет данных
        if (extension != "bin") {
            file.close();
        } else if (extension == "bin" && isUpdateFlash) {
            if (Update.end(true)) {  // завершить обновление
                Serial.println("Update Complete");
                ESP.restart();  // перезагрузка для применения обновления
            } else {
                Update.printError(Serial);
                 return; 
            }
        }
        Serial.println("File Upload Complete");
    }
     isUpdate = false;
}

//===================================================

void downloadAndUpdateFirmware(String url, String fileName) {
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure();  // Использовать небезопасный клиент (для тестирования HTTPS)

    http.setTimeout(60000); // Увеличение таймаута до 60 секунд
    http.begin(client, url);

    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
        Serial.printf("HTTP Response code: %d\n", httpResponseCode);

        int actualFileSize = http.getSize();
        Serial.printf("Expected file size: %d\n", actualFileSize);

        if (actualFileSize > 0) {
            String filename = getFileNameFromUrl(url);

            if (filename.endsWith(".bin")) {
                // Начать процесс обновления прошивки
                statusUpdate = "Starting firmware update... ~1 min";

                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    Update.printError(Serial);
                    http.end();
                    return;
                }

                WiFiClient *stream = http.getStreamPtr();
                uint8_t buff[128];
                int bytesRead = 0;

                while (http.connected() && (bytesRead < actualFileSize)) {
                    size_t size = stream->available();
                    if (size) {
                        int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
                        if (Update.write(buff, c) != c) {
                            Serial.println("Error: Write failed");
                            Update.printError(Serial);
                            http.end();
                            return;
                        }
                        bytesRead += c;
                        //Serial.printf("Read %d bytes, total %d bytes\n", c, bytesRead);
                    } else {
                        delay(10); // Пауза для предотвращения быстрого опроса
                    }
                }

                if (Update.end(true)) {
                    statusUpdate = "Update Complete";
                    Serial.println(statusUpdate); //
                    delay(300); //
                    ESP.restart();  // Перезагрузка для применения обновления
                } else {
                    statusUpdate = "Error: Update failed";
                    Update.printError(Serial);
                }
            } else {
                // Сохранение других файлов в SPIFFS с именем из fileName
 
                File file = SPIFFS.open("/" + fileName, FILE_WRITE);
                if (!file) {
                    statusUpdate = "Failed to open file for writing";
                    http.end();
                    return;
                }

                WiFiClient *stream = http.getStreamPtr();
                uint8_t buff[128];
                int bytesRead = 0;

                while (http.connected() && (bytesRead < actualFileSize)) {
                    size_t size = stream->available();
                    if (size) {
                        int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
                        file.write(buff, c);
                        bytesRead += c;
                        //Serial.printf("Read %d bytes, total %d bytes\n", c, bytesRead);
                    } else {
                        delay(10); // Пауза для предотвращения быстрого опроса
                    }
                }

                file.close();
                statusUpdate = "File saved as /" + fileName + " in SPIFFS";
            }
        } else {
            Serial.println("Error: Incorrect file size");
        }
    } else {
        Serial.printf("HTTP Error code: %s\n", http.errorToString(httpResponseCode).c_str());
        statusUpdate = "HTTP Error code: " + http.errorToString(httpResponseCode);
    }

    http.end();
    
    isUpdate = false;
}

//==============================================
String listFiles() {
    String fileList;

    // Начало работы с SPIFFS
    if (!SPIFFS.begin(true)) {
        fileList += "Failed to mount SPIFFS\n";
        return fileList;
    }

    File root = SPIFFS.open("/");
    if (!root) {
        fileList += "Failed to open root directory\n";
        return fileList;
    }

    if (!root.isDirectory()) {
        fileList += "Root is not a directory\n";
        return fileList;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            fileList += "Dir: ";
        } else {
            fileList += "File: ";
        }
        fileList += String(file.name());
        fileList += " Size: ";

        // Вычисляем размер в килобайтах с двумя десятичными знаками
        float fileSizeKB = file.size() / 1024.0;
        fileList += String(fileSizeKB, 2); // Два десятичных знака
        fileList += " KB";
        fileList += "\n";
        
        file = root.openNextFile();
    }

    return fileList;
}


//================================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <title>Kolibri Web Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
  
      body {
        font-family: Arial, sans-serif;
        background-color: #E1EAF0;
        color: #333;
        margin: 0;
        padding: 20px;
    }

    h1,
    h2 {
        color: #444;
    }
    
    #current-value-label {
    transition: top 0.5s, left 0.5s; /* Плавное перемещение лейбла */
  }

    button {
        font-size: 13px;
        padding: 5px 10px;
        /* Отступы внутри кнопки */
        border: 1px solid gray;
        border-radius: 10px;
        background-color: rgba(173, 216, 230, 0.5);
        color: black;
        box-shadow: 0 4px 6px rgba(206, 183, 183, 0.1);
        transition: all 0.3s ease;
        /* Плавное изменение стилей при наведении */
        cursor: pointer;
    }
  
    #downloadLink {
      display: none;
      margin-top: 10px;
    }
    #copyMessage {
      display: none;
      margin-top: 10px;
      color: green;
    }
  </style>
</head>
<body>
  <h1>Kolibri Web Server</h1>
  
  <h3 id="statusHeader">Обновить прошивку</h3>
  
  <button id="uploadButton">Открыть файл</button>
  <button id="downloadButton">Скачать прошивку</button>
  
  <div id="downloadLink">
    <a href="https://cloud.mail.ru/public/KtJ5/WgxbfXTrP" target="_blank">Скачать прошивку</a>
  </div>
  <div id="copyMessage">Ссылка скопирована. Подключитесь к интернету и скачайте файлы.</div>

  <input type="file" id="fileInput" style="display:none" onchange="uploadFile()">
  
  <script>
    document.getElementById('uploadButton').addEventListener('click', function() {
        document.getElementById('fileInput').click(); // Имитация клика по input
    });

    document.getElementById('downloadButton').addEventListener('click', function() {
        var downloadLink = document.getElementById('downloadLink');
        var copyMessage = document.getElementById('copyMessage');
        downloadLink.style.display = 'block';
        copyMessage.style.display = 'block';

        // Копирование ссылки в буфер обмена
        var dummy = document.createElement('input');
        document.body.appendChild(dummy);
        dummy.setAttribute('value', downloadLink.getElementsByTagName('a')[0].href);
        dummy.select();
        document.execCommand('copy');
        document.body.removeChild(dummy);
    });

    function uploadFile() {
        var statusHeader = document.getElementById('statusHeader');
        statusHeader.textContent = "Загрузка файла..."; // Статус перед началом загрузки

        var fileInput = document.getElementById('fileInput');
        var file = fileInput.files[0];
        var formData = new FormData();
        formData.append('file', file);

        var fileName = file.name;
        var extension = fileName.substring(fileName.lastIndexOf('.') + 1);

        if (extension === "bin") {
            var countdown = 5;
            statusHeader.textContent = `Обновление... ${countdown} секунд осталось`; // Статус перед успешной загрузкой и начала обновления
            var interval = setInterval(function() {
                countdown--;
                if (countdown > 0) {
                    statusHeader.textContent = `Обновление... ${countdown} секунд осталось`;
                } else {
                    clearInterval(interval);
                    statusHeader.textContent = "Переподключитесь к устройству"; // Сообщение о переподключении
                    setTimeout(function() {
                       checkServerAndReload();
                    }, 5000); 
                }
            }, 1000); // Таймер на 1 секунду
        }

        fetch('/uploadFile', {
            method: 'POST',
            body: formData
        })
        .then(response => {
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            return response.text();
        })
        .then(data => {
            if (extension !== "bin") {
                statusHeader.textContent = "Успешно"; // Статус после успешной загрузки
                location.reload();
            }
        })
        .catch((error) => {
            console.error('Ошибка:', error);
            statusHeader.textContent = "Ошибка загрузки"; // Статус при ошибке загрузки
        });
    }

 function checkServerAndReload() {
        fetch('/', { method: 'HEAD' })
            .then(response => {
                if (response.ok) {
                    location.reload();
                } else {
                    console.error('Сервер недоступен');
                }
            })
            .catch(error => {
                console.error('Ошибка подключения к серверу:', error);
            });
 }

 async function getStatus() {
      try {
        const response = await fetch('/sysStatus'); // Отправляем GET запрос к серверу
        if (!response.ok) throw new Error('Network response was not ok');
        const status = await response.text(); // Получаем текст ответа
        showModal(status); // Отображаем статус в алерте
        // Или можно использовать модальное окно вместо алерта
        // showModal(status); 
      } catch (error) {
        console.error('Ошибка:', error);
        alert('Не удалось получить статус системы.');
      }
    }

    // Функция для отображения статуса в модальном окне
    function showModal(status) {
  // Преобразование символов переноса строки в теги <br>
  const htmlStatus = status.replace(/\n/g, '<br>');

  const modal = document.createElement('div');
  modal.style.position = 'fixed';
  modal.style.top = '50%';
  modal.style.left = '50%';
  modal.style.transform = 'translate(-50%, -50%)';
  modal.style.backgroundColor = 'white';
  modal.style.border = '1px solid black';
  modal.style.padding = '20px';
  modal.style.zIndex = '1000';

  const text = document.createElement('p');
  text.innerHTML = htmlStatus;  // Используем innerHTML для вставки HTML
  modal.appendChild(text);

  const closeButton = document.createElement('button');
  closeButton.textContent = 'Закрыть';
  closeButton.onclick = () => document.body.removeChild(modal);
  modal.appendChild(closeButton);

  document.body.appendChild(modal);
}
  </script>
</body>
</html>
)rawliteral";


//===========================================
  void printRequestParameters(AsyncWebServerRequest* request) {
     Serial.println("Params:");
    int params = request->params();  // Получаем количество параметров запроса
    for (int i = 0; i < params; i++) {
      AsyncWebParameter* p = request->getParam(i);  // Получаем параметр по индексу
      if (p->isPost()) {
        Serial.print("POST[");
      } else {
        Serial.print("GET[");
      }
      Serial.print(p->name());  // Выводим имя параметра
      Serial.print("]: ");
      Serial.println(p->value());  // Выводим значение параметра
    }
  }

//===========================================

void handleRoot(AsyncWebServerRequest *request) {
    if (SPIFFS.exists("/index.html")) {
        request->send(SPIFFS, "/index.html", "text/html");
    } else {
        request->send_P(200, "text/html", index_html);
    }
}
//=====================

String checkNewToken(const String& token) {
  HTTPClient http;
  String url = "https://api.telegram.org/bot" + token + "/getMe";

  http.begin(url);
  int httpResponseCode = http.GET();

  if (httpResponseCode == 200) { // Успешный ответ
    String response = http.getString();
    http.end();

    // Обработка JSON ответа
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);
    if (error) {
      Serial.println("Failed to parse JSON: " + String(error.c_str()));
      return "";
    }

    String botName = doc["result"]["username"].as<String>();
    return botName;

  } else {
    Serial.println("Invalid token: " + String(httpResponseCode));
    http.end();
    return "";
  }
}
//=========

bool botSetCommands(String botToken, const String& commands) {
 HTTPClient http;
  String url = "https://api.telegram.org/bot" + String(botToken) + "/setMyCommands";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  // Использование StaticJsonDocument для создания корректного JSON
  StaticJsonDocument<1024> doc;
  deserializeJson(doc, "{\"commands\":" + commands + "}");
  String postData;
  serializeJson(doc, postData);

  int httpResponseCode = http.POST(postData);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println("Response: " + response);
    
    // Проверка, была ли установка успешной
    DynamicJsonDocument responseDoc(1024);
    DeserializationError error = deserializeJson(responseDoc, response);
    if (error) {
      Serial.println("Failed to parse JSON: " + String(error.c_str()));
      return false;
    }
    
    bool success = responseDoc["ok"];
    return success;
  } else {
    Serial.println("Error on sending POST: " + String(httpResponseCode));
    return false;
  }

  http.end();
}
//========

bool deleteBotCommands(String botToken) {
  HTTPClient http;
  String url = "https://api.telegram.org/bot" + String(botToken) + "/deleteMyCommands";

  http.begin(url);
  int httpResponseCode = http.POST("");

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println("Response: " + response);
  } else {
    Serial.println("Error on sending POST: " + String(httpResponseCode));
  }

  http.end();
  return httpResponseCode == 200;
}
//======================================

struct TaskParameters {
  String filePath;  // Полный путь к файлу
  String fileName;  // Имя файла
};

void otaUpdateTask(void * parameter) {
  TaskParameters* params = (TaskParameters*) parameter;
  
  // Используйте params->filePath и params->fileName в вашей функции обновления
  Serial.println("Starting firmware update from file: " + params->filePath);
  Serial.println("File name: " + params->fileName);
  
  // Передаем оба параметра в функцию обновления
  downloadAndUpdateFirmware(params->filePath, params->fileName);

  // Освобождение выделенной памяти
  delete params;  // Освобождение памяти, выделенной с помощью new

  vTaskDelete(NULL);  // Завершение задачи
}

//=========================

// Функция для получения состояния системы
String getSystemStatus() {
  String status = "";

  uint32_t cpuFreq = getCpuFrequencyMhz();  // Частота процессора в MHz
  status += "Частота процессора: ";
  status += cpuFreq;
  status += " MHz\n";

  status += "Объем ОЗУ: ";
  status += 512;
  status += " KB\n";

  status += "Свободная память ОЗУ: ";
  status += esp_get_free_heap_size() / 1024;
  status += " KB\n";

  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  status += "Общий объем SPIFFS: " + String(totalBytes / 1024) + " KB\n";
  status += "Используемый объем SPIFFS: " + String(usedBytes / 1024) + " KB\n";
  status += "Свободный объем SPIFFS: " + String((totalBytes - usedBytes) / 1024) + " KB\n";

  status += "\n";

if (WiFi.getMode() == WIFI_AP) {
    status += "Режим: Точка доступа\n";
    status += "SSID точки доступа: " + WiFi.softAPSSID() + "\n";
    status += "IP адрес точки доступа: " + WiFi.softAPIP().toString() + "\n";
    status += "Число подключенных станций: " + String(WiFi.softAPgetStationNum()) + "\n";
  } else if (WiFi.getMode() == WIFI_STA) {
    status += "Режим: Клиент\n";
    if (WiFi.isConnected()) {
      status += "SSID сети: " + WiFi.SSID() + "\n";
      status += "IP адрес: " + WiFi.localIP().toString() + "\n";
      status += "Шлюз: " + WiFi.gatewayIP().toString() + "\n";
      status += "Маска подсети: " + WiFi.subnetMask().toString() + "\n";
      status += "DNS1: " + WiFi.dnsIP(0).toString() + "\n";
      status += "Доступ из локальной сети: http://" + WiFi.localIP().toString() + "\n";
    } else {
      status += "Нет активного подключения\n";
    }
  } else {
    status += "Режим WiFi не определен\n";
  }
  
  status += "\n";
  status += "Kolibri home v1.3\n";
  status += "Dolgopolov Pavel, 2024\n";
  return status;
}

//=================================================================
