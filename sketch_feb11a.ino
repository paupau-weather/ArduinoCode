#include <Adafruit_GFX.h>                    // Подключаем библиотеку Adafruit_GFX для работы с дисплеем
#include <Max72xxPanel.h>                    // Подключаем библиотеку Max72xxPanel для работы с дисплеем
#include <GyverBME280.h>                      // Подключение библиотеки для работы с датчиком BMP/BME280
#include <SPI.h>                             // Подключаем библиотеку SPI для работы с дисплеем
#include <SoftwareSerial.h>

// Перменные дисплея
int pinCS = 9;                               // Указываем к какому выводу подключен контакт CS
int numberOfHorizontalDisplays = 1;          // Количество матриц по горизонтали
int numberOfVerticalDisplays = 4;            // Количество матриц по-вертикали

// Переменные кнопок
const int button_MODE_PIN = 4;
const int button_MIN_PIN = 3;
const int button_HR_PIN = 2;
int buttonState_MODE = 0;
int buttonState_MIN = 0;
int buttonState_HR = 0;
bool lastModeButtonState = LOW;
bool lastMinButtonState = LOW;
bool lastHrButtonState = LOW;
unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// Переменные для времени
int hours = 0;
int minutes = 0;
int seconds = 0;
unsigned long timer;

// Переменные для двоеточия
const unsigned long BLINK_INTERVAL = 300;
bool colonVisible = true;
unsigned long lastBlinkTime = 0;

// Переменные для бегущей строки
int spacer = 1; // расстояние между буквами
int width = 5 + spacer; // размер шрифта

// Переменная id станции
const String ID_station = "000001";

// Переменные датчиков
float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;
float co2 = 0.0;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 5000;

// Перменные для запросов и ответов
const unsigned long sendInterval = 5000;      // 5 секунды
const unsigned long connectionInterval = 500; // 100 миллисекунд

// Создание объектов
SoftwareSerial espSerial(7,6);   // Создаем объект программного последовательного порта, Первый параметр - RX пин, второй - TX пин.
Max72xxPanel matrix = Max72xxPanel(pinCS, numberOfHorizontalDisplays, numberOfVerticalDisplays);  // Создаем объект дисплея
GyverBME280 bme;    // Создаём объект для датчика

void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

  pinMode(button_MIN_PIN, INPUT);
  pinMode(button_HR_PIN, INPUT); 
  pinMode(button_MODE_PIN, INPUT);

  bme.begin();               // Если доп. настройки не нужны  - инициализируем датчик
  matrix.setIntensity(2);    // Задаем яркость от 0 до 15
  matrix.setRotation(3); 

  pinMode(A0, INPUT); // режим работы аналогового пина
  ConnectESP(); // Подключение модуля к сети
}

void loop() {
  Time();
  Buttons();
  readSensors();
  Request(ID_station, temperature, humidity, pressure, co2);
}

void Time(){
  if (millis() - timer >= 1000) {
    timer = millis();
    seconds++;
    if(seconds > 59){
      seconds = 0;
      minutes++;
    }
    
    if(minutes > 59){
      minutes = 0;
      hours++;
    }
    
    if(hours > 23){
      hours = 0;
    }
    
    if (millis() - lastBlinkTime >= BLINK_INTERVAL){
      lastBlinkTime = millis();
      colonVisible = !colonVisible;
    }

    char buffer[20]; 
    sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);

    // Выводим время на последовательный порт
    Serial.println(buffer);

    matrix.fillScreen(LOW);
    matrix.drawChar(1, 0, buffer[0], HIGH, LOW, 1);
    matrix.drawChar(7, 0, buffer[1], HIGH, LOW, 1);

    if (colonVisible){
      matrix.drawChar(13, 0, buffer[2], HIGH, LOW, 1);
    }

    matrix.drawChar(19, 0, buffer[3], HIGH, LOW, 1);
    matrix.drawChar(25, 0, buffer[4], HIGH, LOW, 1);
    matrix.write();
  }
}

void Buttons(){
  bool buttonState_MIN = digitalRead(button_MIN_PIN);  
  bool buttonState_HR = digitalRead(button_HR_PIN); 
  bool buttonState_MODE = digitalRead(button_MODE_PIN);  
  
  // Обработка кнопки MODE
  if (buttonState_MODE != lastModeButtonState && millis() - lastButtonPress > DEBOUNCE_DELAY){
    lastButtonPress = millis();
    if (buttonState_MODE == HIGH){
      DisplayOutput();
    }
  }
  lastModeButtonState = buttonState_MODE;

  // Обработка кнопки минут
  if (buttonState_MIN != lastMinButtonState && millis() - lastButtonPress > DEBOUNCE_DELAY){
    lastButtonPress = millis();
    if (buttonState_MIN == HIGH){
      minutes++;
      seconds = 0;
      if (minutes > 59) minutes = 0;
    }
  }
  lastMinButtonState = buttonState_MIN;

  // Обработка кнопки часов
  if (buttonState_HR != lastHrButtonState && millis() - lastButtonPress > DEBOUNCE_DELAY){
    lastButtonPress = millis();
    if (buttonState_HR == HIGH){
      hours++;
      seconds = 0;
      if (hours > 23) hours = 0;
    }
  }
  lastHrButtonState = buttonState_HR;
}

void readSensors(){
  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure();
    pressure = pressureToMmHg(pressure);
    co2 = analogRead(A0) * 0.0001;
  }
}

void DisplayOutput(){
  char temperatureString[16]; // Создаем массив, в который далее переводим значение из переменной temperature в строку temperatureString
  dtostrf(temperature, 4, 2, temperatureString);
  String temp {"Temp - "};
  String Celsius {"C"};
  String Temperature =  temp + temperatureString + Celsius; // текст, который будет плыть

  char humidityString[16];  // Создаем массив, в который далее переводим значение из переменной Humidity в строку HumidityString
  dtostrf(humidity, 4, 2, humidityString);
  String hum {"Hum - "};
  String Percent {"%"};
  String humidity =  hum + humidityString + Percent; // текст, который будет плыть

  char pressureString[16];  // Создаем массив, в который далее переводим значение из переменной pressure в строку pressureString
  dtostrf(pressure, 4, 2, pressureString);
  String prs {"Prs - "};
  String PressureUnits_1 {"mm"};
  String PressureUnits_2 {"Hg"};
  String Pressure =  prs + pressureString + PressureUnits_1 + PressureUnits_2; // текст, который будет плыть

  char COstring[16]; // Создаем массив, в который далее переводм значение из переменной analogValue в строку COstring
  dtostrf(co2, 4, 3, COstring);
  String co {"CO2 - "};
  String CO = co + COstring + Percent;   // Текст, который будет плыть

  int wait = 60; // время между крайними перемещениями букв

  for ( int i = 0 ; i < width * Temperature.length() + matrix.width() - 1 - spacer; i++ ) {

    matrix.fillScreen(LOW);

    int letter = i / width;
    int x = (matrix.width() - 1) - i % width;
    int y = (matrix.height() - 8) / 2; // center the text vertically

    while ( x + width - spacer >= 0 && letter >= 0 ) {
      if ( letter < Temperature.length() ) {
        matrix.drawChar(x, y, Temperature[letter], HIGH, LOW, 1);
        }
      letter--;
      x -= width;
    }
    matrix.write();
    delay(wait);
  } 

  for ( int i = 0 ; i < width * Pressure.length() + matrix.width() - 1 - spacer; i++ ) {
    matrix.fillScreen(LOW);

    int letter = i / width;
    int x = (matrix.width() - 1) - i % width;
    int y = (matrix.height() - 8) / 2; // center the text vertically

    while ( x + width - spacer >= 0 && letter >= 0 ) {
      if ( letter < Pressure.length() ) {
         matrix.drawChar(x, y, Pressure[letter], HIGH, LOW, 1);
        }    
        letter--;
        x -= width;
    }
    matrix.write();
    delay(wait);
  }

  for ( int i = 0 ; i < width * humidity.length() + matrix.width() - 1 - spacer; i++ ) {
    matrix.fillScreen(LOW);

    int letter = i / width;
    int x = (matrix.width() - 1) - i % width;
    int y = (matrix.height() - 8) / 2; // center the text vertically

    while ( x + width - spacer >= 0 && letter >= 0 ) {
      if ( letter < humidity.length() ) {
        matrix.drawChar(x, y, humidity[letter], HIGH, LOW, 1);
        }
      letter--;
      x -= width;
    }
    matrix.write();
    delay(wait);
  }

  for ( int i = 0 ; i < width * CO.length() + matrix.width() - 1 - spacer; i++ ) {
    matrix.fillScreen(LOW);

    int letter = i / width;
    int x = (matrix.width() - 1) - i % width;
    int y = (matrix.height() - 8) / 2; // center the text vertically

    while ( x + width - spacer >= 0 && letter >= 0 ) {
      if ( letter < CO.length() ) {
        matrix.drawChar(x, y, CO[letter], HIGH, LOW, 1);
        }
      letter--;
      x -= width;
    }
    matrix.write();
    delay(wait);
  }
}

// Функция подключения Wi-Fi модуля к сети
void ConnectESP() {
  // Отправка данных на ESP-01
  espSerial.println("AT");  //Проверяем соединение с модулем
  Response();
  delay(2000);
  espSerial.println("AT+CWMODE=1"); //Переводим модуль в режим клиента
  Response();
  delay(2000);
  espSerial.println("AT+CWJAP=\"TP-Link_9F83\",\"Aleksandr03\""); //Присоединяемся к сети Wi-Fi
  delay(10000); //Задержка для подключения
  Response();
  delay(2000);
  espSerial.println("AT+CWJAP?"); //Проверка подключения модуля к сети
  Response();
  delay(2000);
  espSerial.println("AT+CIPSTART=\"TCP\",\"192.168.0.100\",8080");  //Подключение модуля к серверу через протокол TCP
  delay(5000);
  Response();
  delay(2000);
}

//Функция позволяющая отслеживать отправку АТ-команд и получать ответ от модуля
void Response() {
  static unsigned long lastResponseTime = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastResponseTime >= connectionInterval) {
    while (espSerial.available() > 0) {
      String incomingString = espSerial.readString();
      Serial.println(incomingString);
    }
  }
  lastResponseTime = currentMillis;
}


// Функция отправляющая запрос на сервер 
void Request(String ID_station, float temperature, float Humidity, float pressure, float analogValue){
  static unsigned long lastRequestTime = 0;
  unsigned long currentMillis = millis();
  
  String ESP_Request = "id_station=" + ID_station + ",temperature=" + String(temperature) + ",humidity=" + String(Humidity) + ",pressure=" + String(pressure) + ",CO2=" + String(analogValue);
  if (currentMillis - lastRequestTime >= sendInterval) {
    espSerial.println("AT+CIPSEND=" + String(ESP_Request.length()));
    Response();
    
    espSerial.println(ESP_Request);
    Response();
    lastRequestTime = currentMillis;
  }
}
