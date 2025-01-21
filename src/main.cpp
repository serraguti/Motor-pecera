#include <Arduino.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <U8g2lib.h>
#include <Adafruit_BMP280.h>
#include "logo.h"
#include <ArduinoOTA.h>

const int pinMotor1 = 17;  // Pin que controla el sentido de giro Motor A
const int pinMotor2 = 16;  // Pin que controla el sentido de giro Motor A
const int pulsadorPinStop = 19; //Pin que controla el pulsador
int valorPulsadorStop = 0;  //Variable para capturar la pulsación
int motorIniciado = 0;

int movimiento = 0;
const int pinLED = 22;
//CONFIGURACION DEL WIFI
const char* ssid = "DIGIFIBRA_2G";
const char* password = "SerraTormo14";
//CONFIGURACION DE MQTT
const char* mqttServer = "192.168.1.157";
const int mqttPort = 1883;
const char* mqttUser = "serragutimqtt";
const char* mqttPassword = "serrañhome";

//WIFI
// Set your Static IP address
IPAddress local_IP(192, 168, 1, 105);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 255, 0);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

String ultimacomida = " 07/01/2025 19:47 ";

String ultimocambio = " 03/12/2024 10:14 ";

void ponerLogotipo(){
    u8g2.setDrawColor(1);
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_ncenB14_tr);
      u8g2.drawXBMP(0,0, super_width, super_height, super_bits);
    } while ( u8g2.nextPage() );
    // esperar un poco en la pantalla inicial
    delay(10000);
    u8g2.clear();
}

//NOS SUBSCRIBIMOS A UN TOPIC 
void SuscribeMqtt()
{
    mqttClient.subscribe("pecera");
    mqttClient.subscribe("pecera/ultimacomida");
    mqttClient.subscribe("pecera/ultimalimpieza");
}

void StopMotor(){
  //delay(500);
  Serial.print("Parando motor...");
  Serial.print("Movimiento deteniendo...");
  digitalWrite(pinMotor1, LOW);  // PARA
  digitalWrite(pinMotor2, LOW);
  motorIniciado = 0;
}

void IniciarMotor(){
  Serial.print("Movimiento comenzando...");
  digitalWrite(pinMotor1, LOW);  // GIRO DERECHA
  digitalWrite(pinMotor2, HIGH);
  delay(5000);
  motorIniciado = 1;
  // valorPulsador = digitalRead(pulsadorPin);
  //   if (valorPulsador == HIGH) {
  //   //AQUI ES DONDE VAMOS A PARAR EL MOTOR
  //     Serial.print("Boton pulsado, deteniendo motor");
  //     //StopMotor();
  //     //mqttClient.publish("pecera","stop");
  // }
  //AQUI PONEMOS UN DELAY PARA ESPERAR UN POCO HASTA ABRIR EL 
  //COMEDERO
  //2 SEGUNDOS POR EJEMPLO...
  //delay(2000);
  //StopMotor();
}


void dibujarDatosPantalla(){
  //u8g2.clear();
  // inciar el cíclo de dibujo de la pantalla
  //u8g2.firstPage();
  u8g2.clearBuffer();
  
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_7x13B_mr);
  
  u8g2.setCursor(0,16);
  //u8g2.sendBuffer();
  //u8g2.drawStr(5,35,"tecnotizate.es");
  u8g2.print("Ultima comida:");
  u8g2.setCursor(5,30);  
  u8g2.setDrawColor(0);
  u8g2.print(ultimacomida);
  u8g2.setDrawColor(1);
  u8g2.setCursor(0,44);
  u8g2.print("Ultima limpieza:");
  u8g2.setDrawColor(0);
  u8g2.setCursor(5,60);
  u8g2.print(ultimocambio);
  u8g2.sendBuffer();
}
// callback to execute when a message is received
// in this example, shows the received message by serial
//SE SUPONE QUE ES CUANDO RECIBIMOS UN MENSAJE DE MQTT
void OnMqttReceived(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Received on ");
    Serial.print(topic);
    Serial.print(": ");

    String content = "";
    for (size_t i = 0; i < length; i++)
    {
        content.concat((char)payload[i]);
    }
    Serial.print(content);
    Serial.println();
    //String tema = "";
    String tema(topic); 
    Serial.print("Tema: ");
    Serial.print(tema);
    Serial.println();
    if (tema == "pecera/ultimacomida"){
      //DIBUJAMOS EN PANTALLA??
      ultimacomida = " " + content + " ";
      Serial.print(ultimacomida);
    }else if (tema == "pecera/ultimalimpieza"){
      //DIBUJAMOS EN PANTALLA??
      ultimocambio = " " + content + " ";
    }else if (tema == "pecera"){
      if (content == "on"){
          motorIniciado = 0;
          IniciarMotor();
          Serial.print("Iniciando motor");
          mqttClient.publish("bicho","iniciando");
          //delay(10000);      
          //mqttClient.publish("pecera","stop");
        }else if (content == "off"){
          //movimiento = 0;
          Serial.print("Deteniendo motor");
          mqttClient.publish("bicho","deteniendo");
          StopMotor();
        }
    }       
 
}

void encenderLed(){
  digitalWrite(pinLED, HIGH); // turn the LED on (HIGH is the voltage level)
}

void apagarLed(){
  digitalWrite(pinLED, LOW);  // turn the LED off by making the voltage LOW
}

// starts the MQTT communication
// starts setting the server and the callback when receiving a message
void InitMqtt()
{
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(OnMqttReceived);
    //mqttClient.publish("pecera","conectado mqtt");
}

void InitOTA()
{
  ArduinoOTA.setHostname("ESP32-pecera");
  ArduinoOTA.onStart([]()
  {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_FS
        type = "filesystem";
      }

      Serial.println("Iniciando Programación " + type); 
  });
  ArduinoOTA.onEnd([]()
  { 
    Serial.println("\nTerminando"); 
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  { 
    Serial.printf("Progreso: %u%%\r", (progress / (total / 100))); 
  });
  ArduinoOTA.onError([](ota_error_t error)
  {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      } 
  });

  ArduinoOTA.begin();
  delay(12000);  
}

// connects or reconnects to MQTT
// connects -> subscribes to topic and publishes a message
// no -> waits 5 seconds
void ConnectMqtt()
{
    Serial.print("Starting MQTT connection...");
    while (!mqttClient.connected())
    {      
      Serial.println("Connecting to MQTT...");
      if (mqttClient.connect("ESP32Client", mqttUser, mqttPassword )){
        SuscribeMqtt();
        Serial.println("Connected to MQTT");
        //apagarLed();
        mqttClient.publish("bicho","conectado!!!");
        mqttClient.publish("bicho","conectado mqtt");
      }else
      {   
        Serial.print("failed with state ");
        Serial.print(mqttClient.state());
        delay(2000);
      }
    }    
}

// manages MQTT communication
// checks if the client is connected
// no -> tries to reconnect
// yes -> calls the MQTT loop
void HandleMqtt()
{
    if (!mqttClient.connected())
    {
        ConnectMqtt();
    }
    mqttClient.loop();
}

void setup() {
  Serial.begin(9600);
  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  pinMode(pinLED, OUTPUT);
  //INICIAMOS LA CONFIGURACION DEL MOTOR
  pinMode(pinMotor1, OUTPUT);    // Configura  los pines como salida
  pinMode(pinMotor2, OUTPUT);
  pinMode(pulsadorPinStop, INPUT); //configuramos el pulsador
  //mqttClient.publish("pecera","Todo configurado");
  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Algo falla en el Wifi del motor...");
  }  
  WiFi.begin(ssid, password);
  Serial.println("...................................");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  //encenderLed();
  //mqttClient.publish("pecera","conectado al Wifi");
  InitMqtt();
  InitOTA();
  //CONFIGURAMOS LA PANTALLA
  u8g2.begin(); //Inicializamos el dispositivo
  //u8g2.drawStr(5,35,"tecnotizate.es");
  
  ponerLogotipo();
  //mqttClient.publish("pecera","ready");
  delay(12000);
}

// the loop routine runs over and over again forever:
void loop() {
  ArduinoOTA.handle();
  //encenderLed();
  HandleMqtt();
    valorPulsadorStop = digitalRead(pulsadorPinStop);
    if (valorPulsadorStop == HIGH) {
      //encenderLed();
      //PONEMOS UNA VARIABLE PARA COMPROBAR SI HA INICIADO O NO
      if (motorIniciado == 1){
        //AQUI ES DONDE VAMOS A PARAR EL MOTOR
        Serial.print("Boton pulsado, deteniendo motor");
        mqttClient.publish("pecera","off");
        motorIniciado = 0;
        //apagarLed();
      }
      //StopMotor();
    }
    dibujarDatosPantalla();  
  // digitalWrite(pinMotor1, LOW);  // GIRO DERECHA
  // digitalWrite(pinMotor2, HIGH);
}