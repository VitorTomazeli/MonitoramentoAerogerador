/*============================================================================
PROJETO: SISTEMA DE MONITORAMENTO DE GRANDEZAS ELÉTRICAS E MECÂNICAS DE UM AEROGERADOR DE PEQUENO PORTE
AUTOR 1: VITOR MATEUS TOMAZELI
AUTOR 2: GUILHERME HENRIQUE DA MAIA
ORIENTADOR: CARLOS EDUARDO VIANA
CO-ORIENTADOR: HENRIQUE DE SOUZA MEDEIROS
MODELO ESP32: ESP-WROOM-32-DEVKIT1
IDE: Arduino IDE 2.1.1
============================================================================*/

//============================================================================
// DECLARAÇÃO BIBLIOTECAS                                                                        
//============================================================================
#include <Adafruit_BME280.h>          // Adafruit BME280 by Adafruit         v2.2.2  -> https://github.com/adafruit/Adafruit_BME280_Library
#include <Adafruit_INA219.h>          // Adafruit INA219 by Adafruit         v1.2.1  -> https://github.com/adafruit/Adafruit_INA219
#include <Adafruit_Sensor.h>          // Adafruit Unified Sensor by Adafruit v1.1.12 -> https://github.com/adafruit/Adafruit_Sensor
#include <arduinoFFT.h>               // arduinoFFT by Enrique Condes        v1.6.1  -> https://github.com/kosme/arduinoFFT
#include <Arduino.h>                  // Biblioteca Nativa 
#include <HTTPClient.h>               // Biblioteca Nativa
#include <WiFi.h>                     // Biblioteca Nativa
#include <Wire.h>                     // Biblioteca Nativa 
//============================================================================

//============================================================================
// DECLARAÇÃO VARIÁVEIS GLOBAIS                                                                     
//============================================================================
                                                                   
//_________________GRANDEZAS_AMBIENTE_________________//                                               -
Adafruit_BME280 bme;                  
float temperatura, pressao, umidade;  

//_____________________ANEMOMETRO_____________________//
const float pi = 3.14159265;       
int periodo = 5000;                   
int amostragem = 2000;                
float RaioAnemometro = 7.5;           // em cm
int tempo1 = 0;                       
int tempo2 = 0;                       
unsigned int contador1 = 0;           
unsigned int contador2 = 0;          
unsigned int RPM_jusante = 0;          
unsigned int RPM_montante = 0;            
float velocidade_montante = 0;        
float velocidade_jusante = 0;         

//_________________GRANDEZAS_ELETRICAS_________________//
float tensao_shunt = 0;
float tensao_barramento = 0;
float corrente = 0;
float tensao = 0;
float potencia = 0;
Adafruit_INA219 ina219;

//_______________________VIBRACAO______________________//
#define AMOSTRAS	1024                // Deve ser uma potencia de 2 (max = 4096)
#define FREQ_AMOSTRAGEM 1024          // em Hz
unsigned int periodo_amostragem_us;
double freq_fundamental;
unsigned int indice_freq_pico1;
double freq_pico1;
double amplitude_pico1;
unsigned int indice_freq_pico2;
double freq_pico2;
double amplitude_pico2;
unsigned int indice_freq_pico3;
double freq_pico3;
double amplitude_pico3;
unsigned int indice_freq_pico4;
double freq_pico4;
double amplitude_pico4;
unsigned int indice_freq_pico5;
double freq_pico5;
double amplitude_pico5;
unsigned int indice_freq_pico6;
double freq_pico6;
double amplitude_pico6;
double freq_vibracao;
double dados_real[AMOSTRAS];
double dados_imag[AMOSTRAS];
unsigned long tempo_corrido;
arduinoFFT FFT = arduinoFFT(dados_real,dados_imag,AMOSTRAS,FREQ_AMOSTRAGEM); // OBJETO FFT DA CLASSE arduinoFFT

//________________________WIFI________________________//
const char* ssid = "raspiAP";                                       // Login wifi
const char* password = "raspberry";                                 // Senha wifi
const char* serverName = "http://10.0.0.220/aerogeradorDataVG.php"; // PHP Bando de Dados
String apiKeyValue = "tPmAT5Ab3j7F9";                               // Chave Banco de Dados
int httpResponseCode;                                               // Verificar variaveis enviadas

//______________________LORATASK______________________//
TaskHandle_t loRaTask; // CORE0 - loRaTask
//============================================================================

//============================================================================
// FUNÇÃO -> void setup                                                                       
//============================================================================
void setup() {

  btStop();       // Desliga o Bluetooth
  
  //______________________LORATASK______________________//
  vTaskDelay(100);
  xTaskCreatePinnedToCore(
               zerotaskcode,	// Function to implement the task  
               "loRaTask",		// Name of the task  
               10000,			    // Stack size in words   
               NULL,			   //Task input parameter  
               10,  // 0,		// Priority of the task  
               &loRaTask,		// Task handle.  
               0);				  //Core where the task should run  
  vTaskDelay(1);
  
  //_________________GRANDEZAS_AMBIENTE_________________//
  bool status = bme.begin(0x76);  //Define endereço na I2C
  if (!status) {
    Serial.println("Não foi possível encontrar um sensor BME280 válido. Verifique a fiação, o endereço e a ID do sensor!");
    Serial.print("SensorID era: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("   ID de 0xFF provavelmente significa um endereço incorreto, um BMP 180 ou BMP 085\n");
    Serial.print("   ID de 0x56-0x58 representa BMP 280,\n");
    Serial.print("   ID de 0x60 representa um BME 280.\n");
    Serial.print("   ID de 0x61 representa um BME 680.\n");
    while (1) delay(10);
  }

  //_______________________SERIAL_______________________//
  Serial.begin(115200);
  Serial.println("Serial Ok");

  //_____________________ANEMOMETRO_____________________// 
  pinMode(14, INPUT);  // Configura o digital 14 como entrada velocidade a jusante
  pinMode(25, INPUT);  // Configura o digital 25 como entrada velocidade a montande 

  //_________________GRANDEZAS_ELETRICAS________________// 
  if (! ina219.begin()) {
    Serial.println("Falha ao encontrar o chip INA219");
    while (1) { delay(10); }
  }
  
  //______________________VIBRACAO______________________// 
  char print_buf[300];
  //==========Calcula periodo de amostragem========//
  periodo_amostragem_us = round(1000000.0 / FREQ_AMOSTRAGEM);
  freq_fundamental = (double)FREQ_AMOSTRAGEM/ (double)AMOSTRAS;

  //________________________WIFI______________________// 
  WiFi.begin(ssid, password);  // Conecta wifi
  Serial.println("Conectando");

  while (WiFi.status() != WL_CONNECTED) { // Aguarda conexão wifi
    delay(500);  // Enquanto não conectar, fica tentando.....
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Conectado à rede WiFi com endereço IP: ");
  Serial.println(WiFi.localIP());  // Wifi conectado + mostra IP
} 
//============================================================================

//============================================================================
// FUNÇÃO -> void anemometro                                                              
//============================================================================
void anemometro() {  //Rotina para o anemometro

  attachInterrupt(14, interrupcao1, RISING);  // Habilita interrupção em borda de subida no pino 14
  attachInterrupt(25, interrupcao2, RISING);  // Habilita interrupção em borda de subida no pino 25
  if ((millis() - tempo1) > periodo) {
    velocidade_montante = 0;
    velocidade_jusante = 0;
    RPM_jusante = ((contador1) / (periodo / 1000));                         // Calcula rotações por minuto
    velocidade_jusante = ((2 * pi * RaioAnemometro * RPM_jusante))/100;     // Calcula a velocidade do vento em m/s
    RPM_montante = ((contador2) / (periodo / 1000));                          // Calcula rotações por minuto
    velocidade_montante = ((2 * pi * RaioAnemometro * RPM_jusante))/100;     // Calcula a velocidade do vento em m/s
    contador1 = 0;
    contador2 = 0;
    tempo1 = millis();
  }
}
//============================================================================

//============================================================================
// FUNÇÃO -> void interrupcao1                                                                  
//============================================================================
void interrupcao1() { // Rotina para a interrupção do anemometro
  contador1++;
}
//============================================================================

//============================================================================
// FUNÇÃO -> void interrupcao2                                                                 
//============================================================================
void interrupcao2() { // Rotina para a interrupção do anemometro
  contador2++;
}
//============================================================================

//============================================================================
// FUNÇÃO -> void grandezas_eletricas                                                                  
//============================================================================
void grandezas_eletricas() { // Rotina para o sensor de corrente
  tensao_shunt = ina219.getShuntVoltage_mV();
  tensao_barramento = ina219.getBusVoltage_V();
  corrente = ina219.getCurrent_mA() / 1000;
  potencia = ina219.getPower_mW() / 1000;
  tensao = tensao_barramento + (tensao_shunt / 1000);
}
//============================================================================

//============================================================================
// FUNÇÃO -> void grandezas_ambiente                                                                  
//============================================================================
void grandezas_ambiente() {               // Rotina para a leitura de pressão, temperatura e umidade
  temperatura = bme.readTemperature();    // Definindo a variavel da temperatura
  pressao = bme.readPressure() / 100.0F;  // Definindo a variavel de pressao
  umidade = bme.readHumidity();           // Definindo a variavel de umidade
}
//============================================================================

//============================================================================
// FUNÇÃO -> void vibracao                                                                  
//============================================================================
void vibracao() {
  
//______________________Reseta os valores de pico________________________// 
amplitude_pico1 = 0;
indice_freq_pico1 = 0;
amplitude_pico2 = 0;
indice_freq_pico2 = 0;
amplitude_pico3 = 0;
indice_freq_pico3 = 0;
amplitude_pico4 = 0;
indice_freq_pico4 = 0;
amplitude_pico5 = 0;
indice_freq_pico5 = 0;
amplitude_pico6 = 0;
indice_freq_pico6 = 0;

//____________________Coleta um conjunto de amostras_____________________// 
for (int i = 0; i < AMOSTRAS; i++){
	tempo_corrido = micros();

	dados_real[i] = analogRead(34) * 3300 / 4095;
	dados_imag[i] = 0;
	while ((micros() - tempo_corrido) < periodo_amostragem_us) { } // Aguarda momento de fazer a amostragem
}

//___________________________Calcula a FFT_______________________________// 
FFT.DCRemoval (); //remove nível DC
FFT.Compute(FFT_FORWARD); // Realiza o calculo usando metodo forward (normal)
FFT.ComplexToMagnitude(); // Converte o numero complexo para amplitude real (descartando o angulo de fase)

//______________________Analisa os dados da FFT__________________________// 
for(int i = 2; i < (AMOSTRAS/2); i++){

  if(dados_real[i] > amplitude_pico1){            // Detecta o indice da frequencia de pico
		indice_freq_pico1 = i;
		amplitude_pico1 = dados_real[i];
	}
  else if(dados_real[i] > amplitude_pico2){
		indice_freq_pico2 = i;
		amplitude_pico2 = dados_real[i];
	}
  else if(dados_real[i] > amplitude_pico3){
		indice_freq_pico3 = i;
		amplitude_pico3 = dados_real[i];
	}
  else if(dados_real[i] > amplitude_pico4){
		indice_freq_pico4 = i;
		amplitude_pico4 = dados_real[i];
	}
  else if(dados_real[i] > amplitude_pico5){
		indice_freq_pico5 = i;
		amplitude_pico5 = dados_real[i];
	}
   else if(dados_real[i] > amplitude_pico6){
		indice_freq_pico6 = i;
		amplitude_pico6 = dados_real[i];
	}

	freq_pico1 = (indice_freq_pico1)*freq_fundamental; //calcula a frequencia de pico 1 
  freq_pico2 = (indice_freq_pico2)*freq_fundamental; //calcula a frequencia de pico 2
  freq_pico3 = (indice_freq_pico3)*freq_fundamental; //calcula a frequencia de pico 3
  freq_pico4 = (indice_freq_pico4)*freq_fundamental; //calcula a frequencia de pico 4
  freq_pico5 = (indice_freq_pico5)*freq_fundamental; //calcula a frequencia de pico 5
  freq_pico6 = (indice_freq_pico6)*freq_fundamental; //calcula a frequencia de pico 6

  freq_vibracao = sqrt (pow(freq_pico1, 2) + pow(freq_pico2, 2) + pow(freq_pico3, 2) + pow(freq_pico4, 2) + pow(freq_pico5, 2) + pow(freq_pico6, 2));
  
  amplitude_pico1 = 0;
  indice_freq_pico1 = 0;
  amplitude_pico2 = 0;
  indice_freq_pico2 = 0;
  amplitude_pico3 = 0;
  indice_freq_pico3 = 0;
  amplitude_pico4 = 0;
  indice_freq_pico4 = 0;
  amplitude_pico5 = 0;
  indice_freq_pico5 = 0;
  amplitude_pico6 = 0;
  indice_freq_pico6 = 0;

  vTaskDelay(1);
}
}
//============================================================================

//============================================================================
// FUNÇÃO -> void loop     (CORE 1)                                                                  
//============================================================================
void loop() {                 // Código a ser executado no CORE 1

  anemometro();               // Rotina para leitura anemômetro
  grandezas_eletricas();      // Rotina para leitura sensor de corrente
  grandezas_ambiente();       // Rotina para leitura Sensor temperatura pressão umidade

  if ((millis() - tempo2) > amostragem) {
    Serial.println("***************MEDIÇÃO DAS VARIÁVEIS******************* ");
    Serial.print("Vel. Vento Antes:");
    Serial.print(velocidade_montante);  // Imprime valor da velocidade a montante na Serial
    Serial.println("[m/s]");

    Serial.print("Vel. Vento Depois:");
    Serial.print(velocidade_jusante);  // Imprime valor da velocidade a jusante na Serial
    Serial.println("[m/s]");
    Serial.println();

    Serial.print("Tensão Barramento:   "); Serial.print(tensao_barramento); Serial.println(" V");
    Serial.print("Tensão Shunt:        "); Serial.print(tensao_shunt); Serial.println(" mV");
    Serial.print("Tensão Carga:        "); Serial.print(tensao); Serial.println(" V");
    Serial.print("Corrente:            "); Serial.print(corrente); Serial.println(" A");
    Serial.print("Potência:            "); Serial.print(potencia); Serial.println(" W");
    Serial.println("");

    Serial.print("Pressão Atmosférica: ");
    Serial.print(pressao, 1);      // Imprime valor da pressão na Serial
    Serial.println("hPa");

    Serial.print("Temperatura Ambiente: ");
    Serial.print(temperatura, 1);  // Imprime valor da temperatura na Serial
    Serial.println("°C");

    Serial.print("Umidade Relativa do Ar: ");
    Serial.print(umidade, 1);     // Imprime valor da umidade na Serial
    Serial.println("%");

    Serial.print("Frequência Vibração:   "); Serial.print(freq_vibracao); Serial.println(" Hz");

    if (WiFi.status() == WL_CONNECTED) {  // Se o wifi estiver conectado
      WiFiClient client;
      HTTPClient http;
      http.begin(client, serverName);     
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      String httpRequestData = "api_key=" + apiKeyValue
                               + "&potencia=" + potencia
                               + "&tensao=" + tensao
                               + "&corrente=" + corrente
                               + "&pressao=" + pressao
                               + "&umidade=" + umidade
                               + "&temperatura=" + temperatura
                               + "&velocidade_jusante=" + velocidade_jusante
                               + "&velocidade_montante=" + velocidade_montante
                               + "&vibracao=" + freq_vibracao;

      int httpResponseCode = http.POST(httpRequestData);  // Tenta salvar os dados no banco de dados
      
      if (httpResponseCode > 0) {  // Apresenta mensagem se salvou
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        Serial.println("Dados salvos com sucesso");
      } else {  //Mostra o n do alarme se não salvou
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
        Serial.println("Falha ao salvar");
      }

      http.end();  // Finaliza conexão
    } else {
      Serial.println("WiFi Disconnected"); 
    }

    tempo2 = millis();  // Tempo para envio ao servidor e impressão serial
  }
}
//============================================================================

//============================================================================
// FUNÇÃO -> void zerotaskcode     (CORE 0)                                                                
//============================================================================
void zerotaskcode(void * pvParameters){   // Código a ser executado no CORE 0
  for(;;) {
vibracao(); // Rotina para leitura vibração
  } 
  vTaskDelay(1);
}