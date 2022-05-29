#include <SoftwareSerial.h>
/*
  Rui Santos
  Complete project details at Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-http-get-post-arduino/

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  
  Code compatible with ESP8266 Boards Version 3.0.0 or above 
  (see in Tools > Boards > Boards Manager > ESP8266)
*/

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

const char* ssid = "JinDamanee";
const char* password = "123456789";

/*
const char* ssid = "Tepsutboonyout_2G";
const char* password = "Platoo1234";
*/
/*
const char* ssid = "OPPOTontan";              //<----- WIFI change this 
const char* password = "tontan1234";
*/
//Your Domain name with URL path or IP address with path
const char* serverName = "https://led-api.herokuapp.com/sensor";
//https://led-api.herokuapp.com/sensor

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 1000;

SoftwareSerial NodeSerial(D2, D3); // RX | TX
SoftwareSerial NodeSerial2(D4, D5); // RX | TX

int RGB[3] ;
int a = 0 ;         //Lux value
int idx = 0 ;

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
 
  Serial.println("Connect Success");

  pinMode(D2, INPUT);
  pinMode(D3, OUTPUT);
  Serial.begin(115200);
  NodeSerial.begin(115200);
  
  pinMode(D4, INPUT);
  pinMode(D5, OUTPUT);
  NodeSerial2.begin(115200);
  
}

void loop() {

  //--------------Get data from Sensor-----------
  while (NodeSerial.available() > 0) {
    int c = NodeSerial.read();
    if (c > 0) {
      RGB[idx] = c ;
      Serial.println("RGB IN : " + String(RGB[idx]));
      idx += 1 ;
    }
    if (idx >= 3) {
      idx = 0 ;
    }
    //Serial.println(c, DEC);
  }

  String acc = "" ;
  bool isIn = false ;
  while (NodeSerial2.available() > 0) {
     char c = NodeSerial2.read();
     acc += c ;
     isIn = true ;
    //Serial.println(c, DEC);
  }
  if (isIn) {
     a = acc.toInt() ;
     Serial.println("LUX : "+ String(a));
  }
   
  //Send an HTTP POST request every 10 sec
  if ((millis() - lastTime) > timerDelay) {
    //Check WiFi connection status
    if(WiFi.status() == WL_CONNECTED){
      WiFiClientSecure client;
      HTTPClient http;

      client.setInsecure() ;
      
      // Your Domain name with URL path or IP address with path
      http.begin(client, serverName);

      // Specify content-type header
      
      http.addHeader("Content-Type", "application/json");
      // Data to send with HTTP POST
      int r = RGB[0] ;
      int g = RGB[1] ;
      int b = RGB[2] ;
      String rr =  String(r, HEX);
      String gg =  String(g, HEX);
      String bb =  String(b, HEX);
      if (rr.length() == 1) {
          rr = "0" + rr ;
      }
      if (gg.length() == 1) {
          gg = "0" + gg ;
      }
      if (bb.length() == 1) {
          bb = "0" + bb ;
      }
      String c = "#" + rr + gg + bb ;
      Serial.println("HEX :" + c);
       
      String httpRequestData = "{\"brightness\":" + String(a) + ",\"color\":\"" + c + "\"}" ;
      //String httpRequestData2 = "{\"brightness\":23,\"color\":\"#fcba03\"}";    
      Serial.println(httpRequestData) ; 
      //Serial.println(httpRequestData2) ;       
      // Send HTTP POST request
      
      int httpResponseCode = http.POST(httpRequestData);

      /*
      // If you need an HTTP request with a content type: application/json, use the following:
      //http.addHeader("Content-Type", "application/json");
      //int httpResponseCode = http.POST("{\"api_key\":\"tPmAT5Ab3j7F9\",\"sensor\":\"BME280\",\"value1\":\"24.25\",\"value2\":\"49.54\",\"value3\":\"1005.14\"}");

      // If you need an HTTP request with a content type: text/plain
      //http.addHeader("Content-Type", "text/plain");
      //int httpResponseCode = http.POST("Hello, World!");
     */
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
        
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}
