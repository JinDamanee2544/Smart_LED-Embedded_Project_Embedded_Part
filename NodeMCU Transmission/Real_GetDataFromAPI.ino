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
#include <Arduino_JSON.h>


const char* ssid = "JinDamanee";                                 //<------Change Wifi here
const char* password = "123456789";

/*
const char* ssid = "Tepsutboonyout_2G";                                 //<------Change Wifi here
const char* password = "Platoo1234";
*/
/*
const char* ssid = "OPPOTontan";              //<----- WIFI change this 
const char* password = "tontan1234";
*/
//Your Domain name with URL path or IP address with path
const char* serverName = "https://led-api.herokuapp.com/led";           //<------Server name
//https://led-api.herokuapp.com/led
//http://jsonplaceholder.typicode.com/users/1

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
unsigned long timerDelay = 500;

SoftwareSerial NodeSerial(D2, D3); // RX | TX
SoftwareSerial NodeSerial2(D4, D5); // RX | TX

String sensorReadings;

//--------------Data for Send to LED--------------
int lux;
String color="";
char arr[7] ;
int sender [4];
//------------------------------------------------

void setup() {
  Serial.begin(115200);

  pinMode(D2, INPUT);
  pinMode(D3, OUTPUT);
  NodeSerial.begin(115200);

  pinMode(D4, INPUT);
  pinMode(D5, OUTPUT);
  NodeSerial2.begin(115200);

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
}

void loop() {
  // Send an HTTP POST request depending on timerDelay
  if ((millis() - lastTime) > timerDelay) {
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
              
      sensorReadings = httpGETRequest(serverName);
      Serial.println(sensorReadings);
      JSONVar myObject = JSON.parse(sensorReadings);
  
      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
    
      //Serial.print("JSON object = ");
      //Serial.println(myObject);
    
      // myObject.keys() can be used to get an array of all the keys in the object
      JSONVar keys = myObject.keys();
      lux = myObject[keys[0]] ;
      //color = String(myObject[keys[1]]) ;
      color = JSON.stringify(myObject[keys[1]]);
      
      Serial.println(myObject[keys[0]]);
      Serial.println(color);

      /*
      for (int i = 0; i < 7 ; i++) {
        arr[i] = color[i] ;
        Serial.println(arr[i]) ;
        NodeSerial.write(arr[i]) ;
      }
      //NodeSerial.write(arr, 7) ;
      NodeSerial2.write(lux) ;
      */
      /*
        String tmp = "" ;
           for (int i = 1; i < 7; i++) {
              tmp += color[i] ;
              
           }
           Serial.println("TMP : " + tmp);
      
      // Get rid of '#' and convert it to integer
       long number = (long) strtol( &tmp[1], NULL, 16);

     // Split them up into r, g, b values
       
       int r = number >> 16;
       int g = number >> 8 & 0xFF;
       int b = number & 0xFF;
       

       
       Serial.println("r : " + String(r));
       Serial.println("g : " + String(g));
       Serial.println("b : " + String(b));

       long rgbL = (r * pow(10, 6)) + (g * pow(10, 3)) + (b * pow(10, 0)) ;
      
       Serial.println(rgbL) ;
       */
        // Format : int 90 -> String "0090"
      String luxString="";    // Convert to my format
        if(lux==100){
           luxString = "-100";
        } else if(lux<100){
           luxString = "--" + String(lux);
        } else if(lux<10){  
           luxString = "---" + String(lux);
        } else if(lux==0){
           luxString = "----";
        }
        Serial.println("luxStr : " + luxString) ;
        Serial.println("luxInt : " + String(lux));

        // SEND BY UART (String)
        Serial.println("Color : " + color);
      NodeSerial.print(color) ;
      NodeSerial2.print(luxString) ;
       
      /*
      String colorString = "#ff04ee";
      String luxNEW = "0789";
      Serial.println(colorString);
      Serial.println(luxNEW);
      NodeSerial.print(colorString);      
      NodeSerial2.print(luxNEW);
      */
      
      
     
      /*
      for (int i = 0; i < keys.length(); i++) {             //<----Print what I get
        JSONVar value = myObject[keys[i]];
        Serial.print(keys[i]);
        Serial.print(" = ");
        Serial.println(value);
      }
      */
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}

String httpGETRequest(const char* serverName) {
  WiFiClientSecure client;
  HTTPClient http;

  client.setInsecure() ;
  //client.connect(serverName, httpsport) ;
    
  // Your IP address with path or Domain name with URL path 
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}
