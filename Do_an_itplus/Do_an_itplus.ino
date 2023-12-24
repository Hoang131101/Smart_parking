#include <WiFiClientSecure.h>
#include <ESP8266WiFi.h>

const char* ssid = "Redmi Note 11 Pro+ 5G";
const char* pass = "1234567890";

const char* host = "script.google.com";
const int httpsPort = 443;

WiFiClientSecure client;

String GAS_ID = "AKfycbzEGDA0DVxLoMmpLxyLiptnsIe5eUAVpkPsjodgtYWrDQLe8HWQfqqa5y3WRdSLRu6_5Q";//Nhap spreadsheet script ID
char rdata;//receive data
String mystring;
String dt0;
int dt1;

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.begin(ssid, pass);
  Serial.println();

  Serial.print("Connecting...");
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Successfully connected to : ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  client.setInsecure();

}

String getvalue(String data, char separator, int index){
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1; 
  
  for(int i = 0;i <= maxIndex && found <= index; i++){
    if(data.charAt(i) == separator || i==maxIndex){
      found++;
      strIndex[0] = strIndex[1]+1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void sendata(String id, int state){
  Serial.println("=========");
  Serial.print("connecting to: ");
  Serial.println(host);

  if(!client.connect(host, httpsPort)){
    Serial.println("connection Failed");
    return;
  }
  String url;
  if(state == 1){
    url = "/macros/s/" + GAS_ID + "/exec?ID=" + id + "&state=IN";
  } else if(state == 2){
    url = "/macros/s/" + GAS_ID + "/exec?ID=" + id + "&state=OUT";
  }

  Serial.print("Requesting URL: ");
  Serial.println(url);

  client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "User-Agent: BuildFailureDetectorESP8266\r\n"+"Connection: close\r\n\r\n");

  Serial.println("request sent");

  while(client.connected()){
    String line = client.readStringUntil('\n');
    if(line == "\r"){
      Serial.println("header received");
      break;
    }
  }
  String line = client.readStringUntil('\n');
  if(line.startsWith("{\"state\":\"success\"")){
    Serial.println("esp8266/Arduino CI successfull!");
  } else {
    Serial.println("esp8266/Arduino CI has failed");
  }
  Serial.print("reply was : ");
  Serial.println(line);
  Serial.println("closing connection");
  Serial.println("=====================");
  Serial.println();
}

void loop() {
  if(Serial.available() > 0){
    rdata = Serial.read();
    mystring = mystring + rdata;
    if(rdata == '\n'){
      String a = getvalue(mystring, ',', 0);
      String b = getvalue(mystring, ',', 1);

      dt0 = a;
      dt1 = b.toInt();
    }
  }
  sendata(dt0,dt1);

}
