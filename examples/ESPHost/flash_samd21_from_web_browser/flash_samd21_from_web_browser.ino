/*
  Update your DAP from a browser. ESP8266 & ESP32
  Based on the webupdate example that comes with the ESP32 core and the flash from SD examples
  that come with the Adafruit_DAP library.
  
  This example let's you upload firmware to your DAP device directly from a browser.
  After compiling and uploading this example open your Serial Monitor and look for the following message:
  "Open http://esp-webdap.local or http://192.168.***.*** in your browser"
  
  If your system supports MDNS you can use the first address. Otherwise use the second. You will be greated
  by a file select field and a update button. Select your firmware and click select.
  
  The more advanced users can also upload through a terminal. Use: curl -F "update=@firmware.bin" esp-webdap.local/update
*/

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <WiFiClient.h>
  #include <ESP8266WebServer.h>
  #include <ESP8266mDNS.h>
  
  //WebServer
  ESP8266WebServer server(80);
#else if defined ESP32
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WebServer.h>
  #include <ESPmDNS.h>
  
  //WebServer
  WebServer server(80);
#endif

#include <Adafruit_DAP.h>


#define SWDIO 14
#define SWCLK 33
#define SWRST 26

const char* host      = "esp-webdap";
const char* ssid      = "........";
const char* password  = "........";

const int BUFSIZE     = Adafruit_DAP_SAM::PAGESIZE;
uint8_t buf[BUFSIZE];

unsigned long t  = 0; // timer
uint32_t addr    = 0; // current addr
uint32_t bufferd = 0; // currently bufferd

//create a DAP for programming Atmel SAM devices
Adafruit_DAP_SAM dap;


// Upload page used by the Webserver
const char* serverIndex = "<!DOCTYPE html><html><body><form method='POST' action='/update' enctype='multipart/form-data' onsubmit='fileCheck(event)'><input type='file' name='update' accept='.bin'><input type='submit' value='Update'></form><script>function fileCheck(e){document.getElementsByName('update')[0].files[0]||(alert('Select a file before clicking Update'),e.preventDefault())}</script></body></html>";


// Function called when there's an SWD error
void error(const char *text) {
  Serial.println(text);
  while (1);
}


void programDap(void) {
  dap.programBlock(addr, buf);
  addr += BUFSIZE;
  
  // reset buffer for the next round
  memset(buf, BUFSIZE, 0xFF);
  bufferd = 0;
}


void setup(void) {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting Sketch...");

  // Setup WiFi
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);

  // Wait for WiFi connection
  Serial.printf("Connecting to WiFi AP: %s", ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" done\n");

  // Start multicast DNS for host
  MDNS.begin(host);

  // Upload form
  server.on("/", HTTP_GET, []() {
    Serial.println("Serving upload form\n");
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  
  // Process received update
  server.on("/update", HTTP_POST, []() {
    // if this point is reached the upload to the ESP was successful
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "OK");
    
  }, []() {
    HTTPUpload& upload = server.upload();
    
    if (upload.status == UPLOAD_FILE_START) {
      Serial.println("Processing uploaded file\n");

      dap.begin(SWCLK, SWDIO, SWRST, &error);

      Serial.println("Connecting to DAP...");
      if (!dap.targetConnect()) {
        error(dap.error_message);
      }

      char debuggername[100];
      dap.dap_get_debugger_info(debuggername);
      Serial.println(debuggername);
    
      uint32_t dsu_did;
      if (!dap.select(&dsu_did)) {
        Serial.println("Unknown device found 0x"); Serial.println(dsu_did, HEX);
        error("Unknown device found");
      }

      Serial.print("\nFound Target\t: "); Serial.println(dap.target_device.name);
      Serial.print("Flash size\t: ");     Serial.println(dap.target_device.flash_size);
      Serial.print("Flash pages\t: ");    Serial.println(dap.target_device.n_pages);
      Serial.print("Update to\t: ");      Serial.println(upload.filename.c_str());

      Serial.print("\nErasing... ");
      dap.erase();
      Serial.println("done");
      
      Serial.print("Programming... ");
      t = millis();
      addr = dap.program_start();
      
    } else if (upload.status == UPLOAD_FILE_WRITE) {

      // fill DAP buffer with received data(upload) and program device
      for (unsigned int i=0; i < upload.currentSize; i++) {
        
        buf[bufferd] = upload.buf[i];
        bufferd++;
  
        // SAMD21 devices like to be updated 1 page(PAGESIZE) at a time
        // because we do not know the file size untill fully received
        // we will run this function once more in final for the last bytes(if any)
        if (bufferd == BUFSIZE) {
          programDap();
        }
      }

    // program last bytes(if any) and disconnect from DAP
    } else if (upload.status == UPLOAD_FILE_END) {
      
      if (bufferd > 0){
        programDap();
      }

      Serial.print("done\nFinished in "); Serial.print(millis() - t); Serial.println("ms");
      
      dap.dap_set_clock(50);
      dap.deselect();
      dap.dap_disconnect();
    }
  });


  Serial.print("Setting up server & MDNS... ");
  server.begin();
  MDNS.addService("http", "tcp", 80);

  Serial.printf("done\nOpen http://%s.local or http://", host);
  Serial.print(WiFi.localIP()); Serial.println(" in your browser\n");
}


void loop(void) {
  server.handleClient();
  #ifdef ESP8266
    MDNS.update();
  #endif
  delay(2);//allow the cpu to switch to other tasks
}
