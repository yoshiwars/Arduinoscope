#include <esp_now.h>
#include <WiFi.h>

/************************************************************************************************************************
Start Configurable Items 
*************************************************************************************************************************/
//Sender
uint8_t telescopeAddress[] = {0xec, 0x94, 0xcb, 0x6d, 0x0d, 0xcc};
//#define DEBUG_COMMS

/************************************************************************************************************************
End Configurable Items
*************************************************************************************************************************/

/************************************************************************************************************************
Global Variables - These should not need to change based on devices
*************************************************************************************************************************/
const byte numChars = 32;

char input[numChars];

int retries = 0;

esp_now_peer_info_t peerInfo;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  char message[numChars];  
} struct_message;

// Create a struct_message called myData
struct_message espNowOut;
struct_message espNowIn;

bool newData = false;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, telescopeAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0){
    communication(Serial.read());
  }  
}


void communication(byte inByte)
{
  
  static unsigned int input_pos = 0;    
  switch(inByte){
    /*case 6:
      
      break;
      */
    case '\r':   // discard line ends
      break;
    case '\n':
      break;
     case '#':
      
      //terminator reached! process newdata
      if(input_pos > 0){
        newData = true;
      }else{
        break;
      }
      
      //reset buffer for next time
    default:
     // keep adding if not full ... allow for terminating null byte
    if (input_pos < (numChars - 1))
      input[input_pos++] = inByte;
    break;
  }
  
  if(newData){
    
    input[input_pos] = 0; //terminating null byte
    input_pos = 0; 
    strcpy(espNowOut.message, input);

    esp_now_send(telescopeAddress, (uint8_t *) &espNowOut, sizeof(espNowOut));
    for(int i = 0; i < numChars; i++){
      input[i] = '\0';
    }
    newData = false;
  }else if(input_pos == numChars){
    
    #ifdef DEBUG_COMMS
      Serial.print("Input Buffer Overrun: ");
      Serial.print(input);
    #endif
    for(int i = 0; i < numChars; i++){
      input[i] = '\0';
    }
  }
}

// Callback when data is sent through ESPNow
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #ifdef DEBUG_COMMS
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  #endif

  if(status == ESP_NOW_SEND_SUCCESS){
    for(int i = 0; i < numChars; i++){
      espNowOut.message[i] = '\0';
    }
    retries = 0;
  }else{
    
    if(retries < 3){
      esp_now_send(telescopeAddress, (uint8_t *) &espNowOut, sizeof(espNowOut));
    }
    retries++;
  }
  
}

// Callback when data is received through ESPNow
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&espNowIn, incomingData, sizeof(espNowIn));
  #ifdef DEBUG_COMMS
    Serial.println(len);
  #endif
  
  
  Serial.print(espNowIn.message);
  

  for(int i = 0; i < numChars; i++){
      espNowIn.message[i] = '\0';
    }
}