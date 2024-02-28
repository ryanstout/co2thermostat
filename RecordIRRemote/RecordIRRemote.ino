#include <IRremote.h>

const int RECV_PIN = 11; // IR Receiver pin
const unsigned long MIN_SIGNAL_LENGTH = 2000; // Minimum length of IR signal in microseconds
const unsigned long DEBOUNCE_TIME = 500; // Debounce time in milliseconds
const int TOLERANCE_PERCENT = 25; // Tolerance percentage

IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  //irrecv.setTolerance(TOLERANCE_PERCENT); // Set the tolerance for decoding
  Serial.println("IR Receiver with Filter and Tolerance");
}

void loop() {
  if (irrecv.decode(&results)) {
    unsigned long currentTime = millis();

    // Debounce
    if (currentTime - lastTime > DEBOUNCE_TIME) {
      // Check if signal length is greater than minimum threshold
      if (results.rawlen * USECPERTICK > MIN_SIGNAL_LENGTH) {
        Serial.println("Received IR:");

        // Print the type of the protocol
        switch (results.decode_type) {
          case NEC: Serial.println("Protocol: NEC"); break;
          case SONY: Serial.println("Protocol: SONY"); break;
          case RC5: Serial.println("Protocol: RC5"); break;
          case RC6: Serial.println("Protocol: RC6"); break;
          case UNKNOWN: Serial.println("Protocol: UNKNOWN"); break;
          default: Serial.println("Protocol: Other"); break;
        }

        // Print raw data
        Serial.print("Raw data: ");
        for (int i = 0; i < results.rawlen; i++) {
          Serial.print(results.rawbuf[i]*USECPERTICK, DEC);
          if (i < results.rawlen-1) Serial.print(",");
        }
        Serial.println();

        lastTime = currentTime;
      }
    }
    irrecv.resume(); // Receive the next value
  }
}