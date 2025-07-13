// UNO_modbus_simple_chaining.ino
// NonBlockingModbusMaster multiple reads on UNO using software serial
//AltSoftSerial on Arduino Uno       TX pin 9        RX pin 8        PWM used pin 10

/**
  (c)2025 Forward Computing and Control Pty. Ltd.
  NSW Australia, www.forward.com.au
  This code is not warranted to be fit for any purpose. You may only use it at your own risk.
  This code may be freely used for both private and commercial use
  Provide this copyright is maintained.
  Also see LICENSE.txt for original ModbusMaster license.
*/

#include "AltSoftSerial.h"
#include "NonBlockingModbusMaster.h"
#include "loopTimer.h"
#include "millisDelay.h"

// list of addresses to read and array to place readings in
#define NUM_REGISTERS 16
int registerAddresses[] = {0, 2, 7, 19, 25, 30, 31, 40, 41, 50, 51, 60, 61, 97, 98, 99};
uint16_t data[NUM_REGISTERS];
size_t addressIdx = 0; // keep track of which address to read
 
AltSoftSerial altSerial;
const long baudrate = 9600;
NonBlockingModbusMaster nbModbusMaster;
millisDelay samplingDelay;
unsigned long sampleInterval_ms = 3000; // timeout is 2000

bool readAddr();
bool processAddr();

const int MAX_RETRIES = 1; // retry once if timeout

unsigned long start_ms = 0;

void setup() {
  Serial.begin(115200);
  for (int i = 10; i > 0; i--) {
    Serial.print(i); Serial.print(' ');
    delay(500);
  }
  Serial.println();

  Serial.println("NonBlockingModbusMaster on UNO using AltSoftSerial ");
  Serial.println("  reading Slave ID: 1, Holding Registers listed in registerAddresses[]");
  Serial.println("  store results in data[] and then finish." );
  altSerial.begin(baudrate);

  // MODBUS over serial line specification and implementation guide V1.02
  // Paragraph 2.5.1.1 MODBUS Message RTU Framing
  // https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
  float bitduration = 1.f / baudrate;
  float charlen = 10.0f; // 8 bits + 1 stop, parity ?
  float preDelayBR = bitduration * charlen * 3.5f * 1e6  + 1; // in us
  float postDelayBR = bitduration * charlen * 3.5f * 1e6 + 1; // in us
  // ~3.28ms (4ms) for 9600 baud, or 0.84ms (1ms) for 34800 baud

  nbModbusMaster.initialize(altSerial, preDelayBR, postDelayBR); // default timeout 2000ms (2sec)
  samplingDelay.start(sampleInterval_ms);
}

void processAddr(NonBlockingModbusMaster &mb) {
  static int retryCount = 0;
  Serial.print(" in processData() for addressIdx:"); Serial.println(addressIdx);
  // check for errors
  int err = mb.getError(); // 0 for OK
  if (err) {
    Serial.print("Error: "); mb.printHex(err, Serial); Serial.println();
    if ((err == nbModbusMaster.ku8MBResponseTimedOut) && (retryCount < MAX_RETRIES)) {
      retryCount++;
      Serial.println(" Retry");
      mb.retry(); // send same cmd again
    }
  } else {
    retryCount = 0; // success
  }
  if (!err) {
    data[addressIdx] = mb.getResponseBuffer(0);
    Serial.print("response Len "); Serial.print(mb.getResponseBufferLength()); Serial.print("  ");
    for (size_t i = 0; i < mb.getResponseBufferLength(); i++) {
      mb.printHex(mb.getResponseBuffer(i), Serial); Serial.print(" ");
    }
    Serial.println();
  }
  // else err continue or not?? can stop here by not calling readAddr_2()
  addressIdx++;
  if (addressIdx < NUM_REGISTERS) { // read next address
    //mb.oneTimeDelay(10); // add delay here if needed.
    // but in this example could just increase preDelayBR instead
    readAddr(); 
  } 
  // else finish here
}


bool readAddr() {
  return nbModbusMaster.readHoldingRegisters(1, registerAddresses[addressIdx], 1, processAddr); // add processing fn to save results and continue
}


void loop() {
  // loopTimer.check(Serial);

  if (samplingDelay.justFinished()) {
    samplingDelay.restart();
    if (!nbModbusMaster.isProcessing()) {
      Serial.println(" ++ start block reads ");
      addressIdx = 0;
      readAddr();
    } else {
      Serial.println(" Still processing skip this restart");
    }
  }

  // this will return false and be ignored if nbModbusMaster is still processing the last cmd
  // i.e. if nbModbusMaster.isProcessing() returns true
  if (nbModbusMaster.justFinished()) {
    // all reads finished
    Serial.println(" -- Finished all reads.");
  }

}
