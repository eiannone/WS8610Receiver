/*
  WS8610Receiver - Arduino library for decoding RF 433Mhz signals of Lacrosse wheater stations sensors
  Tested with the following transimitter models: TX3-TH, TX4 and TX7U

  Based on RCSwitch library, by Suat Özgür (https://github.com/sui77/rc-switch/)
  
  OVERVIEW OF MAIN LOGIC 
  (by @Joetgithub https://github.com/Joetgithub/TX7U/blob/master/TX7UReceiver.ino)
  
  Continously loads pulses into a rolling buffer that is sized to hold one temp
  or humidity reading. About every 57 seconds the TX4 and TX7 sensors send a
  data transmission consisting of a 44 bit temperature sequence followed by a
  repeat of that same 44 bit temperature sequence followed by a 44 bit humidity
  sequence.  A relativly long delay occurs after each temp/humidity 44 bit sequence
  and that delay is used as the trigger to evaluate the contents of the buffer and
  obtain the data values from the respective sequence.
  A pulse is the time in microseconds between changes in the data pin. The pulses
  have to follow a rule of being a LONG or SHORT followed by FIXED.
  A TOLERANCE value specifies the allowable range from the hard coded LONG, SHORT
  and FIXED pulse times. A 1 bit is a SHORT followed by a FIXED.  A 0 bit is a LONG
  followed by a SHORT.
  Here is example of a 10101. Note the variations in the SHORT, LONG and FIXED times.
            SHORT  LONG SHORT  LONG  LONG
             505   1300  595   1275  1395
             ┌-┐  ┌---┐  ┌-┐  ┌---┐  ┌-┐
             |1|  | 0 |  |1|  | 0 |  |1|
           --┘ └--┘   └--┘ └--┘   └--┘ └-
  FIXED        980    1030 1105   950
  Example showing two timings needed for each pulse
          t2          t4
           \           \              t2-t1=pulse1  FIXED
            ┌----┐     ┌----┐         t3-t2=pulse2  LONG or SHORT
            |    |     |    |     |   t4-t3=pulse3  FIXED
        ----┘    └-----┘    └-----┘   t5-t4=pulse4  LONG or SHORT
       /         /          /
      t1        t3         t5
  Because two timings are needed for each bit a total of 88 pulses are needed
  to decode the 44 bits.
  The pulses are converted into bits and stored in a six byte array as follows:
  [0]00001010 [1]11101110 [2]11110100 [3]10010000 [4]01001001 [5]1111
                   |   \      / |  \      |   |       |   |       |
     00001010    1110  1110111  1  0100  1001 0000   0100 1001   1111
  bits: 8         4     4+3=7   1    4    4    4       4    4      4
  key: (1)       (2)     (3)   (4)  (5)  (6)  (7)     (8)  (9)   (10)
      header    sensor sensor parity 10s  1s  10ths   10s  10th  check
                 type    ID    bit                                sum
  key: 1) Start Sequence is always 0x0A
       2) sensor 0000 = temp; 1110 = humidity
       3) sensor id
       4) parity bit
       5) ones
       6) tens
       7) tenths (for temp)
       8) repeat ones
       9) repeat tens
       10) checksum
 http://www.f6fbb.org/domo/sensors/tx3_th.php  
*/

#ifndef WS8610Receiver_h
#define WS8610Receiver_h

#define PW_FIXED 1000 // Pulse width for the "fixed" part of signal
#define PW_SHORT 550  // Pulse width for the "short" part of signal
#define PW_LONG 1350  // Pulse width for the "long" part of signal
#define PW_TOLERANCE 100

#define TIMINGS_BUFFER_SIZE 88
#define PACKET_SIZE 6

#ifdef ESP8266
    // interrupt handler and related code must be in RAM on ESP8266
    #define RECEIVE_ATTR ICACHE_RAM_ATTR
#else
    #define RECEIVE_ATTR
#endif

enum measureType {TEMPERATURE, HUMIDITY};

struct measure {
    uint8_t sensorAddr;
    measureType type;
    int units;
    uint8_t decimals;
};

class WS8610Receiver {
    public:
        WS8610Receiver(const int pin);

        void enableReceive();
        void disableReceive();
        bool available();
        void resetAvailable();
        measure getReceivedValue();

    private:
        static void handleInterrupt();
        static decodeBit(const uint32_t pulse1, const uint32_t pulse2);
        static uint8_t parity(const uint8_t b1, const uint8_t b2);
        static void decodeTimings(int timingsPos);

        int interrupt;
        static volatile measure receivedValue;
        static volatile uint32_t timingsBuf[TIMINGS_BUFFER_SIZE];
};

volatile measure WS8610Receiver::receivedValue;
volatile uint32_t WS8610Receiver::timingsBuf[TIMINGS_BUFFER_SIZE];

// Board                               Digital Pins Usable For Interrupts
// Uno, Nano, Mini, other 328-based    2, 3
// Mega, Mega2560, MegaADK             2, 3, 18, 19, 20, 21
// Micro, Leonardo, other 32u4-based   0, 1, 2, 3, 7
// Zero                                all digital pins, except 4
// MKR1000 Rev.1                       0, 1, 4, 5, 6, 7, 8, 9, A1, A2
// Due                                 all digital pins
WS8610Receiver::WS8610Receiver(const int pin) {
#ifdef ESP8266
    this->interrupt = pin;
#else
    this->interrupt = digitalPinToInterrupt(pin);
#endif
}


/**
 * Enable receiving data
 */
void WS8610Receiver::enableReceive() {
    WS8610Receiver::receivedValue = { .sensorAddr = 0};
    attachInterrupt(this->interrupt, handleInterrupt, CHANGE);
}

/**
 * Disable receiving data
 */
void WS8610Receiver::disableReceive() {
    detachInterrupt(this->interrupt);
}

bool WS8610Receiver::available() {
    return WS8610Receiver::receivedValue.sensorAddr != 0;
}

void WS8610Receiver::resetAvailable() {
    WS8610Receiver::receivedValue.sensorAddr = 0;
}

measure WS8610Receiver::getReceivedValue() {    
    return {
        WS8610Receiver::receivedValue.sensorAddr,
        WS8610Receiver::receivedValue.type,
        WS8610Receiver::receivedValue.units,
        WS8610Receiver::receivedValue.decimals
    }; 
}

int WS8610Receiver::decodeBit(const uint32_t pulse1, const uint32_t pulse2) {
    // Check second pulse (fixed width)
    uint32_t pw_diff = (pulse2 > PW_FIXED)? (pulse2 - PW_FIXED) : (PW_FIXED - pulse2);
    if (pw_diff > PW_TOLERANCE) return -1;
    
    // Check first pulse (long or short)
    if (pulse1 < PW_SHORT) return ((PW_SHORT - pulse1) < PW_TOLERANCE)? 1 : -1;
    if (pulse1 > PW_LONG) return ((pulse1 - PW_LONG) < PW_TOLERANCE)? 0 : -1;
    // pulse1 width is between PW_SHORT and PW_LONG
    if (pulse1 - PW_SHORT < PW_TOLERANCE) return 1;
    if (PW_LONG - pulse1 < PW_TOLERANCE) return 0;
    return -1;
}

uint8_t WS8610Receiver::parity(const uint8_t b1, const uint8_t b2) {
    uint8_t t = b1 ^ b2;
    t ^= t >> 4;
    t ^= t >> 2;
    t ^= t >> 1;
    return t & 1;    
}

void RECEIVE_ATTR WS8610Receiver::decodeTimings(int timingPos) {
    uint8_t packet[PACKET_SIZE] = {0};

    uint32_t t1, t2;
    int bit;
    for(int b = 0; b < TIMINGS_BUFFER_SIZE / 2; b++) {
        timingPos++;
        if (timingPos == TIMINGS_BUFFER_SIZE) timingPos = 0;
        t1 = WS8610Receiver::timingsBuf[timingPos];
        timingPos++;
        if (timingPos == TIMINGS_BUFFER_SIZE) timingPos = 0;
        t2 = WS8610Receiver::timingsBuf[timingPos];
        
        bit = WS8610Receiver::decodeBit(t1, t2);
        if (bit == -1) return; // Timings mismatch
        
        packet[b / 8] <<= 1;
        if (bit == 1) packet[b / 8]++;
    }
    // Check parity. Parity bit is #19 and it makes data bits (from #19 to #31) even
    if (WS8610Receiver::parity(packet[2] & 0x1F, packet[3])) return; // Parity error

    // Checksum
    uint8_t checksum = 0;
    for(int b = 0; b < PACKET_SIZE - 1; b++) checksum += (packet[b] & 0xF) + (packet[b] >> 4);
    if ((checksum & 0xF) != packet[PACKET_SIZE - 1]) return; // Checksum error

    // check start sequence
    if (packet[0] != 0x0A) return;

    WS8610Receiver::receivedValue = {
        .sensorAddr = ((packet[1] << 3) & 0x7F) + (packet[2] >> 5),
        .type = (packet[1] >> 4)? HUMIDITY : TEMPERATURE,
        .units = (int)(packet[2] & 0xF) * 10 + (packet[3] >> 4) - ((packet[1] >> 4)? 0 : 50),
        .decimals = packet[3] & 0xF
    };    
}

void RECEIVE_ATTR WS8610Receiver::handleInterrupt() {
    static int timingPos = 0;
    static uint32_t lastTime = 0;

    const uint32_t time = micros();
    const uint32_t duration = time - lastTime;

    if (duration > 50000) { // Synchronization signal detected
        WS8610Receiver::timingsBuf[timingPos] = PW_FIXED;
        WS8610Receiver::decodeTimings(timingPos);
    }

    if (timingPos >= TIMINGS_BUFFER_SIZE) timingPos = 0;
    WS8610Receiver::timingsBuf[timingPos++] = duration;
    lastTime = time;  
}
#endif
