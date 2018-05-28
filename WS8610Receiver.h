/*
  WS8610Receiver - Arduino library for decoding RF 433Mhz signals of Lacrosse wheater stations sensors
  Tested with the following transmitter models: TX3-TH, TX4 and TX7U

  Based on RCSwitch library, by Suat Özgür (https://github.com/sui77/rc-switch/)
  
  OVERVIEW OF MAIN LOGIC 
  (by @Joetgithub https://github.com/Joetgithub/TX7U/blob/master/TX7UReceiver.ino)
  
  Continuously loads pulses into a rolling buffer that is sized to hold one temp
  or humidity reading. About every 57 seconds the TX4 and TX7 sensors send a
  data transmission consisting of a 44 bit temperature sequence followed by a
  repeat of that same 44 bit temperature sequence followed by a 44 bit humidity
  sequence.  A relatively long delay occurs after each temp/humidity 44 bit sequence
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

#define PW_FIXED 1050 // Pulse width for the "fixed" part of signal
#define PW_SHORT 550  // Pulse width for the "short" part of signal
#define PW_LONG 1340  // Pulse width for the "long" part of signal
#define PW_TOLERANCE 140

#define TIMINGS_BUFFER_SIZE 88
#define PACKET_BUFFER_SIZE 20
#define MEASURE_BUFFER_SIZE 10

#ifdef ESP8266
    // interrupt handler and related code must be in RAM on ESP8266
    #define RECEIVE_ATTR ICACHE_RAM_ATTR
#else
    #define RECEIVE_ATTR
#endif

enum measureType : uint8_t {TEMPERATURE, HUMIDITY};

struct packet {
    uint32_t msec;
    uint32_t timings[TIMINGS_BUFFER_SIZE];
};

struct measure {
    uint32_t msec;
    uint8_t sensorAddr;
    measureType type;
    int8_t units;
    uint8_t decimals;
};

class WS8610Receiver {
public:
    WS8610Receiver(const int pin);
    void enableReceive();
    void disableReceive();
    int receivedMeasures();
    measure getNextMeasure();

private:
    static volatile uint32_t timingsBuf[TIMINGS_BUFFER_SIZE];
    static volatile packet packets[PACKET_BUFFER_SIZE];
    static volatile int packetPos;
    int interrupt;
    int lastPacketPos;
    measure measures[MEASURE_BUFFER_SIZE];
    int measurePos;
    int lastMeasurePos;

    static void handleInterrupt();
    int decodeBit(const uint32_t pulse1, const uint32_t pulse2);
    bool decodePacket();
    bool unreadMeasures();
};

volatile uint32_t WS8610Receiver::timingsBuf[TIMINGS_BUFFER_SIZE];
volatile packet WS8610Receiver::packets[PACKET_BUFFER_SIZE];
volatile int WS8610Receiver::packetPos = 0;

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
    for(int p = 0; p < PACKET_BUFFER_SIZE; p++) WS8610Receiver::packets[p].msec = 0;
    measurePos = lastMeasurePos = 0;
}


/**
 * Enable receiving data
 */
void WS8610Receiver::enableReceive() {
    WS8610Receiver::packetPos = 0;
    lastPacketPos = 0;
    attachInterrupt(this->interrupt, handleInterrupt, CHANGE);
}

/**
 * Disable receiving data
 */
void WS8610Receiver::disableReceive() {
    detachInterrupt(this->interrupt);
}

void RECEIVE_ATTR WS8610Receiver::handleInterrupt() {
    static int timingPos = 0;
    static uint32_t lastTime = 0;
    static uint32_t lastSync = 0; // Number of timings since last sync signal

    const uint32_t time = micros();
    const uint32_t duration = time - lastTime;
    lastTime = time;

    WS8610Receiver::timingsBuf[timingPos++] = duration;
    if (timingPos == TIMINGS_BUFFER_SIZE) timingPos = 0;
    lastSync++;

    if (duration > 5000) { // Synchronization signal detected
        // Sync signal must be at least one packet away from the previous one
        if (lastSync > TIMINGS_BUFFER_SIZE) {
            WS8610Receiver::packets[packetPos].msec = millis();
            for(int t = 0; t < TIMINGS_BUFFER_SIZE; t++) {
                WS8610Receiver::packets[packetPos].timings[t] = WS8610Receiver::timingsBuf[timingPos];
                if (++timingPos == TIMINGS_BUFFER_SIZE) timingPos = 0;
            }
            packetPos++;
            if (packetPos == PACKET_BUFFER_SIZE) packetPos = 0;
        }
        lastSync = 1;
    }
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

bool WS8610Receiver::decodePacket() {
    volatile packet *p = &WS8610Receiver::packets[lastPacketPos];
    if (++lastPacketPos == PACKET_BUFFER_SIZE) lastPacketPos = 0;

    // Decode and pack the bits into an array of bytes
    uint8_t bytes[6] = {0};
    int bit;
    p->timings[TIMINGS_BUFFER_SIZE - 1] = PW_FIXED;
    for(int b = 0; b < TIMINGS_BUFFER_SIZE; b += 2) {
        bit = WS8610Receiver::decodeBit(p->timings[b], p->timings[b+1]);
        if (bit == -1) {
//            Serial.printf("Timings mismatch (%d, %d)\n", p->timings[b], p->timings[b+1]);
            return false; // Timings mismatch
        }

        bytes[b / 16] <<= 1;
        if (bit == 1) bytes[b / 16]++;
    }

    // check start sequence
    if (bytes[0] != 0x0A) {
//        Serial.println("Wrong start sequence");
        return false;
    }

    // Check parity. Parity bit is #19 and it makes data bits (from #19 to #31) even
    uint8_t bits = (bytes[2] & 0x1F) ^ bytes[3];
    bits ^= bits >> 4;
    bits ^= bits >> 2;
    bits ^= bits >> 1;
    if (bits & 1) {
//        Serial.println("Parity error");
        return false; // Parity error
    }

    // Checksum
    uint8_t checksum = 0;
    for(int b = 0; b < 5; b++) checksum += (bytes[b] & 0xF) + (bytes[b] >> 4);
    if ((checksum & 0xF) != bytes[5]) {
//        Serial.println("Checksum error");
        return false; // Checksum error
    }

    measures[measurePos].msec = p->msec;
    measures[measurePos].sensorAddr = ((bytes[1] << 3) & 0x7F) + (bytes[2] >> 5);
    measures[measurePos].type = (bytes[1] >> 4)? HUMIDITY : TEMPERATURE;
    measures[measurePos].units = (int8_t)(bytes[2] & 0xF) * 10 + (bytes[3] >> 4) - ((bytes[1] >> 4)? 0 : 50);
    measures[measurePos].decimals = bytes[3] & 0xF;
    if (++measurePos == MEASURE_BUFFER_SIZE) measurePos = 0;
    return true;
}

int WS8610Receiver::receivedMeasures() {
    // Counts how many unread measures there are in the buffer
    int unreadMeasures = measurePos - lastMeasurePos;
    if (unreadMeasures < 0) unreadMeasures += MEASURE_BUFFER_SIZE;

    // Checks if there is any new measure among the received packets and decodes them
    while(lastPacketPos != WS8610Receiver::packetPos) {
        // Tries to decode a packet
        if (decodePacket()) unreadMeasures++;
    }
    return unreadMeasures;
}

bool WS8610Receiver::unreadMeasures() {
    // Checks if there are unread measures in the buffer
    if (lastMeasurePos != measurePos) return true;

    // Checks if there is any new measure in the received packets and decodes it
    while(lastPacketPos != WS8610Receiver::packetPos) {
        // Tries to decode a packet
        if (decodePacket()) return true;
    }
    return false;
}

measure WS8610Receiver::getNextMeasure() {
    // Checks if there are unread measures in the buffer
    if (!unreadMeasures()) return { 0, 0, TEMPERATURE, 0, 0 };

    // There are unread measures in the buffer, get the next one
    measure* m = &measures[lastMeasurePos];
    if (++lastMeasurePos == MEASURE_BUFFER_SIZE) lastMeasurePos = 0;
    return {
        m->msec,
        m->sensorAddr,
        m->type,
        m->units,
        m->decimals
    };
}
#endif
