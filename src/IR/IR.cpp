#include "IR/IR.h"
#include <Arduino.h>
#include <IRremote.hpp>

uint8_t turn_on[229] = {61, 62, 61, 89, 11, 34, 10, 12, 11, 34, 10, 11, 11, 11, 11, 34, 11, 34, 10, 12, 10, 34, 11, 11, 11, 11, 11, 34, 11, 11, 11, 11, 11, 34, 10, 12, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 10, 12, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 34, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 34, 11, 11, 11, 34, 10, 11, 11, 11, 11, 12, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 10, 34, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 10, 12, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 11, 10, 12, 10, 12, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 10, 12, 11, 34, 10, 11, 11, 34, 11, 11, 11, 34, 10, 11, 11, 34, 11, 34, 10, 34, 11, 11, 11, 34, 10}; // Protocol=UNKNOWN Hash=0x33D9D85F 115 bits (incl. gap and start) received

uint8_t turn_off[229] = {61, 62, 61, 89, 11, 34, 11, 11, 11, 34, 10, 11, 11, 12, 10, 34, 11, 34, 10, 12, 11, 34, 10, 11, 11, 11, 11, 34, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 10, 12, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 10, 12, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 34, 11, 11, 11, 34, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 10, 12, 10, 12, 11, 11, 11, 34, 10, 11, 11, 11, 11, 12, 10, 12, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 10, 12, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 10, 12, 10, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 10, 12, 10, 12, 11, 11, 11, 11, 11, 34, 10, 11, 11, 34, 11, 11, 11, 11, 11, 11, 11, 34, 10, 35, 10, 11, 11, 34, 11, 34, 10}; // Protocol=UNKNOWN Hash=0x40A1F1BD 115 bits (incl. gap and start) received

void IR_setup()
{
    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
    Serial.println("");
    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));

    // IrSender.sendRaw(turn_off, sizeof(turn_off), 38);
}
void IR_loop()
{
    if (IrReceiver.decode() && 0)
    {
        /*
         * Print a short summary of received data
         */
        IrReceiver.compensateAndPrintIRResultAsCArray(&Serial, false); // Output the results as uint8_t source code array of ticks
        if (IrReceiver.decodedIRData.protocol == UNKNOWN)
        {
            Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
            // We have an unknown protocol here, print more info
            IrReceiver.printIRResultRawFormatted(&Serial, true);
        }
        Serial.println();

        /*
         * !!!Important!!! Enable receiving of the next value,
         * since receiving has stopped after the end of the current received data packet.
         */
        IrReceiver.resume(); // Enable receiving of the next value
    }
}
