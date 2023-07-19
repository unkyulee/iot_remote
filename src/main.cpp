#include "IR/IR.h"
#include "GUI/GUI.h"

#include <Arduino.h>

void setup()
{
  // setup serial
  Serial.begin(9600);
  delay(100);

  // IR Setup
  IR_setup();

  // GUI Setup
  GUI_setup();
}
void loop()
{
  // IR loop
  IR_loop();

  // GUI loop
  GUI_loop();
}