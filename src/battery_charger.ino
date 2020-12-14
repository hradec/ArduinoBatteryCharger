/* 
==============================================================================================================================
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
==============================================================================================================================
*/

#include "batteryCharger.hpp"

#define VOLTAGE_REFERENCE_PIN 2
batteryCharger *charger1;
batteryCharger *charger2;
void setup(){
  // D1 = digital output PWM pin for charging
  // A0 = analog input pin before the 10ohm resistor
  // A1 = analog input pin after the 10ohm resistor
  // A2 = analog input pin connected to 5V for reference
  // use batteryCharger( <D1>, <A0>, <A1>, <A2> ) to create a charger for one battery. 
  // We can create as many as we want, using different D1,A0 and A1 pins, charing the same A2 pin
  charger1 = new batteryCharger( 11, 0, 1, VOLTAGE_REFERENCE_PIN );
  charger1->serialBegin();

  // example of a second charger to charge a second battery with the same arduino. 
  // this second one uses pin 10 as PWM to charge, and analogs 3 and 4 to measure current
  charger2 = new batteryCharger( 10, 3, 4, VOLTAGE_REFERENCE_PIN );
  charger2->serialBegin();
}

void loop(){
  // pass 1 to loop() to clear the serial output on every interaction
  charger1->loop(1); 

  // without passing anything to loop(), the serial will not clear the screen
  charger2->loop();
}
