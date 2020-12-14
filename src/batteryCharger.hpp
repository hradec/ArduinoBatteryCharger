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

// ==============================================================================================================================
// Constants
// ==============================================================================================================================
#define BATTERY_CAPACITY          2500.0 // capacity rating of battery in mAh
#define RESISTANCE                10.0   // measured resistance of the power resistor
#define CUTOF_VOLTAGE             1650.0 // maximum battery voltage (in mV) that should not be exceeded
#define CHARGING_VOLTAGE          1800.0 // charging voltage limit
#define CHARGING_VOLTAGE_BOOST    2100.0 // charging voltage limit during boost
#define MAXIMUN_CHARGING_CURRENT  1000.0 // this will be set automatically later on once the PWM charging pin gets to 255!
#define CHECK_CHARGE 500
#define STUCK_BOOST_THRESHOLD 4
#define TARGET_CURRENT_STEP 1
#define VOLTAGE_READING_AVRG_LOOP (100*2)
#define VIN_5 5000
#define TERMINAL_SIZE 70


class batteryCharger {
  public:
    float cutoffTemperatureC; //maximum battery temperature that should not be exceeded (in degrees C)
    float cutoffTemperatureF; //maximum battery temperature that should not be exceeded (in degrees F)
    long cutoffTime;          //maximum charge time of 13 hours that should not be exceeded
    int chargingVoltage;      //maximum battery charging voltage (in mV) that should not be exceeded

    int outputPin;            //Output signal wire connected to digital pin 9
    int outputValue;          //value of PWM output signal
    int _outputValue;         //value of PWM output signal
    int _old_outputValue;     //value of PWM output signal

    int analogPinOne;         //first voltage probe connected to analog pin 1
    float valueProbeOne;      //variable to store the value of analogPinOne
    float voltageProbeOne;    //calculated voltage at analogPinOne

    int analogPinTwo;         //second voltage probe connected to analog pin 2
    float valueProbeTwo;      //variable to store the value of analogPinTwo
    float voltageProbeTwo;    //calculated voltage at analogPinTwo

    int analogPinThree;       //third voltage probe connected to analog pin 2
    float valueProbeThree;    //variable to store the value of analogPinThree
    float voltageProbeThree;

    float tmp36Voltage;       //calculated voltage at analogPinThree
    float temperatureC;       //calculated temperature of probe in degrees C
    float temperatureF;       //calculated temperature of probe in degrees F

    float voltageDifference;  //difference in voltage between analogPinOne and analogPinTwo
    float batteryVoltage;     //calculated voltage of battery
    float current;            //calculated current through the load (in mA)
    float targetCurrent;      //target output current (in mA) set at C/10 or 1/10 of the battery capacity per hour
    float currentError;       //difference between target current and actual current (in mA)
    int maximunAutoAdjustedChargingCurrent;

    float charged;
    float checkIfCharged;
    float ChargingDifference;
    float voltageReference;
    float chargeStuckBoost;
    float batteryVoltageAvgOld;

    String doubleLine;
    String singleLine;

    int __deferedFullClear;

    batteryCharger(int __outputPin, int __analogPinOne, int __analogPinTwo, int __analogPinThree);
    float getBatteryVoltage();
    void deferedFullClearScreen();
    void serialClearScreen(int REAL);
    void serialBegin();
    void loop(int CLEAR_SCREEN);
    void print(String msg);
    String repeat(String str, int times);
};


// ====================================================================================
batteryCharger::batteryCharger(int __outputPin, int __analogPinOne, int __analogPinTwo, int __analogPinThree){
    outputPin       = __outputPin;
    analogPinOne    = __analogPinOne;
    analogPinTwo    = __analogPinTwo;
    analogPinThree  = __analogPinThree;

    // ======================================
    // extra class global variables init
    // ======================================
    cutoffTemperatureC = 35;    //maximum battery temperature that should not be exceeded (in degrees C)
    // cutoffTemperatureF = 95;    //maximum battery temperature that should not be exceeded (in degrees F)
    cutoffTime = 46800000;      //maximum charge time of 13 hours that should not be exceeded
    chargingVoltage = CHARGING_VOLTAGE;     //maximum battery charging voltage (in mV) that should not be exceeded
    outputValue = 207;          //value of PWM output signal
    _outputValue = 0;           //value of PWM output signal
    _old_outputValue = 0;       //value of PWM output signal

    valueProbeOne = 0;          //variable to store the value of analogPinOne
    voltageProbeOne = 0;        //calculated voltage at analogPinOne
    valueProbeTwo = 0;          //variable to store the value of analogPinTwo
    voltageProbeTwo = 0;        //calculated voltage at analogPinTwo
    valueProbeThree = 0;        //variable to store the value of analogPinThree
    voltageProbeThree = 0;
    tmp36Voltage = 0;           //calculated voltage at analogPinThree
    temperatureC = 0;           //calculated temperature of probe in degrees C
    // temperatureF = 0;           //calculated temperature of probe in degrees F

    voltageDifference = 0;                  //difference in voltage between analogPinOne and analogPinTwo
    batteryVoltage = 0;                     //calculated voltage of battery
    current = 0;                            //calculated current through the load (in mA)
    targetCurrent = BATTERY_CAPACITY / 10;  //target output current (in mA) set at C/10 or 1/10 of the battery capacity per hour
    currentError = 0;                       //difference between target current and actual current (in mA)
    maximunAutoAdjustedChargingCurrent = 1000;

    charged = 0;
    checkIfCharged = 2000;
    ChargingDifference;
    voltageReference = 4900;
    chargeStuckBoost = 0;
    batteryVoltageAvgOld = 0;
    __deferedFullClear = 1;

    doubleLine = repeat( "=", TERMINAL_SIZE );
    singleLine = repeat( "-", TERMINAL_SIZE );

    pinMode(outputPin, OUTPUT);           // sets the pin as output
    _outputValue = outputValue;
    analogWrite(outputPin, _outputValue); // Write output value to output pin
}

// ====================================================================================
void batteryCharger::serialBegin(){
    Serial.begin(115200);                 // setup serial
    serialClearScreen(1);
}

// ====================================================================================
String batteryCharger::repeat(String str, int times){
    String ret="";
    for( int n=0 ; n < times ; n++ ){
        ret += str;
    }
    return ret;
}
// ====================================================================================

void batteryCharger::print(String msg){
    String txt = msg ;
    txt += repeat( " ", TERMINAL_SIZE-msg.length() );
    Serial.println(txt.c_str());
}

// ====================================================================================
void batteryCharger::deferedFullClearScreen(){
    __deferedFullClear = 1;
}

// ====================================================================================
void batteryCharger::serialClearScreen(int REAL=0){
    if ( REAL > 0 || __deferedFullClear > 0){
        Serial.write(27);       // ESC command
        Serial.print("[2J");    // clear screen command
        __deferedFullClear = 0;
    }
    Serial.write(27);
    Serial.print("[H");     // cursor to home command
    Serial.println(doubleLine.c_str());
    delay(1);
    //Serial.print("\u001B[2J");
}

// ====================================================================================
float batteryCharger::getBatteryVoltage(){
  // ====================================================================================
  // read 2 analog pins to measure the amp current going to the battery.
  // Basically put a 10ohm resistor before the battery negative line, and connect
  // pin1 right before the resistor and pin2 right after.
  // the difference in voltage reading is the current.
  // we use a 3rd pin connect to the 5Vin, so we can have a reference value for 5volts
  // to be used when calculating the voltage on pin1 and pin2.
  // So voltage reference is default to 4.9Volts on a first read, but after that is
  // automatically adjusted based on the voltage reading on pin3.
  // average the current measure values here, so instead of using delay()
  // we actually do something usefull while waiting to improve precision!
  // ====================================================================================
  voltageProbeOne = 0;
  voltageProbeTwo = 0;
  voltageProbeThree = 0;
  for( int n=0 ; n < VOLTAGE_READING_AVRG_LOOP ; n++ ){
    valueProbeOne      = analogRead(analogPinOne);    // read the input value at probe one
    voltageProbeOne   += (valueProbeOne/1023);     //calculate voltage at probe one in milliVolts
    valueProbeTwo      = analogRead(analogPinTwo);    // read the input value at probe two
    voltageProbeTwo   += (valueProbeTwo/1023);     //calculate voltage at probe two in milliVolts
    valueProbeThree    = analogRead(analogPinThree);    // read the input value at probe three
    voltageProbeThree += (valueProbeThree/1023);     //calculate voltage at probe 3 in milliVolts
  }
  voltageProbeThree = (voltageProbeThree / VOLTAGE_READING_AVRG_LOOP) * VIN_5;
  //voltageProbeThree = VIN_5 ;
  voltageReference = voltageProbeThree;
  voltageProbeOne  = (voltageProbeOne / VOLTAGE_READING_AVRG_LOOP) * voltageReference;
  voltageProbeTwo  = (voltageProbeTwo / VOLTAGE_READING_AVRG_LOOP) * voltageReference;

  // ====================================================================================
  // calculate voltage and current
  // ====================================================================================
  // calculate battery voltage
  batteryVoltage = (voltageProbeThree - voltageProbeTwo) * ( 1.0 - (1.0/RESISTANCE) );

  // calculate charge current
  current = (voltageProbeTwo - voltageProbeOne) / RESISTANCE;

  //difference between target current and measured current
  currentError = targetCurrent - current;

  /*
  // ====================================================================================
  // temperature - TODO!
  // ====================================================================================
  tmp36Voltage = valueProbeThree * 5.0;     // converting that reading to voltage
  tmp36Voltage /= 1024.0;
  temperatureC = (tmp36Voltage - 0.5) * 100 ;     //converting from 10 mv per degree wit 500 mV offset to degrees ((voltage - 500mV) times 100)
  print(String("Temperature (degrees C) ")+String(temperatureC));     //display the temperature in degrees C
  temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;     //convert to Fahrenheit
  print(Strin("Temperature (degrees F) ")+String(temperatureF));
  */

  return   batteryVoltage ;
}

// ====================================================================================
void batteryCharger::loop(int CLEAR_SCREEN=0){
    // ====================================================================================
    // getBatteryVoltage() returns batteryVoltage, but also updates the global variables:
    //  - batteryVoltage
    //  - current
    //  - currentError
    //  - voltageProbeOne
    //  - voltageProbeTwo
    // ====================================================================================
    batteryVoltage = getBatteryVoltage();


    // ====================================================================================
    // when battery voltage is above 2500, we don't have a battery connected!
    // ====================================================================================
    if ( batteryVoltage > 2500 || batteryVoltage < 200) {

        targetCurrent = BATTERY_CAPACITY / 10;
        maximunAutoAdjustedChargingCurrent = MAXIMUN_CHARGING_CURRENT;
        _outputValue  = 0;
        batteryVoltageAvgOld = chargeStuckBoost = 0;
        print("No Battery connected");     //display the temperature in degrees C
        analogWrite(outputPin, 0);
        // delay(1000);

    // ====================================================================================
    // so we have a battery to charge then...
    // ====================================================================================
    }else{
        //delay(10);

        ChargingDifference = chargingVoltage - batteryVoltage;
        currentError = targetCurrent - current;

        // ====================================================================================
        // automatically adjust target current so the charging voltage
        // stays around cutoffVoltage, avoiding over temperature and leakage.
        // ====================================================================================
        if ( ! charged ){

            // adjust the charging current so we can automatically adjust for any battery type!
            if ( ChargingDifference > 0 ){
              targetCurrent += TARGET_CURRENT_STEP;
            }else{
              targetCurrent -= TARGET_CURRENT_STEP;
            }

            // lets keep charging current always above 50 for now.
            if ( targetCurrent < 50 ){
              targetCurrent = 50;
            }
            // don't increase targetCurrent above maximunAutoAdjustedChargingCurrent
            // we set the maximunAutoAdjustedChargingCurrent to the maximun current
            // we get when charging PWM is set to 255!
            else if ( targetCurrent > maximunAutoAdjustedChargingCurrent ){
              targetCurrent = maximunAutoAdjustedChargingCurrent;
            }
        }else{
            targetCurrent = 0;
        }

        // ====================================================================================
        // automatically adjust pwm pin to feed voltage according to the required
        // amount of current needed!
        // ====================================================================================
        _outputValue = 0;
        //if(abs(currentError) > 1 and targetCurrent > 0) {    //if output error is large enough, adjust output
        if( targetCurrent > 0 ) {
            outputValue = outputValue + currentError / 10;

            if(outputValue < 0){
                outputValue = 0;
            }
            else if(outputValue > 254){     //output can never go above 255
                outputValue = 255;
                if(maximunAutoAdjustedChargingCurrent == MAXIMUN_CHARGING_CURRENT && _old_outputValue == 255){
                    // limit the maximunAutoAdjustedChargingCurrent  to the maximunChargingCurrent here, since
                    // the maximun PWM is 255, so we can't have more current than what we have right now!
                    // we also only do this if PWM is 255 twice in a row, just to be sure to limit the current
                    // when PWM is really at the max!
                    maximunAutoAdjustedChargingCurrent = targetCurrent;
                }
            }else{
                // if we're below 255, reset maximum current back so it can auto-adjust again.
                maximunAutoAdjustedChargingCurrent = MAXIMUN_CHARGING_CURRENT;
            }

            //analogWrite(outputPin, outputValue);     //write the new output value
            _outputValue = outputValue;
            _old_outputValue = _outputValue;
        }

        /*
        // ====================================================================================
        // temperature reading
        // ====================================================================================
        if(temperatureC > cutoffTemperatureC)     //stop charging if the battery temperature exceeds the safety threshold
        {
            _outputValue = 0;
            print("Max Temperature Exceeded");
        }else

        /*
        if(temperatureF > cutoffTemperatureF)     //stop charging if the battery temperature exceeds the safety threshold
        {
            outputValue = 0;
        }
        */

        // ====================================================================================
        // increase the counter to check if its charged
        // ====================================================================================
        checkIfCharged += 1;

        // ====================================================================================
        // Check if battery is charged. For that, we have to switch charging off and wait a bit
        // so the battery sets down and we can get a more reliable reading.
        // ====================================================================================
        float batteryVoltageAvg=0;
        float limitCheckIfCharged =  CHECK_CHARGE / ( 1 + charged * 10 ) ;
        int n=0;
        if( checkIfCharged > limitCheckIfCharged ) {
            deferedFullClearScreen();
            print("Waiting 5 seconds for battery cool down so we can get better \nvoltage reading...");
            analogWrite(outputPin, 0);
            delay(5000);
            print(doubleLine);
            for(n = 0 ; n<10 ; n++){
                float _batteryVoltage = getBatteryVoltage();
                batteryVoltageAvg += _batteryVoltage;
                print(String("Battery Charged Voltage (mV): ")+String(_batteryVoltage));     //display battery voltage
                delay(1000);
            }
            batteryVoltageAvg /= n;
            print(doubleLine);
            print("Battery Charged Voltage Avg (mV): "+String(batteryVoltageAvg));     //display battery voltage
            print(doubleLine);

            if ( batteryVoltageAvg < batteryVoltageAvgOld ) {
                chargeStuckBoost = 20;
            }else if( abs( CUTOF_VOLTAGE - batteryVoltageAvg ) < 50 ){
                chargeStuckBoost++;
                if( chargeStuckBoost > STUCK_BOOST_THRESHOLD ){
                    chargeStuckBoost = 20;
                }
            }else{
                if( chargeStuckBoost > STUCK_BOOST_THRESHOLD ){
                    chargeStuckBoost -= 1;
                }else{
                    chargeStuckBoost = 0;
                }
            }
            batteryVoltageAvgOld = batteryVoltageAvg;


            // ====================================================================================
            // if the battery voltage got to the cutout voltage, stop charging!
            // ====================================================================================
            if(batteryVoltageAvg > CUTOF_VOLTAGE){
                charged = 1;
            } else {
                charged =0;
            }

            // ====================================================================================
            // we keep checking if the batery is still charged, so if it drops, we start charging
            // again!
            // ====================================================================================
            checkIfCharged = 0;
            if(charged) {    //stop charging if the battery voltage exceeds the safety threshold
                targetCurrent = BATTERY_CAPACITY / 10;
                maximunAutoAdjustedChargingCurrent = MAXIMUN_CHARGING_CURRENT;
                _outputValue  = 0;
                batteryVoltageAvgOld = chargeStuckBoost = 0;
                print("Max Voltage Exceeded... stopping charging.");
            }
        }

        // ====================================================================================
        // if we are in charge stuck boost mode, set output value to max!
        // ====================================================================================
        if( chargeStuckBoost >= STUCK_BOOST_THRESHOLD ){
            chargingVoltage = CHARGING_VOLTAGE_BOOST;
        }else{
            chargingVoltage = CHARGING_VOLTAGE;
        }

        // ====================================================================================
        // write the PWM here
        // ====================================================================================
        analogWrite(outputPin, _outputValue);  //Write output value to output pin

        // ====================================================================================
        // display values on the serial port
        // ====================================================================================
        if ( CLEAR_SCREEN > 0 ) serialClearScreen();
        print("Voltage Probe One (mV): "+String(voltageProbeOne));     //display voltage at probe one
        print("Voltage Probe Two (mV): "+String(voltageProbeTwo));
        print("Voltage Probe Three (mV): "+String(voltageProbeThree));
        print("Battery Voltage (mV): "+String(batteryVoltage));
        print("Charging Voltage diff (mV): "+String(ChargingDifference));
        print("Target Current (mA): "+String(targetCurrent));
        print("Battery Current (mA): "+String(current));
        print("Current Error  (mA): "+String(currentError));
        print("Target Current Adjusted Limit (mA): "+String(maximunAutoAdjustedChargingCurrent));

        // ====================================================================================
        // bottom line final data output
        // ====================================================================================
        print(doubleLine);
        print("| Out: "
            + String( _outputValue )
            + " | isChrgd: "
            + String( String(100*(1.0 - (checkIfCharged / limitCheckIfCharged)),1) )
            + "% | stuckBoost: "
            // + String( 100*( chargeStuckBoost>STUCK_BOOST_THRESHOLD ? STUCK_BOOST_THRESHOLD : chargeStuckBoost/STUCK_BOOST_THRESHOLD) )
            + String( String( chargeStuckBoost-STUCK_BOOST_THRESHOLD < 0 ? 0 : chargeStuckBoost-STUCK_BOOST_THRESHOLD, 0 ) )
            + " | Chrgd? "
            + (charged > 0 ?
                "YES |" :
                "NO  |"
              )
        );
        print(doubleLine);
        print("");
    }
}
