/*
   This is code to test the A/D for noise with and without the motor
   being on.  There's no need to convert the A/D output to an angle.
   We can also use this to examine the motor output with a scope.
   If the voltage is ok, we can also use the A/D's
*/
// Streaming supports Serial << "text"; (like C++)
#include <Streaming.h>
#include <Adafruit_MotorShield.h>

// constants
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

// globals
int     potCount = 0;        // value read from the pot
float   angleRead;
unsigned long   sTime;  // for tic(), toc()
unsigned long   gTime;  // global for gTime = toc()

// for histogram
unsigned int histo[32];
// motor related
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select which 'port' M1, M2, M3 or M4. In this case, M4
Adafruit_DCMotor *myMotor = AFMS.getMotor(4);

void setup() {
  // Set up serial port
  Serial.begin(115200);
  Help();
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor->run(RELEASE); // make sure motor is stopped
}

void loop() {
  char  inByte;
  bool  testOn = false;
  int   samples, count;
  unsigned long   sum, sumSq;
  unsigned int    dms = 1;
  int   mSpeed = 0;
  int   i;  
  
  while ( true ) {

    // process command if present
    inByte = Serial.read();
    switch ( inByte ) {
      case -1 : // no character
        break;
      case 'q' :
        testOn = false;
        myMotor->run(RELEASE); // make sure motor is stopped (coasting)
        Serial << "Stopping test, resetting parameters\r\n";
        return;
        break;
      case 'h' :
      case '?' :
        Help();
        Serial << "Process " << samples << " samples\r\n";
        Serial << "Loop delay = " << dms << " msec\r\n";
        Serial << "Run motor at " << mSpeed << endl<<endl;
        break;
      case 'a' : // get A/D paramenters
        samples = Serial.parseInt();
        Serial << "Process " << samples << " samples\r\n";
        break;
      case 'd' :
        dms = Serial.parseInt();
        Serial << "Loop delay = " << dms << " msec\r\n";
        break;
      case 'm' :
        mSpeed = Serial.parseInt();
        Serial << "Run motor at " << mSpeed << endl;
        break;
      case 'g' : // go - start controller
        testOn = true;
        sum = sumSq = 0L;
        count = 0;
        // clear histogram
        for ( i=0; i < 32; i++) {
          histo[i] = 0;
        }
        Serial << "\r\nStarting test\r\n";
        MotorOn( mSpeed );
        delay( 1000 ); // wait for motor to come up to speed
        break;
      case 's' : // stop controller
        testOn = false;
        MotorOn( 0 );
        break;
      default: // bad input, just eat the character
        break;
    }
  
    if ( testOn ) { // run test
      if ( count++ >= samples ) { // finished, display data
        double mean, std;
        
        mean = sum / samples;
        std = sqrt(sumSq/samples - sq(mean));
        Serial << "For " << samples << " samples with motor ";
        if ( 0 == mSpeed ) {
          Serial << "stopped\r\n";
        }
        else {
          Serial << "set to " << mSpeed << endl;
        }
        // debug Serial << "sum=" << sum <<" sumSq=" << sumSq << " samples=" << samples << endl;
        Serial << "mean = " << mean << " std = " << std << " counts\r\n";
        mean = 143.996650585439 + (-0.241484320020696) * mean;
        std = (0.241484320020696) * std;
        Serial << "mean = " << mean << " std = " << std << " degrees\r\n\n";
        Serial << "Histogram\r\nn\tcount\r\n";
        for ( i=0; i < 32; i++ ){
          Serial << (i << 5) << "\t" << histo[i] << endl;
        }
        Serial << endl;
        myMotor->run(RELEASE); // make sure motor is stopped (coasting)
        testOn = false;
      }
      else { //not finished
//        if (1 == count ){ // debug
//          Serial << "n\tv\tsum\tsumSq\r\n";
//        }
        // read the analog in value:
        potCount = analogRead(analogInPin);
        sum += potCount;
        sumSq += sq((unsigned long)potCount);
        histo[potCount >> 5]++;
// debug Serial << count << "\t"<< potCount << "\t"<< sum << "\t"<< sumSq<<endl;
      }
      delay( dms );
    } // run test
  } // while true

}

// functions are alphabetical


void Help() {
  Serial << "A/D noise test\r\n";
  Serial << "q - stop motor, reset\r\n";
  Serial << "a<number> - select number of samples\r\n";
  Serial << "m<number> - set motor speed\r\n";
  Serial << "g - go\r\n";
  Serial << "s - stop early\r\n";
}


void MotorOn( int sp ) {

  sp = constrain( (int)sp, -255, 255 );
  if ( 0 != sp ) {
    myMotor->setSpeed( abs(sp) );
    if ( sp <= 0 ) {
      myMotor->run(FORWARD);
    }
    else {
      myMotor->run(BACKWARD);
    }
  }
  else {
    myMotor->run(RELEASE); // make sure motor is stopped (coasting);
  }
  Serial << "motor set to " << sp <<endl;
}


// read the angle and return it.
// potCount and angle read are globals,for convenience in DisplayAngle
// returns value for use in other pieces of code
float ReadAngle() {
  // read the analog in value:
  potCount = analogRead(analogInPin);
  // emperically, angleRead = 143.996650585439 + (-0.24148432002069) * potCount
  angleRead = 143.996650585439 + (-0.241484320020696) * potCount;
  return angleRead;
}

// if at least t milliseconds has passed since last call, reset timer and return true
// allows non-blocking timed events
boolean Timer( unsigned long t ) {
  static unsigned long  last = 0;
  unsigned long ms = millis();

  if ( ms >= t + last ) {
    last = ms;
    return true;
  }
  else {
    return false;
  }
}

// Timer utility functions.  Concept from Matlab
void tic() {
  sTime = micros();
}

unsigned long toc() {
  return micros() - sTime;
}


