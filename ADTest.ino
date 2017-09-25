/*
   This is code to test the A/D for noise with and without the motor
   being on.  There's no need to convert the A/D output to an angle.
   We can also use this to examine the motor output with a scope.
   If the voltage is ok, we can also use the A/D's
*/
// Streaming supports Serial << "text"; (like C++)
#include <Streaming.h>
#include <Adafruit_MotorShield.h>

// debug code
#define EN(x) Serial << "entering " << x << endl
#define EX(x) Serial << "exiting " << x << endl

#define UL unsigned long
#define H ((Histogram *)Histo)

// constants
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

// globals
int     potCount = 0;        // value read from the pot
float   angleRead;
UL   sTime;  // for tic(), toc()
UL   gTime;  // global for gTime = toc()



// motor related
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select which 'port' M1, M2, M3 or M4. In this case, M4
Adafruit_DCMotor *myMotor = AFMS.getMotor(4);

// Histogram class mostly it assumes we've working with ints
class Histogram {
  public:
  UL *hist;
  int histSize;
  int low, high;
  UL below, above;
  int shift; // for speed, spacing is power of 2
  
  Histogram( int n) {
    //EN("Histogram");
    this->histSize = n;
    this->hist = new UL[n];
    //EX("Histogram");
  }
  
  ~Histogram() {
    delete this->hist;
  }
  
  void Add( int d ) {
    //EN("Add");
    if (  d < this->low ) {
      this->below++;  
    }
    else if ( d > this->high ) {
      this->above++;
    }
    else {
      hist[(d-this->low)>>this->shift]++;
    }
    //EX("Add");
  }
  
  void Clear(){
    int i;
    //EN("Clear");
    Serial << "&hist=" << (unsigned int)&(this->hist) << " histSize=" << this->histSize << endl;
    for( i = 0; i < this->histSize;i++ ){
      this->hist[i] = 0L;
    }
    this->below = 0L;
    this->above = 0L;
    //EX("Clear");    
  }
  
  void Dump(){
    int i, bin;
    
    Serial << "bin\tcount\r\n";
    Serial << "below\t" << this->below << endl;
    for ( i = 0; i < this->histSize; i++ ) {
      bin = this->low -1 + (1 << this->shift)*i;
      Serial << bin << "\t" << this->hist[i] << endl;
    }
    Serial << "above\t" << this->above << endl << endl;
  }
  
  void SetScale( int l, int h){
    int d;
    Serial << "SetScale( " << l << ", " << h << ")\r\n";

    this->low = l;
    this->high = h;
    d = (h-l+1)/this->histSize;

    for(this->shift = 0; (1 <<this->shift) < d; this->shift++){}
    Serial << "d= " << d << " shift= " << this->shift <<endl;
  }
};

//Histogram Histo = new Histogram(32);
void *Histo; //this works using the H macro to cast it to Histogram *
  

void setup() {
  // Set up serial port
  Serial.begin(115200);
  Help();
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor->run(RELEASE); // make sure motor is stopped
  Histo = new Histogram(32);
}

void loop() {
  char  inByte;
  bool  testOn = false;
  UL  samples, count;
  //unsigned long   sum, sumSq; this is "better" except sumSq can exceed 4G
  UL   sum, sumSq;
  UL   dUsec = 1;
  int   histRange = 32;
  int   mSpeed = 0;
  int   i;  
  UL   loopTime, lastTime;
  char  dStr[20];
  
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
      case '?' :
        Help();
        Serial << "Process " << samples << " samples " << "Loop delay = " << dUsec << " usec\r\n";
        Serial << "Run time = " << samples * dUsec * 1e-6 << " sec\r\n";
        Serial << "histogram range = " << histRange << " histogram  bins = " << H->histSize << endl; 
        Serial << "Run motor at " << mSpeed << endl<<endl;
        break;
      case 'a' : // get A/D paramenters
        samples = Serial.parseInt();
        Serial << "Process " << samples << " samples\r\n";
        break;
      case 'd' :
        dUsec = Serial.parseInt();
        Serial << "Loop delay = " << dUsec << " usec\r\n";
        break;
      case 'm' :
        mSpeed = Serial.parseInt();
        Serial << "Run motor at " << mSpeed << endl;
        break;
      case 'g' : // go - start controller
        testOn = true;
        sum = sumSq = 0L;
        count = 0L;
        H->Clear();
        Serial << "\r\nStarting test\r\n";
        MotorOn( mSpeed );
        lastTime = micros();
        delay( 1000 ); // wait for motor to come up to speed
        Serial << "Run time = " << samples * dUsec * 1e-6 << " sec\r\n";
        break;
      case 's' : // stop controller
        testOn = false;
        MotorOn( 0 );
        break;
      case 'r' :
        histRange = Serial.parseInt();
        Serial << "histogram range = " << dUsec << endl;
        break;
      default: // bad input, just eat the character
        break;
    }
  
    if ( testOn ) { // run test
      if ( count++ >= samples ) { // finished, display data
        double mean, std;

        mean = sum / samples;
        std = sqrt((double(sumSq)/samples) - sq(mean));
        Serial << "For " << samples << " samples with motor ";
        if ( 0 == mSpeed ) {
          Serial << "stopped\r\n";
        }
        else {
          Serial << "set to " << mSpeed << endl;
        }
        // basically, it's a crap shoot for more than about 1000 samples
        // setting sum and sumSq to unsinged long can overflow.  Using Arduino's double loses too much precision.
        Serial << "sum=" << sum <<" sumSq=" << sumSq << " samples=" << samples << endl;
        Serial << "mean = " << mean << " std = " << std << " counts\r\n";
        mean = 143.996650585439 + (-0.241484320020696) * mean;
        std = (0.241484320020696) * std;
        Serial << "mean = " << mean << " std = " << std << " degrees\r\n";
        Serial << "last loopTime " << loopTime << endl;
        Serial << "Histogram\r\n";
        H->Dump();
        myMotor->run(RELEASE); // make sure motor is stopped (coasting)
        testOn = false;
      }
      else { //not finished
        int r;
        
        // read the analog in value:
        potCount = analogRead(analogInPin);
        if (1 == count ) { 
          // first time through, set up scaling for histo around current value
          r = histRange >>1;
          H->SetScale( potCount-r, potCount+r-1); // one count per bin
        }
        sum += potCount;
        sumSq += sq((UL)potCount);
        H->Add(potCount);
// debug Serial << count << "\t"<< potCount << "\t"<< sum << "\t"<< sumSq<<endl;
      }
      do {
        loopTime = micros() - lastTime;
      } while ( loopTime < dUsec );
      
      lastTime += loopTime; // so lastTime = micros()

    } // run test
  } // while true

}

// functions are alphabetical


void Help() {
  Serial << "A/D noise test\r\n";
  Serial << "? - print help information\r\n";
  Serial << "q - stop motor, reset\r\n";
  Serial << "a<number> - select number of angle samples\r\n";
  Serial << "d<number> - loop delay (uSec)\r\n";
  Serial << "m<number> - set motor speed\r\n";
  Serial << "g - go\r\n";
  Serial << "r - range for histogram\r\n";
  Serial << "s - stop early\r\n";
}


void MotorOn( int sp ) {

  sp = constrain( (int)sp, -255, 255 );
  if ( 0 != sp ) {
    myMotor->setSpeed( abs(sp) );
    if ( sp >= 0 ) {
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
boolean Timer( UL t ) {
  static UL  last = 0;
  UL ms = millis();

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

UL toc() {
  return micros() - sTime;
}


