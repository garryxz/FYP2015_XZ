//based on Improving the Beginner PID
//http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

double input, output;
double Iterm, previnput; //Derivative on Measurement
double Kp, Ki, Kd;
double MAX, MIN;

/*
// Set PID on/off
 void Modesetup(int mode){
 
 boolean temp = (mode == 1);
 if(temp && !auto){   // if switched on
 PIDinit();
 }
 auto = temp;
 }
 */


// get PID gain
void PIDsetup(double kp, double ki, double kd, double sampletime){
  previnput = 0;
  Iterm = 0;
  double sampletimesec = ((double)sampletime) / 1000;
  Kp = kp;
  Ki = ki * sampletimesec;
  Kd = kd / sampletimesec;
}
/*
// set sampling time
void sampletimesetup(int setsample){

  if(setsample > 0){

    double ratio = (double)setsample / (double)sampletime;

    Ki *= ratio;
    Kd /= ratio;

    sampletime = (unsigned long)setsample;
  }
}
*/
// set limits - avoid windup induced lag
void PIDlimit(double Min, double Max){

  if(Min > Max)
    return;
  MAX = Max;
  MIN = Min;

  if(Iterm > MAX){
    Iterm = MAX;
  }
  else if(Iterm < MIN){
    Iterm = MIN;
  }

  if(output > MAX){
    output = MAX;
  }
  else if(output < MIN){
    output = MIN;
  }


}



// PID algorithm output
void PIDcompute(double input, double setpoint){

  double Error = setpoint - input; // calculate P term

  Iterm += (Ki * Error);  // calculate I term with wrap
  if(Iterm > MAX){
    Iterm = MAX;
  }
  else if(Iterm < MIN){
    Iterm = MIN;
  }

  double dinput = input - previnput;  // calculate D term

  //PID output + wrap
  output = Kp * Error + Iterm + Kd * dinput;
  
  if(output > MAX){
    output = MAX;
  }
  else if(output < MIN){
    output = MIN;
  }

  previnput = input;
}



void PIDreset(){

  Iterm = 0;

}



