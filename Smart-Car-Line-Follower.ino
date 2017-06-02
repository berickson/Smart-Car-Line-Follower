#include <SoftwareSerial.h>


float Kp=100,Ki=0,Kd=.24;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0;
float previous_time=0;
int sensor[5]={0, 0, 0, 0, 0};
int initial_motor_speed=100;
int max_speed = 200;

int left_motor_speed = 0;
int right_motor_speed = 0;

bool go = false;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

const int pin_left_motor_enable = 6;
const int pin_right_motor_enable = 11;
 
const int pin_left_forward = 4;
const int pin_left_reverse = 5;

const int pin_right_forward = 12;
const int pin_right_reverse = 3;

const int pin_usb_rx = 9;
const int pin_usb_tx = 10;



SoftwareSerial usb(pin_usb_rx, pin_usb_tx);

void setup()
{
  pinMode(pin_right_motor_enable,OUTPUT); 
  pinMode(pin_left_motor_enable,OUTPUT); 

  digitalWrite(pin_left_motor_enable,0);
  digitalWrite(pin_right_motor_enable,0);
  
  pinMode(pin_left_forward,OUTPUT);
  pinMode(pin_left_reverse,OUTPUT);
  pinMode(pin_right_forward,OUTPUT);
  pinMode(pin_right_reverse,OUTPUT);

  digitalWrite(pin_left_forward,0);
  digitalWrite(pin_left_reverse,0);
  digitalWrite(pin_right_forward,0);
  digitalWrite(pin_right_reverse,0);
  usb.begin(9600);
  delay(1000);
  usb.println();
  usb.flush();
  usb.println("AT+NAMESmartCar");
  usb.flush();
  
 Serial.begin(115200);
}

char command_buffer[80] = {0};
int command_length = 0;



/*void trace(String s) {
  return;
  Serial.print(s);
}
void trace(int i) {
  trace(String(i));
}*/


void execute_command(char * s) {
  
  usb.print("executing command ");
  usb.println(s);
  if(s[0] == 'g') {
      go = true;
      usb.println("go");
  }
  if(s[0] == 's') {
      go = false;
      usb.println("stop\n");
  }
  if(s[0] == 'v') {
    initial_motor_speed=atoi(s+1);
    usb.print("set initial_motor_speed to ");
    usb.println(initial_motor_speed);
  }
  if(s[0] == 'm') {
    max_speed=atoi(s+1);
    usb.print("set max_speed to ");
    usb.println(max_speed);
  }
  if(s[0] == 'p') {
    Kp=atoi(s+1);
    usb.print("set Kp to ");
    usb.println(Kp);
  }
  if(s[0] == 'd') {
    Kd=atoi(s+1)/100.;
    usb.print("set Kd to ");
    usb.println(Kd);
  }
  usb.flush();
}

void loop()
{
  if(usb.available()) {
    
    char c = usb.read();
    if(c=='\r' || c=='\n') {
      if(command_length > 0) {
        execute_command(command_buffer);
      }
      command_buffer[0] = 0;
      command_length = 0;
      
    }
    else {
      if(command_length > 30) {
        command_length = 0;
      }
      command_buffer[command_length] = c;
      command_buffer[++command_length]=0;
    }
  }
  
  get_line_position();
  //return;
  //read_sensor_values();
/*  trace("Sensors: ");
  trace(sensor[0]);
  trace(sensor[1]);
  trace(sensor[2]);
  trace(sensor[3]);
  trace(sensor[4]);
  trace("  Error: ");
  trace(error);*/
  calculate_pid();
  motor_control();
  /*trace(" Left: ");
  trace(left_motor_speed);

  trace(" Right: ");
  trace(right_motor_speed);
  trace("\n");*/
    
}

int get_line_position() {
  sensor[0]=digitalRead(A0);
  sensor[1]=digitalRead(A1);
  sensor[2]=digitalRead(A2);
  sensor[3]=digitalRead(A3);
  sensor[4]=digitalRead(A4);

  const int sensor_count = 5;
  const int max_sensor = sensor_count -1;

  // find first and last sensor that shows a line
  int line_start = -1;
  int line_end = -1;
  for(int i=0; i<sensor_count; i++) {
    if(sensor[i]) {
      if(line_start == -1) {
        line_start = i;
      }
      line_end = i;
    }
  }

  bool ok = true;

  // must have at least one line reading to be ok
  if(line_end == -1) {
    ok = false;
  }
  
  // reading must be contiguous block of sensors to be ok
  if(ok) {
    for(int i = line_start; i < line_end; i++) {
      if(sensor[i]==false) {
        ok = false;
      }
    }
  }

  // can't be all black
  if(ok) {
    if( line_start == 0 && line_end == max_sensor) {
      ok = false;
    }
  }

  if(ok) {
    error = line_end + line_start  - 4;
  }
  //Serial.println(error);
  
  
}

void read_sensor_values()
{
  sensor[0]=digitalRead(A0);
  sensor[1]=digitalRead(A1);
  sensor[2]=digitalRead(A2);
  sensor[3]=digitalRead(A3);
  sensor[4]=digitalRead(A4);
  
  if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1))
    error=4;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
    error=3;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==0))
    error=2;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0))
    error=1;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
    error=0;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
    error=-1;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
    error=-2;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
    error=-3;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
    error=-4;
  //else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  //  if(error==-4) error=-5;
  //  else error=5;

}

void calculate_pid()
{
    if (error!=previous_error)
    {
      P = error;
      I = I + 1e-6*(error-previous_error)*(micros()-previous_time);
      D = 1e6*(error-previous_error)/(micros()-previous_time);
     previous_error=error;
     previous_time=micros();
     PID_value = Kp*P + Ki*I + Kd*D; 
    }
}

void motor_control()
{
    if(go) {
      // Calculating the effective motor speed:
      left_motor_speed = constrain(initial_motor_speed+PID_value, -max_speed,max_speed);
      right_motor_speed = constrain(initial_motor_speed-PID_value, -max_speed,max_speed);
    } else {
      left_motor_speed = right_motor_speed = 0;
    }

    // set directions
    digitalWrite(pin_left_forward,left_motor_speed > 0);
    digitalWrite(pin_left_reverse,left_motor_speed < 0);

    digitalWrite(pin_right_forward,right_motor_speed > 0);
    digitalWrite(pin_right_reverse,right_motor_speed < 0);
    
    // set speeds
    analogWrite(pin_left_motor_enable,abs(left_motor_speed));
    analogWrite(pin_right_motor_enable,abs(right_motor_speed));
}
