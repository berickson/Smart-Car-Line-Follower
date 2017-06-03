#include <EEPROM.h>
#include <SoftwareSerial.h>

const int options_magic_number = 32568;
const int eeprom_address = 0;
struct Options {
  int magic = options_magic_number;
  float Kp = 100;
  float Ki = 0;
  float Kd = 1.0;
  int base_speed = 200;
  int max_speed = 255;
} options;

float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0;
float delta_error = 0; // last change in error
float delta_t = 999;
float previous_time=0;
int sensor[5]={0, 0, 0, 0, 0};

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

  read_options_from_eeprom();
  
 Serial.begin(115200);
}

bool read_options_from_eeprom() {
  Options new_options;
  EEPROM.get(eeprom_address, new_options);
  if(new_options.magic == options_magic_number) {
    options = new_options;
    return true;
  }
  return false;
}

void write_options_to_eeprom() {
  EEPROM.put(eeprom_address, options);
}


void execute_command(char * s) {
  
  usb.print("executing command ");
  usb.println(s);
  if(s[0] == 'h') {
      usb.println("h - help");
      usb.println("g - go");
      usb.println("s - stop");
      usb.println("v{0-255} - set velocity");
      usb.println("m{0-255} - set max velocity for turns");
      usb.println("p{integer} - set Kp for PID");
      usb.println("i{integer} - set Ki for PID x 0.01");
      usb.println("d{integer} - set Kd for PID x 0.01");
      usb.println("r - read options from eeprom");
      usb.println("w - write options to eeprom");
      usb.println("o - show currrent options");
    
  }
  if(s[0] == 'g') {
      go = true;
      usb.println("go");
  }
  if(s[0] == 's') {
      go = false;
      usb.println("stop\n");
  }
  if(s[0] == 'v') {
    options.base_speed=atoi(s+1);
    usb.print("set base_speed to ");
    usb.println(options.base_speed);
  }
  if(s[0] == 'm') {
    options.max_speed=atoi(s+1);
    usb.print("set max_speed to ");
    usb.println(options.max_speed);
  }
  if(s[0] == 'p') {
    options.Kp=atoi(s+1);
    usb.print("set Kp to ");
    usb.println(options.Kp);
  }
  if(s[0] == 'i') {
    options.Ki=atoi(s+1);
    usb.print("set Ki to ");
    usb.println(options.Kp);
  }
  if(s[0] == 'd') {
    options.Kd=atoi(s+1)/100.;
    usb.print("set Kd to ");
    usb.println(options.Kd);
  }
  if(s[0] == 'r') {
    if( read_options_from_eeprom() ) {
      usb.println("loaded values from eeprom");
    } else {
      usb.println("eeprom options read failed");
    }
  }
  if(s[0] == 'w') {
    write_options_to_eeprom();
    usb.println("options written to eeprom");
  }
  if(s[0] == 'o') {
    usb.println("current option settings");
    usb.print("Kp: ");
    usb.println(options.Kp);
    usb.print("Ki: ");
    usb.println(options.Ki);
    usb.print("Kd: ");
    usb.println(options.Kd);
    usb.print("base_speed: ");
    usb.println(options.base_speed);
    usb.print("max_speed: ");
    usb.println(options.max_speed);
  }
  usb.flush();
}


void process_commands() {
  static char command_buffer[80] = {0};
  static int command_length = 0;
  
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
        command_buffer[0] = 0;
        command_length = 0;
      }
      command_buffer[command_length] = c;
      command_buffer[++command_length]=0;
    }
  }
}

void loop()
{

  process_commands();
  get_line_position();
  calculate_pid();
  motor_control();
}

void get_line_position() {
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

  // must have at least one line reading to be ok
  if(line_end == -1) {
    return;
  }
  
  // reading must be contiguous block of sensors to be ok
  for(int i = line_start; i < line_end; i++) {
    if(sensor[i]==false) {
      return;
    }
  }

  // can't be all black
  if( line_start == 0 && line_end == max_sensor) {
    return;
  }

  error = line_end + line_start  - 4;
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
}

void calculate_pid()
{
    if (error!=previous_error)
    {
      delta_t = micros()-previous_time;
      delta_error = error-previous_error;
      P = error;
      I = I + 1e-6*(delta_error)*(delta_t);
      D = 1e6*(error-previous_error)/(delta_t);
     previous_error=error;
     previous_time=micros();
    }


    // allow D term to settle if error hasn't changed recently
    if(micros()-previous_time > delta_t) {
      D = 1e6*(delta_error) / (micros() -previous_time);
    }
    PID_value = options.Kp*P + options.Ki*I + options.Kd*D; 
}

void motor_control()
{
    if(go) {
      // Calculating the effective motor speed:
      left_motor_speed = constrain(options.base_speed+PID_value, -options.max_speed, options.max_speed);
      right_motor_speed = constrain(options.base_speed-PID_value, -options.max_speed, options.max_speed);
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
