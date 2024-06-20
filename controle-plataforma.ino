// ========================= DEFINES
#define Tms 25

#define MAX_ECHO_TRAVEL_TIME 20000
#define NM 15

#define PWM_LOW 70
#define PWM_HIGH 255
#define PWM_PIN 5

#define ECHO_PIN 11
#define TRIGGER_PIN 7
#define BAUD_RATE 115200
#define TRIGGER_PULSE_DURATION_US 12
#define SERIAL_TIMEOUT_MS 1

float ref = 0;
float erro = 0;
float erro_a = 0;
float i_erro = -17;
float d_erro = 0;
float kp = 2;
float ki = 200;
float kd = 0;

// ========================= GLOBALS
float _dp = 20;
float _de = 4;
float _hi = 18;
float _d0 = 25;
float _previous_angle = 0;
int _pwm = PWM_LOW;
float filtered_angle = 0;

// ========================== SENSOR
int getEchoTravelTime() {
  return pulseIn(ECHO_PIN, HIGH);
}

void sendTriggerPulse() {
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(TRIGGER_PULSE_DURATION_US);
  digitalWrite(TRIGGER_PIN, LOW);
}

float getSensorReading() {
  sendTriggerPulse();
  unsigned int duration = getEchoTravelTime();
  float distance = calcDistance(duration);
  return duration >= MAX_ECHO_TRAVEL_TIME ? -1.0 : distance;

}

// ========================== ANGLE
float calcDistance(unsigned int duration) {
  return duration * 0.01715;
}

float getAngle(float distance) {
  float cateto = distance - _dp + _de;
  return -asin(cateto / _hi) * 57.296 + 20;
}

float filter(float angle) {
  float filtered = 1.0/NM * angle + (1 - 1.0/NM) * _previous_angle;
  _previous_angle = filtered;
  return filtered;
}

// ================================== MAIN
void initPWM() {
  pinMode(PWM_PIN, OUTPUT);
  _pwm = PWM_LOW;
  analogWrite(PWM_PIN, _pwm);
}

void initSensor() {
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
}

void initSerial() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);
}

void setup() {
  initSensor();
  initPWM();
  initSerial();
  ki = ki*(Tms*10e-6);//calcula ki discretizado
  kd = kd/(Tms*10e-6);//calcula kd discretizado
}

void loop() {
  unsigned long t0 = millis();
  
  if (Serial.available() > 0) {
    ref = Serial.parseFloat();
  }

  float distance = getSensorReading();
  float angle = getAngle(distance);
  
  if (isnan(angle)) {
    filtered_angle = filter(_previous_angle);
  } else {
    filtered_angle = filter(angle);
  }

  erro = ref - filtered_angle;
  i_erro += erro;//////////calcula a integral do erro
  d_erro = erro - erro_a;//calcula a derivada do erro
  erro_a = erro;///////////atualiza o erro
  
  _pwm = (kp*erro)+(ki*i_erro)-(kd*d_erro);
  
  _pwm = constrain(_pwm, PWM_LOW, PWM_HIGH);
  analogWrite(PWM_PIN, _pwm);
  
  Serial.print(_pwm);
  Serial.print(" ");
  Serial.print(ref);
  Serial.print(" ");
  Serial.println(filtered_angle);

  while ((t0 + Tms) > millis()) {}
}
