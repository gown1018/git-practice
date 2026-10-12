#include <Servo.h>

// 핀 설정
#define PIN_LED   9
#define PIN_TRIG  12
#define PIN_ECHO  13
#define PIN_SERVO 10

// 초음파 센서 설정
#define SND_VEL        346.0
#define INTERVAL       25
#define PULSE_DURATION 10

// 유효 측정 범위 (mm)
#define _DIST_MIN 180
#define _DIST_MAX 360

#define TIMEOUT ((INTERVAL / 2.0) * 1000.0)
#define SCALE   (0.001 * 0.5 * SND_VEL)

// EMA: 실험하면서 조정
#define _EMA_ALPHA 0.3

// 보정한 서보 펄스 폭 (us)
#define _DUTY_MIN 600
#define _DUTY_MAX 2400
#define _DUTY_NEU ((_DUTY_MIN + _DUTY_MAX) / 2)

// HIGH에서 LED가 켜지는 배선 기준
#define LED_ON  LOW
#define LED_OFF HIGH

Servo myservo;

unsigned long last_sampling_time = 0;

// 최초 유효 측정 전에는 중립 위치 유지
float dist_prev = (_DIST_MIN + _DIST_MAX) / 2.0;
float dist_ema  = (_DIST_MIN + _DIST_MAX) / 2.0;

float USS_measure(int TRIG, int ECHO);

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(PIN_LED, LED_OFF);
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  Serial.begin(57600);
}

void loop() {
  unsigned long now = millis();

  if (now - last_sampling_time < INTERVAL) {
    return;
  }

  last_sampling_time = now;

  // 1. 원본 거리 측정
  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 2. 범위 판정
  // timeout은 0이므로 자동으로 유효 범위에서 제외됨
  bool valid = (dist_raw >= _DIST_MIN &&
                dist_raw <= _DIST_MAX);

  // 실제 측정값이 범위 안일 때만 LED 점등
  digitalWrite(PIN_LED, valid ? LED_ON : LED_OFF);

  // 3. 범위 필터
  float dist_filtered;

  if (valid) {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
  } else {
    dist_filtered = dist_prev;
  }

  // 4. EMA 필터
  dist_ema = _EMA_ALPHA * dist_filtered
           + (1.0 - _EMA_ALPHA) * dist_ema;

  // 5. 제어에 사용할 거리를 180~360mm로 제한
  float control_dist = constrain(
    dist_ema,
    (float)_DIST_MIN,
    (float)_DIST_MAX
  );

  // 6. 거리를 서보 펄스 폭으로 비례 변환
  // map()은 정수 계산이므로 소수 거리 보존을 위해
  // 동일한 비례식을 실수로 계산
  float ratio = (control_dist - _DIST_MIN)
              / (float)(_DIST_MAX - _DIST_MIN);

  int servo_pulse = (int)(
    _DUTY_MIN + ratio * (_DUTY_MAX - _DUTY_MIN) + 0.5
  );

  servo_pulse = constrain(
    servo_pulse, _DUTY_MIN, _DUTY_MAX
  );

  myservo.writeMicroseconds(servo_pulse);

  // 표시용 목표 각도. 실제 축 각도 측정값은 아님.
  float servo_angle = ratio * 180.0;

  // 7. 시리얼 출력
  Serial.print("Min:");
  Serial.print(_DIST_MIN);

  Serial.print(",raw:");
  Serial.print(dist_raw);

  Serial.print(",filtered:");
  Serial.print(dist_filtered);

  Serial.print(",ema:");
  Serial.print(dist_ema);

  Serial.print(",Angle:");
  Serial.print(servo_angle);

  Serial.print(",Max:");
  Serial.println(_DIST_MAX);
}

// 거리 측정: mm 단위 반환, timeout이면 0
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, TIMEOUT);

  return duration * SCALE;
}
