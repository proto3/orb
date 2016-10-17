#pragma GCC optimize ("3")

uint16_t freqCounter = 0;
uint16_t oldfreqCounter = 0;
uint16_t loop_time = 0;         //how fast is the main loop running

//motor control-------------------------------------------------------
#define PWM_A_MOTOR1 OCR2A
#define PWM_B_MOTOR1 OCR1B
#define PWM_C_MOTOR1 OCR1A

#define PWM_A_MOTOR0 OCR0A
#define PWM_B_MOTOR0 OCR0B
#define PWM_C_MOTOR0 OCR2B

// Space Vector PWM lookup table
// using uint8_t overflow for stepping
int8_t pwmSinMotor[] = {0,  5,  10, 16, 21, 27, 32, 37, 43, 48, 53, 59, 64, 69, 74, 79, 84, 89, 94, 99, 104,  109,
111,  112,  114,  115,  116,  118,  119,  120,  121,  122,  123,  123,  124,  125,  125,  126,  126,  126,  127,
127,  127,  127,  127,  127,  126,  126,  126,  125,  125,  124,  123,  123,  122,  121,  120,  119,  118,  116,
115,  114,  112,  111,  110,  112,  113,  114,  116,  117,  118,  119,  120,  121,  122,  123,  124,  124,  125,
125,  126,  126,  126,  127,  127,  127,  127,  127,  126,  126,  126,  125,  125,  124,  124,  123,  122,  121,
120,  119,  118,  117,  116,  114,  113,  112,  110,  106,  101,  97, 92, 87, 82, 77, 72, 66, 61, 56, 51, 45, 40,
35, 29, 24, 18, 13, 8,  2,  -2, -8, -13,  -18,  -24,  -29,  -35,  -40,  -45,  -51,  -56,  -61,  -66,  -72,  -77,
-82,  -87,  -92,  -97,  -101, -106, -110, -112, -113, -114, -116, -117, -118, -119, -120, -121, -122, -123, -124,
-124, -125, -125, -126, -126, -126, -127, -127, -127, -127, -127, -126, -126, -126, -125, -125, -124, -124, -123,
-122, -121, -120, -119, -118, -117, -116, -114, -113, -112, -110, -111, -112, -114, -115, -116, -118, -119, -120,
-121, -122, -123, -123, -124, -125, -125, -126, -126, -126, -127, -127, -127, -127, -127, -127, -126, -126, -126,
-125, -125, -124, -123, -123, -122, -121, -120, -119, -118, -116, -115, -114, -112, -111, -109, -104, -99,  -94,
-89,  -84,  -79,  -74,  -69,  -64,  -59,  -53,  -48,  -43,  -37,  -32,  -27,  -21,  -16,  -10,  -5, 0};

uint16_t MotorPower = 120; // 0 to 255, 255==100% power

//motor numbers
#define L_Motor 0
#define R_Motor 1

//motor pole angle, 0->255 overflow to loop after >>8 shift
uint16_t R_MotorStep = 0;
uint16_t L_MotorStep = 0;

// speed of motors, -127 to 127
int16_t R_Speed = 0;
int16_t L_Speed = 0;
//----------------------------------------------------------------------------//
void setup()
{
    Serial.begin(115200);

    Bl_Setup();

    while (Serial.available())
        Serial.read();
}
//----------------------------------------------------------------------------//
void loop()
{
    //run main loop every ~4ms
    if ((freqCounter & 0x07f) == 0)
    {
        // record when loop starts
        oldfreqCounter = freqCounter;

        R_Speed = 20;
        L_Speed = 1;
        MotorPower = 60;
        MoveMotors(R_Motor, (uint8_t)(R_MotorStep >> 8), MotorPower);
        MoveMotors(L_Motor, (uint8_t)(L_MotorStep >> 8), MotorPower);

        //calculate loop time
        if (freqCounter > oldfreqCounter)
        {
            if (loop_time < (freqCounter - oldfreqCounter))
                loop_time = freqCounter - oldfreqCounter;
        }
    }
}
//----------------------------------------------------------------------------//
void Bl_Setup()
{
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  cli();//stop interrupts

  //timer setup for 31.250KHZ phase correct PWM
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(CS00);
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  // enable Timer 1 interrupt
  TIMSK1 = 0;
  TIMSK1 |= _BV(TOIE1);
  // disable arduino standard timer interrupt
  TIMSK0 &= ~_BV(TOIE1);

  sei(); // Start Interrupt

  //turn off all PWM signals
  OCR2A = 0;  //11  APIN
  OCR2B = 0;  //D3
  OCR1A = 0;  //D9  CPIN
  OCR1B = 0;  //D10 BPIN
  OCR0A = 0;  //D6
  OCR0B = 0;  //D5
}
//----------------------------------------------------------------------------//
void MoveMotors(uint8_t motorNumber, uint8_t posStep, uint16_t power)
{
  uint16_t pwm_a;
  uint16_t pwm_b;
  uint16_t pwm_c;

  // lookup sine values from table with 120deg. offsets
  pwm_a = pwmSinMotor[(uint8_t)posStep];
  pwm_b = pwmSinMotor[(uint8_t)(posStep + 85)];
  pwm_c = pwmSinMotor[(uint8_t)(posStep + 170)];

  // scale motor power
  pwm_a = power * pwm_a;
  pwm_a = pwm_a >> 8;
  pwm_a += 128;

  pwm_b = power * pwm_b;
  pwm_b = pwm_b >> 8;
  pwm_b += 128;

  pwm_c = power * pwm_c;
  pwm_c = pwm_c >> 8;
  pwm_c += 128;

  // set motor pwm variables
  if (motorNumber == 0)
  {
    PWM_A_MOTOR0 = (uint8_t)pwm_a;
    PWM_B_MOTOR0 = (uint8_t)pwm_b;
    PWM_C_MOTOR0 = (uint8_t)pwm_c;
  }

  if (motorNumber == 1)
  {
    PWM_A_MOTOR1 = (uint8_t)pwm_a;
    PWM_B_MOTOR1 = (uint8_t)pwm_b;
    PWM_C_MOTOR1 = (uint8_t)pwm_c;
  }
}
//----------------------------------------------------------------------------//
// minimize interrupt code length
// is called every 31.875us (510 clock cycles)  ???????
ISR( TIMER1_OVF_vect )
{
  //every 32 count of freqCounter ~1ms
  freqCounter++;

  if ((freqCounter & 0x01) == 0)
  {
    R_MotorStep += R_Speed;
    L_MotorStep += L_Speed;
  }
}
//----------------------------------------------------------------------------//
