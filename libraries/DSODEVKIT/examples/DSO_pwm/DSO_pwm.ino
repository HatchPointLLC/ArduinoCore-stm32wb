
void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(BUSA_PWM0, OUTPUT);
  pinMode(BUSA_PWM1, OUTPUT);
  pinMode(BUSB_PWM0, OUTPUT);
  pinMode(BUSB_PWM1, OUTPUT);

  Serial.begin(9600); // send and receive at 9600 baud
}

void flash_led_task(void)
{
	static const uint32_t run_every_ms = 250;
	static uint32_t last_run = 0;
	static uint32_t ms = 0;
  static uint8_t led_state = 0;

	ms = millis();

	if((ms-last_run)>run_every_ms)
	{
	  digitalWrite(LED_BUILTIN, led_state);
    led_state^=0x01;
		last_run = ms;
	}
}

void logger_task(void)
{
	static const uint32_t run_every_ms = 1000;
	static uint32_t last_run = 0;
	static uint32_t ms = 0;
  static char buffer[256] = {0};
  static uint32_t logger_loops = 0;

	ms = millis();

	if((ms-last_run)>run_every_ms)
	{
    sprintf(buffer, "[%lu ms] Logger output %lu loops\n", millis(), logger_loops);
    Serial.print(buffer);
    last_run = ms;
    logger_loops++;
	}
}

void update_temp_task (void)
{
	static const uint32_t run_every_ms = 1000;
	static uint32_t last_run = 0;
	static uint32_t ms = 0;
  static char buffer[256] = {0};
  uint16_t temp_k = 0;

  ms = millis();

	if((ms-last_run)>run_every_ms)
	{
    // TO DO: Read from ADC and convert to temp
    temp_k = 273;
    sprintf(buffer, "[%lu ms] Temp outout %lu K\n", millis(), temp_k);
    Serial.print(buffer);
    last_run = ms;
  }
}

void update_pid_task(void)
{
	static const uint32_t run_every_ms = 25;
	static uint32_t last_run = 0;
	static uint32_t ms = 0;
  static uint8_t pid_loops = 0;

  ms = millis();

	if((ms-last_run)>run_every_ms)
	{
    analogWrite(BUSA_PWM0, (uint8_t)pid_loops);
    analogWrite(BUSA_PWM1, (uint8_t)pid_loops*2);
    analogWrite(BUSB_PWM0, (uint8_t)pid_loops*4);
    analogWrite(BUSB_PWM1, (uint8_t)pid_loops*6);
    last_run = ms;
    pid_loops++;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  flash_led_task();
  logger_task();
  update_pid_task();
  update_temp_task();
}
