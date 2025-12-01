#include <Wire.h>
#include <WiFi.h>
#include <PID_v1.h>
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>

#define LCD_COLUMNS 20
#define LCD_ROWS 4
#define LCD_I2C_ADDRESS 0x27

#define V_SOLAR_IN 0
#define V_SOLAR_OUT 1
#define NTC_TEMP 3

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);

Adafruit_ADS1115 ads;

const int
  buck_pin = 14,
  fan_pin = 15,
  ads_alert = 13,

  v_sample = 3,
  ntc_sample = 50,

  pwm_step = 5;

int
  duty_int = 0,
  temp_c = 0;

const float
  v_program = 12.0,
  ntc_v = 3.3,
  v_in_divider_ratio = 14.3334,
  v_out_divider_ratio = 9.3334;

float
  v_in = 0.0,
  v_in_avg = 0.0,
  v_out = 0.0,
  v_out_avg = 0.0,
  ntc_avg = 0.0,
  ntc_value = 0.0,
  ntc_c = 0.0,
  steinhart = 0.0;

unsigned long
  prev_scn = 0,
  prev_sens = 0,
  current_scn = 0,
  current_sens = 0;

const long
  scn_interval = 1000,
  sens_interval = 50,

  ntc_normal = 10000,
  temp_normal = 25,
  b_coeff = 3950;

void setup() {
  ledcAttach(buck_pin, 32000, 11);
  ledcWrite(buck_pin, 0);

  Serial.begin(115200);
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Firmware : Beta");
  lcd.setCursor(0, 1);
  lcd.print("Mod Date : 30-Nov-25");
  delay(3000);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Checking...");
  lcd.setCursor(0, 1);
  lcd.print("ADS : ");
  delay(3000);

  if (ads.begin()) {
    ads.setGain(GAIN_ONE);
    lcd.print("Done");
  } else {
    lcd.print("Fail");
  }

  pinMode(ads_alert, INPUT);

  lcd.setCursor(0, 3);
  lcd.print("Setup Complete.");
  delay(2000);
  lcd.clear();
}

void loop() {
  if (v_out > v_program) {
    duty_int = duty_int - pwm_step;
  } else if (v_out < v_program) {
    duty_int = duty_int + pwm_step;
  }
    
  if (duty_int < 0) {duty_int = 0;}
  else if (duty_int > 2000) {duty_int = 2000;}

  duty_int = 1024;
  ledcWrite(buck_pin, duty_int);

  current_scn = millis();
  if (current_scn - prev_scn >= scn_interval) {
    prev_scn = current_scn;
    display();
  }

  current_sens = millis();
  if (current_sens - prev_sens >= sens_interval) {
    prev_sens = current_sens;
    read_sensors();
  }
}

void display() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("====================");
  lcd.setCursor(0, 1);
  lcd.print("Vin: ");
  lcd.print(v_in);
  lcd.setCursor(0, 2);
  lcd.print("Vout: ");
  lcd.print(v_out);
  lcd.setCursor(11, 3);
  lcd.print("Temp: ");
  lcd.print(temp_c);
  lcd.setCursor(0, 3);
  lcd.print("Duty: ");
  lcd.print(duty_int);
}

void read_sensors() {
  v_in_avg = 0.0;
  v_out_avg = 0.0;
  ntc_avg = 0.0;

  for (int v = 0; v < v_sample; v++) {
    v_in_avg = v_in_avg + ads.computeVolts(ads.readADC_SingleEnded(V_SOLAR_IN));
    v_out_avg = v_out_avg + ads.computeVolts(ads.readADC_SingleEnded(V_SOLAR_OUT));
  }

  for (int t = 0; t < ntc_sample; t++) {
    ntc_avg = ntc_avg + ads.computeVolts(ads.readADC_SingleEnded(NTC_TEMP));
  }

  v_in = (v_in_avg / v_sample) * v_in_divider_ratio;
  v_out = (v_out_avg / v_sample) * v_out_divider_ratio;

  ntc_avg = ntc_avg / ntc_sample;

  if (ntc_avg < ntc_v) {
    ntc_value = ntc_normal * ((ntc_v - ntc_avg) / ntc_avg);
  } else {
    ntc_value = ntc_normal;
  }

  steinhart = ntc_value / ntc_normal;
  steinhart = log(steinhart);
  steinhart /= b_coeff;
  steinhart += 1.0 / (temp_normal + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;
  temp_c = steinhart;
}
