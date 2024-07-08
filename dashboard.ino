/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <https://unlicense.org>

2024 Matthew Wallace
Adafruit Feather Sense "Dashboard"
*/

#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SH110X.h>
#include <PDM.h>
#include <Wire.h>

#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5

#define SHOW_CONFIG_TIME 3000  // millis

#define PORTRAIT = 0
#define LANDSCAPE = 1
#define UPSIDE_DOWN_PORTRAIT = 2
#define UPSIDE_DOWN_LANDSCAPE = 3

Adafruit_APDS9960 apds9960;      // proximity, light, color, gesture
Adafruit_BMP280 bmp280;          // temperautre, barometric pressure
Adafruit_LIS3MDL lis3mdl;        // magnetometer
Adafruit_LSM6DS3TRC lsm6ds3trc;  // accelerometer, gyroscope
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_SHT31 sht30;  // humidity
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

uint8_t proximity;
uint16_t r, g, b, c;
float temperature, pressure, altitude;
float magnetic_x, magnetic_y, magnetic_z;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float humidity;
int32_t mic;
long int check_array[6] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };

extern PDMClass PDM;
short sampleBuffer[256];   // buffer to read samples into, each sample is 16-bits
volatile int samplesRead;  // number of samples read

bool new_rev = true;

unsigned int updateSpeed = 1000;
unsigned long lastUpdateTime = 0;
unsigned long lastConfigTime = 0;
unsigned int infoPage = 0;
bool buttonReleased = true;

/*****************************************************************
 * Setup
 *****************************************************************/

void setup(void)
{
  Serial.begin(115200);

  setupDevices();
}

void setupDevices()
{
  setupAccelerometer();

  sht30.begin();
  PDM.onReceive(onPDMdata);
  PDM.begin(1, 16000);

  setupDisplay();
}

void setupAccelerometer()
{
  // initialize the sensors
  apds9960.begin();
  apds9960.enableProximity(true);
  apds9960.enableColor(true);
  bmp280.begin();
  lis3mdl.begin_I2C();
  lsm6ds33.begin_I2C();
  // check for readings from LSM6DS33
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  accel_array[0] = accel.acceleration.x;
  accel_array[1] = accel.acceleration.y;
  accel_array[2] = accel.acceleration.z;
  accel_array[3] = gyro.gyro.x;
  accel_array[4] = gyro.gyro.y;
  accel_array[5] = gyro.gyro.z;
  // if all readings are empty, then new rev
  for (int i = 0; i < 5; i++)
  {
    if (accel_array[i] != check_array[i])
    {
      new_rev = false;
      break;
    }
  }
  // and we need to instantiate the LSM6DS3TRC
  if (new_rev)
  {
    lsm6ds3trc.begin_I2C();
  }
}

void setupDisplay()
{
  display.begin(0x3C, true);
  display.clearDisplay();
  display.setRotation(1);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.display();

  // buttons a,b,c on the display.
  // we're using these for refresh control,
  // and page switching
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
}

/*****************************************************************
 * Loop
 *****************************************************************/

void loop(void)
{
  handleControls();

  unsigned long time = millis();
  if (time - lastUpdateTime >= updateSpeed)
  {
    lastUpdateTime = time;
    readActiveSensors();
    render();
  }

  yield();
}

void handleControls()
{
  // handle button presses
  if (!digitalRead(BUTTON_B))
  {
    lastConfigTime = millis();
    render();
  }

  int factor = !digitalRead(BUTTON_A)? 1 : !digitalRead(BUTTON_C)? -1 : 0;
  if (factor != 0)
  {
    if (showUpdateConfig()) // change update speed if config menu is open
    {
      updateSpeed += 50 * factor;
      lastConfigTime = millis();
    }
    else // switch pages
    {
      if (!buttonReleased) return; // don't continuously switch pages... 

      int last = infoPage;
      infoPage -= factor; // this calculation has to be done outside of constrain()!!! (some quirk with the function, not sure why)
      infoPage = constrain(infoPage, 0, 2);
      if (last != infoPage)
        readActiveSensors(); // new page of sensors needs to calculated now
    }

    buttonReleased = false;
    render(); // some change was made here, have to re-display.
  }
  else
    buttonReleased = true;
}

bool showUpdateConfig()
{
  unsigned long time = millis();
  return (time > SHOW_CONFIG_TIME) && (time - lastConfigTime < SHOW_CONFIG_TIME);
}

/*****************************************************************
 * Sensors
 *****************************************************************/

/*
 * Only Read sensors that are relevant to the current
 * displaying page. There is no need to waste resources
 * reading sensors that we aren't even using.
 */
void readActiveSensors()
{
  // acceleration always needs to be read
  // for display orientation
  lsm6ds33.readAcceleration(accel_x, accel_y, accel_z);

  switch (infoPage)
  {
    default:
    case 0:
    {
      temperature = bmp280.readTemperature();
      pressure = bmp280.readPressure();
      altitude = bmp280.readAltitude(1013.25);

      while (!apds9960.colorDataReady())
        delay(5);

      apds9960.getColorData(&r, &g, &b, &c);

      if (!showUpdateConfig())
        proximity = apds9960.readProximity();

      break;
    }
    case 1:
    {
      lsm6ds33.readGyroscope(gyro_x, gyro_y, gyro_z);
      break;
    }
    case 2:
    {
      samplesRead = 0;
      mic = getPDMwave(4000);

      if (!showUpdateConfig())
      {
        lis3mdl.read();
        magnetic_x = lis3mdl.x;
        magnetic_y = lis3mdl.y;
        magnetic_z = lis3mdl.z;

        humidity = sht30.readHumidity();
      }

      break;
    }
  }
}

int32_t getPDMwave(int32_t samples)
{
  short minwave = 30000;
  short maxwave = -30000;

  while (samples > 0)
  {
    if (!samplesRead)
    {
      yield();
      continue;
    }
    for (int i = 0; i < samplesRead; i++)
    {
      minwave = min(sampleBuffer[i], minwave);
      maxwave = max(sampleBuffer[i], maxwave);
      samples--;
    }
    // clear the read count
    samplesRead = 0;
  }
  return maxwave - minwave;
}

void onPDMdata()
{
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

/*****************************************************************
 * Render
 *****************************************************************/

void render()
{
  display.clearDisplay();
  display.setCursor(0, 0);

  rotateDisplay();

  switch (infoPage)
  {
    default:
    case 0:  // page 1 start
    {
      if (!displayingConfig())
      {
        display.print("Prox: ");
        display.println(apds9960.readProximity());
        display.print("Red: ");
        display.println(r);
        display.print("Green: ");
        display.println(g);
        display.print("Blue: ");
        display.println(b);
      }

      display.print("Clear: ");
      display.println(c);
      display.print("Temp: ");
      display.print(temperature);
      display.println(" C");
      display.print("atm: ");
      display.println(pressure);
      display.print("Alt: ");
      display.print(altitude);
      display.print(" m");

      break;
    }
    case 1:  // page 2 start
    {
      if (!displayingConfig())
      {
        display.print("Accel X: ");
        display.println(accel_x);
        display.print("Accel Y: ");
        display.println(accel_y);
        display.print("Accel Z: ");
        display.println(accel_z);
        // display.println(" m/s^2");

        display.print("Gyro X: ");
        display.println(gyro_x);
      }
      display.print("Gyro Y: ");
      display.println(gyro_y);
      display.print("Gyro Z: ");
      display.println(gyro_z);
      // display.println(" dps");

      break;
    }
    case 2:
    {
      if (!displayingConfig())
      {
        display.print("Mag X: ");
        display.println(magnetic_x);
        display.print("Mag Y: ");
        display.println(magnetic_y);
        display.print("Mag Z: ");
        display.println(magnetic_z);
        // display.println(" uTesla");
        display.print("Humidity: ");
        display.print(humidity);
        display.println("%");
      }

      display.print("Mic: ");
      display.println(mic);
    }
  }

  display.display();
}

void rotateDisplay()
{
  int x = round(accel_x);

  switch(x)
  {
    default:
    case 0:
    {
      x = PORTRAIT;
      break;
    }
    case 1:
    {
      x = UPSIDE_DOWN_PORTRAIT;
      break;
    }
    case -1:
    {
      x = PORTRAIT;
      break;
    }
  }

  display.setRotation(x);
}

bool displayingConfig()
{
  if (!showUpdateConfig()) return false;

  display.print("\nUpdate Speed: ");
  display.print(updateSpeed);
  display.print("\n\n\n");

  return true;
}
