#include <MKRIMU.h>
#include <stdio.h>  // required for snprintf

// High-resolution timestamp
volatile uint32_t lastMicros = 0;
volatile uint64_t extendedMicros = 0;

void updateMicros() {
  uint32_t now = micros();
  if (now < lastMicros) {
    extendedMicros += (uint64_t)1 << 32;
  }
  lastMicros = now;
}

double getSecondsSinceBoot() {
  noInterrupts();
  updateMicros();
  uint64_t fullMicros = extendedMicros + lastMicros;
  interrupts();
  return fullMicros / 1e6;
}
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting...");

  if (!IMU.begin()) {
    Serial.println("IMU.begin() failed");
  } else {
    Serial.println("IMU.begin() succeeded");
  }
}
void loop() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    bool acc_ok = IMU.readAcceleration(ax, ay, az);
    bool gyro_ok = IMU.readGyroscope(gx, gy, gz);
    bool mag_ok = false;

    if (!acc_ok || !gyro_ok) {
      Serial.println("{\"error\":\"sensor read failed\"}");
      return;
    }

    double t = getSecondsSinceBoot();

    if (IMU.magneticFieldAvailable()) {
      mag_ok = IMU.readMagneticField(mx, my, mz);
    }

    String json = "{\"t\":" + String(t, 6) +
                  ",\"acc\":[" + String(ax, 4) + "," + String(ay, 4) + "," + String(az, 4) + "]" +
                  ",\"gyro\":[" + String(gx, 4) + "," + String(gy, 4) + "," + String(gz, 4) + "]";

    if (mag_ok) {
      json += ",\"mag\":[" + String(mx, 1) + "," + String(my, 1) + "," + String(mz, 1) + "]";
    }

    json += "}";
    Serial.println(json);
  }
}
