void calibrate_IMU(){
  const int samples = 200;

  float z = 0;

  for (int i = 0; i < samples; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      z = a.acceleration.z;
      
      z_DRIFT_CORRECTION += z;

      delay(5);
  }
  z_DRIFT_CORRECTION /= samples;
  }

void setup_IMU() {
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

  Serial.println("Calibrating...");
  calibrate_IMU();
  Serial.println("Z DRIFT CORRECTION: ");
  Serial.print(z_DRIFT_CORRECTION);

  Serial.println();
  
  delay(100);
}

// update acceleration values
void update_accels(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  a_z = a.acceleration.z - z_DRIFT_CORRECTION;

  // high-pass filter (gets rid of *gravity* :>)
  z_baseline += HP_COEFF * (a_z - z_baseline);
  float z_hp = a_z - z_baseline;

  rms_counter += 1;
  running_sum += sq(z_hp);

  if (rms_counter >= RMS_SAMPLES){
    mean = running_sum / RMS_SAMPLES;
    RMS = sqrt(mean);
    rms_counter = 0;
    running_sum = 0;
  }
}

// if 8/10 of the last RMS readings are above the threshold (400/500ms), vibration is detected!!!
bool vibration_detect(){
  bool vib_in_range = (RMS < THRESHOLD_HIGH && RMS > THRESHOLD_LOW);
  
  // remove oldest sample
  if (history[history_index]){
      vibration_ct--;
  }

  // store new sample
  history[history_index] = vib_in_range;

  if (vib_in_range)
      vibration_ct++;

  // increment and wrap-around if needed
  history_index = (history_index + 1) % WINDOW;

  // the result!
  return (vibration_ct >= REQUIRED);
}

void reset_all_vibration_vars(){
  running_sum = 0;
  rms_counter = 0;
  RMS = 0;

  memset(history, 0, sizeof(history));
  vibration_ct = 0;
  history_index = 0;

  vibration_detected = false;
}