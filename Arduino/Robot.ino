const float TOL = ANGLE_TOL;
  const int STABLE = STABLE_CNT;
  const unsigned long TIMEOUT_MS = 9000;

  bool overshot = false;

  unsigned long t0 = millis();
  int stableCnt = 0;

  int cmdMain = (phi > 0) ? RotateRight : RotateLeft;
  int cmdOpp  = (phi > 0) ? RotateLeft  : RotateRight;

  // startup kick
  Motor(cmdMain, FAST_PWM);
  if (!smartDelay(60)) { Motor(Stop, 0); return false; }
  Motor(Stop, 0);
  if (!smartDelay(50)) return false;

  int cmd = cmdMain;

  const float OVERSHOOT_DEG = 6.0;

  mpu.update();
  float err0 = norm180(targetAngle - mpu.getAngleZ());
  int sign0 = (err0 >= 0) ? +1 : -1;

  while (true) {
    if (shouldStopNow()) return false;

    mpu.update();
    float z = mpu.getAngleZ();
    float err = norm180(targetAngle - z);
    float ae = abs(err);
    int sign = (err >= 0) ? +1 : -1;

    if (millis() - t0 > TIMEOUT_MS) {
      Serial.println("Rotate timeout");
      Motor(Stop, 0);
      return false;
    }

    // within tolerance
    if (ae <= TOL) {
      stableCnt++;
      Motor(Stop, 0);
      debugRotate(z, targetAngle, err, 0, Stop);

      if (stableCnt >= STABLE) {
        return true;
      }

      if (!smartDelay(25)) return false;
      continue;
    } else {
      stableCnt = 0;
    }

    if (!overshot && (sign != sign0) && (ae > OVERSHOOT_DEG)) {
      overshot = true;
    }

    cmd = overshot ? cmdOpp : cmdMain;

    // --- Zones ---
    if (ae < 10.0) {

      // MICRO zone: <6°
      if (ae < 6.0) {
        const int MICRO_STEPS    = 3;
        const int MICRO_KICK_MS  = 8;
        const int MICRO_HOLD_MS  = 20;
        const int MICRO_PAUSE_MS = 25;

        for (int i = 0; i < MICRO_STEPS; i++) {
          if (shouldStopNow()) return false;

          mpu.update();
          float err2 = norm180(targetAngle - mpu.getAngleZ());
          float ae2  = abs(err2);
          if (ae2 <= TOL) break;

          int cmd2 = (err2 >= 0) ? cmdMain : cmdOpp;
          if (overshot) cmd2 = cmdOpp;

          Motor(cmd2, FINE_PWM);
          if (!smartDelay(MICRO_KICK_MS)) return false;

          Motor(cmd2, MIN_PWM);
          if (!smartDelay(MICRO_HOLD_MS)) return false;

          Motor(Stop, 0);
          if (!smartDelay(MICRO_PAUSE_MS)) return false;

          mpu.update();
          float errAfter = norm180(targetAngle - mpu.getAngleZ());
          if ((errAfter >= 0) != (err2 >= 0)) break;
        }

        debugRotate(mpu.getAngleZ(), targetAngle,
                    norm180(targetAngle - mpu.getAngleZ()),
                    MIN_PWM, cmd);
        continue;
      }

      // NEAR zone: 6..10°
      const int NEAR_KICK_MS  = 12;
      const int NEAR_HOLD_MS  = 40;
      const int NEAR_PAUSE_MS = 35;

      Motor(cmd, FINE_PWM);
      if (!smartDelay(NEAR_KICK_MS)) return false;

      Motor(cmd, MIN_PWM);
      if (!smartDelay(NEAR_HOLD_MS)) return false;

      Motor(Stop, 0);
      if (!smartDelay(NEAR_PAUSE_MS)) return false;

      debugRotate(z, targetAngle, err, MIN_PWM, cmd);
      continue;
    }

    // MID zone: 10..25°
    if (ae < 25.0) {
      Motor(cmd, MID_PWM);
      if (!smartDelay(12)) return false;
      debugRotate(z, targetAngle, err, MID_PWM, cmd);
      continue;
    }

    // FAR zone: >=25°
    Motor(cmd, FAST_PWM);
    if (!smartDelay(10)) return false;
    debugRotate(z, targetAngle, err, FAST_PWM, cmd);
    continue;
  }
}

// ---------------- Optional ----------------

void detectYawSign() {
  mpu.update();
  float z0 = mpu.getAngleZ();

  Motor(RotateRight, max(MIN_ROT_PWM, 160));
  delay(350);
  Motor(Stop, 0);
  delay(150);

  mpu.update();
  float z1 = mpu.getAngleZ();
  float dz = norm180(z1 - z0);

  bool RIGHT_INCREASES_ANGLEZ = (dz > 0);
  Serial.print("Yaw sign test: dz=");
  Serial.print(dz, 2);
  Serial.print(" => RIGHT_INCREASES_ANGLEZ=");
  Serial.println(RIGHT_INCREASES_ANGLEZ ? "true" : "false");
}