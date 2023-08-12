// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Enabled Systems
  public static final boolean ENABLE_DRIVE = true;
  public static final boolean ENABLE_LIMELIGHT = false;

  // TODO: make sure arm is fully down before starting robot

  // Drive Config


  // Joystick Dead-zones
  public static final double TWIST_DEADZONE = 0.3; // Joystick deadzone for turning
  public static final double STRAIGHT_DEADZONE = 0.1; // Joystick deadzone for turning
  public static final double THROTTLE_DEADZONE = 0.1; // Joystick deadzone for arm lifting


  // Channels
  public static final int CAN_LEFT_DRIVE_1 = 10; // 10 for Main Chassis, 1 For Test Chassis
  public static final int CAN_LEFT_DRIVE_2 = 28; // 28 for Main Chassis, 2 For Test Chassis
  public static final int CAN_RIGHT_DRIVE_1 = 7; // 7 for Main Chassis, 3 For Test Chassis
  public static final int CAN_RIGHT_DRIVE_2 = 27; // 27 for Main Chassis, 4 For Test Chassis

  public static final int USB_DRIVE_JOYSTICK = 0;
  public static final int USB_UTILITY_JOYSTICK = 1;

  // Limelight Config
  public static final String LIMELIGHT_NAME = "limelight";


  public static double mapPower(double power, double min, double max, double deadZone) {
    double sign = Math.signum(power);
    double absPower = Math.abs(power);

    if (absPower < deadZone) {
      return 0.0;
    } else {
      return mapNumber(absPower, deadZone, 1, min, max) * sign;
    }
  }

  public static double mapLimitedPower(int direction, double pos, double minPos, double maxPos, double minPower, double maxPower, double padding) {
    if (direction > 0) {
      if (pos > maxPos) {
        return 0.0;
      }
      if (pos > maxPos - padding) {
        return mapNumber(pos, maxPos - padding, maxPos, maxPower, minPower);
      } else {
        return maxPower;
      }
    } else if (direction < 0) {
      if (pos < minPos) {
        return 0.0;
      }
      if (pos < minPos + padding) {
        return -mapNumber(pos, minPos, minPos + padding, minPower, maxPower);
      } else {
        return -maxPower;
      }
    } else {
      return 0.0;
    }
  }

  public static double mapNumber(double x, double a, double b, double c, double d) {
    if (x < a) {
      return c;
    }
    if (x > b) {
      return d;
    }
    return (x - a) / (b - a) * (d - c) + c;
  }
}
