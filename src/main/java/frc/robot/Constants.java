// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.Swerve.SwerveModule;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // ENABLED SUBSYSTEMS
  public static final boolean ENABLE_DRIVE = true;
  public static final boolean ENABLE_LIMELIGHT = false;

  // SWERVE DRIVE
  public static final Pose2d STARTING_POSE = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));

  public static final double DRIVE_MOTOR_MAX_RPM = 5676; // DO NOT CHANGE
  public static final double MAX_DRIVE_VELOCITY = SwerveModule.MotorRPMtoMetersPerSecond(DRIVE_MOTOR_MAX_RPM) * 0.1; // measured in meters/second (top speed of 20.1299853 m/s ?)

  public static final double SWERVE_MODULE_SPACING_X = 0.6477; // measured in meters
  public static final double SWERVE_MODULE_SPACING_Y = 0.62865; // measured in meters

  public static final double SWERVE_GEAR_RATIO = 6.75; // 1:6.75
  public static final double SWERVE_WHEEL_DIAMETER = 0.4572; // meters

  public static final double[] SWERVE_STEER_PID = { 1.0, 0.0, 0.0 }; // TODO

  public static final double STARTING_ANGLE_OFFSET = 0.0;

  // JOYSTICK DEAD-ZONES
  public static final double TWIST_DEADZONE = 0.3; // Joystick deadzone for turning
  public static final double STRAIGHT_DEADZONE = 0.1; // Joystick deadzone for turning
  public static final double THROTTLE_DEADZONE = 0.1; // Joystick deadzone for arm lifting

  // CHANNELS
  // In order of: front left, front right, back left, back right, where the battery is in the back
  public static final int[] STEER_CAN_IDS = { 0, 0, 0, 0 }; // TODO
  public static final int[] DRIVE_CAN_IDS = { 0, 0, 0, 0 }; // TODO
  public static final int[] CANCODER_CAN_IDS = { 0, 0, 0, 0 }; // TODO

  public static final int USB_DRIVE_JOYSTICK = 0;
  public static final int USB_UTILITY_JOYSTICK = 1;

  // LIMELIGHT CONFIG
  public static final String LIMELIGHT_NAME = "limelight";

  public static double mapNumber(double x, double a, double b, double c, double d) {
    if (x < a) {
      return c;
    }
    if (x > b) {
      return d;
    }
    return (x - a) / (b - a) * (d - c) + c;
  }

  public static double angleDist(double alpha, double beta) {
    double phi = Math.abs(beta - alpha) % 360.0; // This is either the distance or 360 - distance
    double distance = phi > 180.0 ? 360.0 - phi : phi;
    return distance;
  }
}
