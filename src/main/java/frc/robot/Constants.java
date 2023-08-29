// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.Drivetrain.SwerveModule;

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

  // DASHBOARD NAMING
  public static final String DASHBOARD_TAB_NAME = "Dashboard";

  // DRIVETRAIN
  public static final Pose2d STARTING_POSE = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  public static final double STARTING_ANGLE_OFFSET = 0.0;

  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6477; // left-to-right distance between the drivetrain wheels
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.62865; // front-to-back distance between the drivetrain wheels

  // SWERVE DRIVE
  public static final double SWERVE_FULL_POWER_NEO_RPM = 5676.0; // DO NOT CHANGE
  public static final double SWERVE_MAX_DRIVE_VELOCITY = 5.0; // measured in meters/second (top speed of 20.1299853 m/s ?)
  public static final double SWERVE_MAX_ANGULAR_VELOCITY = SWERVE_MAX_DRIVE_VELOCITY
      / Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0); // measured in radians/second

  public static final double SWERVE_GEAR_REDUCTION = 1.0 / 6.75;
  public static final double SWERVE_WHEEL_DIAMETER = 0.4572; // measured in meters

  public static final int SWERVE_TOTAL_AMP_LIMIT = 300; // Default 480 Amps
  public static final double SWERVE_POWER_RAMP_RATE = 0.1; // Max change in motor power to reduce power spikes

  public static final double[] SWERVE_STEER_PID = { 1.0, 0.0, 0.0 }; // TODO

  // JOYSTICK DEAD-ZONES
  public static final double TWIST_DEADZONE = 0.3; // Joystick deadzone for turning
  public static final double STRAIGHT_DEADZONE = 0.1; // Joystick deadzone for turning
  public static final double THROTTLE_DEADZONE = 0.1; // Joystick deadzone for arm lifting

  // CHANNELS
  // In order of: front left, front right, back left, back right, where the battery is in the back
  public static final String[] SWERVE_MODULE_NAMES = { "FL", "FR", "BL", "BR" };
  public static final int[] CAN_SWERVE_DRIVE = { 10, 20, 30, 40 }; // TODO
  public static final int[] CAN_SWERVE_STEER = { 11, 21, 31, 41 }; // TODO
  public static final int[] CAN_SWERVE_STEER_ENCODER = { 12, 22, 32, 42 }; // TODO

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
