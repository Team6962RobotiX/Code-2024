// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Drivetrain.SwerveDrive;
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

  public static final class DriveConstants {

    // DRIVETRAIN
    public static final Pose2d STARTING_POSE = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final double STARTING_ANGLE_OFFSET = 0.0;

    public static final double TRACKWIDTH_METERS = 0.6477; // left-to-right distance between the drivetrain wheels
    public static final double WHEELBASE_METERS = 0.62865; // front-to-back distance between the drivetrain wheels

    // PHYSICAL
    public static final double GEAR_REDUCTION = 1.0 / 6.75;
    public static final double WHEEL_DIAMETER = 0.4572; // measured in meters
    public static final double PHYSICAL_MAX_NEO_RPM = 5676.0; // DO NOT CHANGE

    public static final double RPM_TO_VELOCITY_CONVERSION_FACTOR = DriveConstants.GEAR_REDUCTION / 60.0 * DriveConstants.WHEEL_DIAMETER * Math.PI;
    public static final double PHYSICAL_MAX_VELOCITY = PHYSICAL_MAX_NEO_RPM * RPM_TO_VELOCITY_CONVERSION_FACTOR;
    public static final double VELOCITY_DEADZONE = 0.05; // speed at which we stop moving all together

    // ELECTRONICS
    public static final int TOTAL_CURRENT_LIMIT = 300; // Default 480 Amps
    public static final double POWER_RAMP_RATE = 0.1; // Max change in motor power to reduce power spikes

    // PID
    public static final double[] STEER_PID = { 1.0, 0.0, 0.0 }; // TODO

    // TELEOPERATED
    public static final double TELEOP_MAX_VELOCITY = 5.0; // measured in meters/sec (top speed of 20.1299853 m/s ?)
    public static final double TELEOP_MAX_ANGULAR_VELOCITY = SwerveDrive.maxAngularVelocity(TELEOP_MAX_VELOCITY);;

    // AUTONOMOUS
    public static final double AUTO_MAX_VELOCITY = TELEOP_MAX_VELOCITY / 4; // measured in meters/sec
    public static final double AUTO_MAX_ACCELERATION = 2.0; // measured in meters/sec^2
    public static final double AUTO_MAX_ANGULAR_VELOCITY = SwerveDrive.maxAngularVelocity(AUTO_MAX_VELOCITY); // measured in radians/sec
    public static final double AUTO_MAX_ANGULAR_ACCELERATION = 2.0; // measured in rad/sec^2
    public static final double[] AUTO_X_PID = { 1.0, 0.0, 0.0 };
    public static final double[] AUTO_Y_PID = { 1.0, 0.0, 0.0 };
    public static final double[] AUTO_ANGLE_PID = { 1.0, 0.0, 0.0 };
    public static final TrapezoidProfile.Constraints AUTO_ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(
        AUTO_MAX_ANGULAR_VELOCITY,
        AUTO_MAX_ANGULAR_ACCELERATION);

  }

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
