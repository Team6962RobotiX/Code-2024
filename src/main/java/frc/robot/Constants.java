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

  // ENABLED SYSTEMS
  public static final class EnabledSystems {
    public static final boolean ENABLE_DRIVE = true;
    public static final boolean ENABLE_LIMELIGHT = false;
  }

  // DEVICES
  public static final class Devices {
    public static final int USB_XBOX_CONTROLLER = 0;
  }

  // DASHBOARD (ShuffleBoard)
  public static final class DashboardConfig {
    public static final String TAB_NAME = "Dashboard";
  }

  // LIMELIGHT
  public static final class LimelightConfig {
    public static final String NAME = "limelight";
  }

  // SWERVE DRIVE
  public static final class SwerveDriveConfig {

    /*
      -------------------------------------
      | SIMPLE CONFIG, FEEL FREE TO EDIT! |
      -------------------------------------
    */

    public static final double MOTOR_POWER_LIMIT = 0.05; // Absolute maximum percent motor power (0.5 = 50%)

    public static final double CONTROLLER_DEADZONE = 0.1; // If joystick values are less than this (0.1 = 10%) than we just read 0
    public static final double VELOCITY_DEADZONE = 0.05; // speed at which we stop moving all together

    public static final int TOTAL_CURRENT_LIMIT = 300; // [TODO] Default is around 640 Amps (also drive motors have double the current allocation than steer motors)
    public static final double MOTOR_POWER_RAMP_RATE = 0.1; // [TODO] Maximum change in motor power between ticks to reduce power spikes

    public static final Pose2d STARTING_POSE = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final double STARTING_ANGLE_OFFSET = 0.0;
    /*
      -------------------------------------------------------------------
      | ADVANCED CONFIG, DO NOT EDIT UNLESS YOU KNOW WHAT YOU'RE DOING! |
      -------------------------------------------------------------------
    */

    // PHYSICAL
    public static final double TRACKWIDTH_METERS = 0.6477; // left-to-right distance between the drivetrain wheels
    public static final double WHEELBASE_METERS = 0.62865; // front-to-back distance between the drivetrain wheels

    public static final double STEER_GEAR_REDUCTION = 7.0 / 150.0;
    public static final double DRIVE_GEAR_REDUCTION = 1.0 / 6.75;
    public static final double WHEEL_DIAMETER = 0.4572; // measured in meters
    public static final double MOTOR_RPM_VELOCITY_RATIO = DRIVE_GEAR_REDUCTION / 60.0 * WHEEL_DIAMETER * Math.PI;
    public static final double[] STEER_ENCODER_OFFSETS = { 0.0, 0.0, 0.0, 0.0 };

    public static final double FULL_POWER_VELOCITY = 21; // [TODO] VERY IMPORTANT DO NOT CHANGE
    public static final double FULL_POWER_ANGULAR_VELOCITY = SwerveDrive.maxAngularVelocity(FULL_POWER_VELOCITY);

    public static final double MAX_VELOCITY = MOTOR_POWER_LIMIT * FULL_POWER_VELOCITY;
    public static final double MAX_ANGULAR_VELOCITY = SwerveDrive.maxAngularVelocity(MAX_VELOCITY);

    // PID
    public static final double[] MODULE_STEER_PID = { 1.0 / 360.0, 0.0, 0.0 }; // [TODO]
    public static final double[] TELEOP_ROTATE_PID = { 1.0 / 360.0, 0.0, 0.0 }; // [TODO]
    public static final double[] AUTO_ROTATE_PID = { 1.0, 0.0, 0.0 };
    public static final double[] AUTO_X_PID = { 1.0, 0.0, 0.0 }; // [TODO]
    public static final double[] AUTO_Y_PID = { 1.0, 0.0, 0.0 }; // [TODO]

    // AUTONOMOUS
    public static final double AUTO_MAX_VELOCITY = MAX_VELOCITY / 4; // [TODO] measured in meters/sec
    public static final double AUTO_MAX_ACCELERATION = 9.80 / 4; // [TODO] measured in meters/sec^2
    public static final double AUTO_MAX_ANGULAR_VELOCITY = SwerveDrive.maxAngularVelocity(AUTO_MAX_VELOCITY); // measured in radians/sec
    public static final double AUTO_MAX_ANGULAR_ACCELERATION = SwerveDrive.maxAngularVelocity(AUTO_MAX_ACCELERATION); // measured in rad/sec^2
    public static final TrapezoidProfile.Constraints AUTO_ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(
        AUTO_MAX_ANGULAR_VELOCITY,
        AUTO_MAX_ANGULAR_ACCELERATION);

    // MODULES
    // In order of: front left, front right, back left, back right, where the battery is in the back
    public static final String[] MODULE_NAMES = { "FL", "FR", "BL", "BR" };
    public static final int[] CAN_DRIVE = { 10, 20, 30, 40 }; // TODO
    public static final int[] CAN_STEER = { 11, 21, 31, 41 }; // TODO
    public static final int[] CAN_STEER_ENCODER = { 12, 22, 32, 42 }; // TODO
  }
}
