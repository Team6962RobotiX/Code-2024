// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.drive.SwerveDrive;

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
  public static final class ENABLED_SYSTEMS {
    public static final boolean ENABLE_DRIVE     = true;
    public static final boolean ENABLE_LIMELIGHT = false;
    public static final boolean ENABLE_DASHBOARD = true;
    public static final boolean ENABLE_SHOOTER   = true;
  }

  public static final class LOGGING {
    public static final int LOGGING_PERIOD_MS = 20;
  }

  // DEVICES
  public static final class DEVICES {
    public static final int USB_XBOX_CONTROLLER = 0;
  }

  // DASHBOARD (ShuffleBoard)
  public static final class DASHBOARD {
    public static final String TAB_NAME = "SwerveDrive";
  }

  // LIMELIGHT
  public static final class LIMELIGHT {
    public static final String NAME = "limelight";
  }

  // SWERVE DRIVE
  public static final class SWERVE_DRIVE {

    ///////////////////////// CONFIG /////////////////////////
  
    public static final double   ROBOT_MASS                         = 25; // kg
    public static final double   FRICTION_COEFFICIENT               = 1.0; // 1.0 when on carpet 0.5 on KLS flooring
    public static final int      MODULE_COUNT                       = 4;
    public static final double   CHASSIS_WIDTH                      = Units.inchesToMeters(28);
    public static final double   CHASSIS_LENGTH                     = Units.inchesToMeters(28);
    public static final double   BUMPER_THICKNESS                   = Units.inchesToMeters(3.25);
    public static final double   WHEEL_TO_EDGE_DISTANCE             = Units.inchesToMeters(2.625);
    public static final double   WHEEL_RADIUS                       = Units.inchesToMeters(2.0); // measured in meters
    public static final double   WHEEL_WIDTH                        = Units.inchesToMeters(2.0); // measured in meters
    public static final double   DRIVE_MOTOR_GEARING                = 6.75;
    public static final double   STEER_MOTOR_GEARING                = 150.0 / 7.0;
    public static final double   GEARBOX_EFFICIENCY                 = 0.8;
    public static final double   BATTERY_RESISTANCE                 = 0.015; // ohms
    public static final double   BATTERY_VOLTAGE                    = 12.6; // volts
    public static final double   BROWNOUT_VOLTAGE                   = 6.8; // volts

    // DRIVING OPTIONS
    public static final double   TELEOPERATED_FINE_TUNE_DRIVE_POWER = 0.1; // Percent driving power when using d-pad
    public static final double   TELEOPERATED_DRIVE_POWER           = 0.5; // Percent driving power
    public static final double   TELEOPERATED_BOOST_POWER           = 1.0; // Percent power when using the triggers
    public static final double   TELEOPERATED_ROTATE_POWER          = 0.5; // Percent rotating power
    public static final double   VELOCITY_DEADBAND                  = 0.01; // Velocity we stop moving at
    
    // ODOMETER
    public static final Pose2d   STARTING_POSE                      = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    // TESTING
    public static final double   MOTOR_POWER_HARD_CAP               = 1.0; // Only use for testing, otherwise set to 1.0
    
    // REDUCE DRIVE VELOCITY WHEN FAR FROM ANGLE
    public static final boolean  DO_ANGLE_ERROR_SPEED_REDUCTION     = true;
    public static final double   DISCRETIZED_TIME_STEP              = 0.1; // Keeps movement in straight lines when rotating
    

    ///////////////////////// CALCUALTED /////////////////////////

    // PHYSICAL
    public static final double   TRACKWIDTH                         = CHASSIS_WIDTH - WHEEL_TO_EDGE_DISTANCE * 2.0; // left-to-right distance between the drivetrain wheels
    public static final double   WHEELBASE                          = CHASSIS_LENGTH - WHEEL_TO_EDGE_DISTANCE * 2.0; // front-to-back distance between the drivetrain wheels
    public static final double   BUMPER_WIDTH                       = SWERVE_DRIVE.CHASSIS_WIDTH + SWERVE_DRIVE.BUMPER_THICKNESS * 2.0;
    public static final double   BUMPER_LENGTH                      = SWERVE_DRIVE.CHASSIS_LENGTH + SWERVE_DRIVE.BUMPER_THICKNESS * 2.0;
    public static final double   MAX_CURRENT_DRAW                   = (BATTERY_VOLTAGE - BROWNOUT_VOLTAGE) / BATTERY_RESISTANCE;
    public static final boolean  IS_PROTOTYPE_CHASSIS               = !new DigitalInput(0).get();

    
    // GEAR AND WHEEL RATIOS
    public static final double   DRIVE_ENCODER_CONVERSION_FACTOR    = (WHEEL_RADIUS * 2.0 * Math.PI) / DRIVE_MOTOR_GEARING;
    public static final double   STEER_ENCODER_CONVERSION_FACTOR    = (Math.PI * 2.0) / STEER_MOTOR_GEARING;
    
    public static class PHYSICS {
      public static final double ROTATIONAL_INERTIA                 = (1.0 / 12.0) * ROBOT_MASS * (Math.pow(BUMPER_WIDTH, 2.0) + Math.pow(BUMPER_LENGTH, 2.0));
      public static final double SLIPLESS_ACCELERATION              = 9.80 * FRICTION_COEFFICIENT;
      public static final int    SLIPLESS_CURRENT_LIMIT             = (int) ((SLIPLESS_ACCELERATION * NEO.STALL_CURRENT * ROBOT_MASS * WHEEL_RADIUS) / (4.0 * DRIVE_MOTOR_GEARING * NEO.STALL_TORQUE));
      
      public static final double MAX_MOTOR_SPEED                    = NEO.FREE_SPEED * GEARBOX_EFFICIENCY;
      public static final double MAX_MOTOR_TORQUE                   = NEO.maxTorqueCurrentLimited(SLIPLESS_CURRENT_LIMIT);
      
      public static final double MAX_WHEEL_VELOCITY                 = (MAX_MOTOR_SPEED * (Math.PI * 2.0)) / 60.0 / DRIVE_MOTOR_GEARING;
      
      public static final double MAX_LINEAR_VELOCITY                = MAX_WHEEL_VELOCITY * WHEEL_RADIUS;
      public static final double MAX_LINEAR_FORCE                   = (4.0 * MAX_MOTOR_TORQUE * DRIVE_MOTOR_GEARING) / WHEEL_RADIUS;
      public static final double MAX_LINEAR_ACCELERATION            = MAX_LINEAR_FORCE / ROBOT_MASS;
      
      public static final double DRIVE_RADIUS                       = Math.hypot(WHEELBASE / 2.0, TRACKWIDTH / 2.0);
      public static final double MAX_ANGULAR_TORQUE                 = MAX_LINEAR_FORCE * DRIVE_RADIUS;
      public static final double MAX_ANGULAR_ACCELERATION           = MAX_ANGULAR_TORQUE / ROTATIONAL_INERTIA;
      public static final double MAX_ANGULAR_VELOCITY               = (MAX_WHEEL_VELOCITY * WHEEL_RADIUS) / DRIVE_RADIUS;
    }

    // Used only for when we have errors in the path (aka only when wheels slip or we're bumped off course)
    public static final class AUTONOMOUS {
      public static final class TRANSLATION_GAINS {
        public static final double kP = 0.75;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
      public static final class ROTATION_GAINS {
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
      public static final class DECCEL_CURVE {
        public static final double INITIAL_DIST = 4.0;
        public static final double INITIAL_SPEED = 1.0;
        public static final double FINAL_DIST = 1.0;
        public static final double FINAL_SPEED = 0.0;
        public static final double DEFAULT_DEGREE = 1.0/3.0;

        public static double getScale(double dist, double degree) {
          if (dist < 0) return 1.0;
          if (dist <= FINAL_DIST) return 0.0;
          double a = (INITIAL_SPEED-FINAL_SPEED)/Math.pow(INITIAL_DIST-FINAL_DIST, degree);
          double ret =  a * Math.pow(dist - FINAL_DIST, degree) + FINAL_SPEED;
          return Math.max(0.0, ret);
        }

        public static double getScale(double dist) { return getScale(dist, DEFAULT_DEGREE); }
      }
    }

    public static final class DRIVE_MOTOR_PROFILE {
      // FROM WPILIB SYSTEM IDENTIFICATION, FREE SPINNING
      public static final double kP                 = 0.00010; // Proportion Gain
      public static final double kI                 = 0.00000; // Integral Gain
      public static final double kD                 = 0.00000; // Derivative Gain
      public static final double kS                 = 0.00000; // volts
      public static final double kA                 = 0.27734; // volts per m/s^2, free spinning
      
      // CALCULATED
      public static final double kV                 = 12.0 / PHYSICS.MAX_LINEAR_VELOCITY; // volts per m/s
      public static final int    CURRENT_LIMIT      = PHYSICS.SLIPLESS_CURRENT_LIMIT; // Amps
      public static final double RAMP_RATE          = 0.1; // Seconds it takes to reach full power
      
      // PREFERENCE
      public static final int[]  STATUS_FRAMES      = { 10, 10, 10, 500, 500, 500, 500 }; // ms
    }

    public static final class STEER_MOTOR_PROFILE {
      // FROM WPILIB SYSTEM IDENTIFICATION
      public static final double kP                 = 0.72776; // Proportion Gain
      public static final double kI                 = 0.00000; // Integral Gain
      public static final double kD                 = 0.06514; // Derivative Gain
      public static final double kS                 = 0.06684; // volts
      public static final double kA                 = 0.01968; // volts per rad/s^2
      
      // CALCULATED
      public static final double kV                 = 12.0 / (NEO.FREE_SPEED / 60.0 * (1.0 / STEER_MOTOR_GEARING) * Math.PI * 2.0);
      public static final int    CURRENT_LIMIT      = 30; // Amps
      public static final double RAMP_RATE          = 0.1; // Seconds it takes to reach full power
      
      // PREFERENCE
      public static final int[]  STATUS_FRAMES      = { 10, 10, 10, 500, 500, 500, 500 }; // ms
    }

    // TELEOPERATED
    public static final class ABSOLUTE_ROTATION_GAINS {
      public static final double kP = 4.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }
    
    // MODULES
    // In order of: front left, front right, back left, back right, where the battery is in the back

    public static final String[] MODULE_NAMES                = { "FL", "FR", "BL", "BR" };
    public static final double[] STEER_ENCODER_OFFSETS_PROTO = { -213.047, 24.785, -34.805, -11.602 };
    public static final double[] STEER_ENCODER_OFFSETS_COMP  = { 69.70908 + 180.0, 88.0214 + 180.0, 57.48648 + 180.0, 191.33784 + 180.0 };
  }

  public static final class CAN {
    // In order of: front left, front right, back left, back right, where the battery is in the back
    public static final int[] SWERVE_DRIVE_SPARK_MAX = { 20, 40, 10, 30 };
    public static final int[] SWERVE_STEER_SPARK_MAX = { 21, 41, 11, 31 };
    public static final int[] SWERVE_STEER_CANCODERS = { 22, 42, 12, 32 };
    public static final int PDH = 5;
    public static final int SHOOTER_WHEELS = 6;
    public static final int SHOOTER_PIVOT = 7;
  }
  
  public static final class NEO {
    public static final double FREE_SPEED = 5880;
    public static final double STALL_TORQUE = 3.28;
    public static final double STALL_CURRENT = 181;
    public static final double SAFE_TEMPERATURE = 60;
    public static final int SAFE_STALL_CURRENT = 40;

    public static double maxTorqueCurrentLimited(int currentLimit) {
      return STALL_TORQUE / STALL_CURRENT * currentLimit;
    }
  }

  public static final class SWERVE_MATH {
    public static double angleDistance(double alpha, double beta) {
      double phi = Math.abs(beta - alpha) % (2.0 * Math.PI);
      return phi > Math.PI ? (2.0 * Math.PI) - phi : phi;
    }

    /**
     * Logical inverse of the Pose exponential from 254. Taken from team 3181.
     *
     * @param transform Pose to perform the log on.
     * @return {@link Twist2d} of the transformed pose.
     */
    public static Twist2d PoseLog(final Pose2d transform) {
      final double kEps          = 1E-9;
      final double dtheta        = transform.getRotation().getRadians();
      final double half_dtheta   = 0.5 * dtheta;
      final double cos_minus_one = transform.getRotation().getCos() - 1.0;
      double       halftheta_by_tan_of_halfdtheta;
      if (Math.abs(cos_minus_one) < kEps) {
        halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
      } else {
        halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
      }
      final Translation2d translation_part = transform.getTranslation().rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
      return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }
  }

  public static final class INPUT_MATH {
    public static double addLinearDeadband(double input, double deadband) { // input ranges from -1 to 1
      if (Math.abs(input) <= deadband) return 0.0;
      if (input > 0) return map(input, deadband, 1.0, 0.0, 1.0);
      return map(input, -deadband, -1.0, 0.0, -1.0);
    }

    public static double mapBothSides(double X, double A, double B, double C, double D) {
      if (X > 0.0) return map(X, A, B, C, D);
      if (X < 0.0) return map(X, -A, -B, -C, -D);
      return 0.0;
    }

    public static Translation2d circular(Translation2d input, double deadband, double snapRadians) {
      double magnitude = input.getNorm();
      double direction = input.getAngle().getRadians();
      if (mod(direction, Math.PI / 2.0) <= snapRadians / 2.0 || mod(direction, Math.PI / 2.0) >= (Math.PI / 2.0) - (snapRadians / 2.0)) {
        direction = Math.round(direction / (Math.PI / 2.0)) * (Math.PI / 2.0);
      }
      if (Math.abs(magnitude) <= deadband) return new Translation2d();
      magnitude = nonLinear(map(magnitude, deadband, 1.0, 0.0, 1.0));
      return new Translation2d(magnitude * Math.cos(direction), magnitude * Math.sin(direction));
    }

    public static double nonLinear(double x) {
      return (1 - Math.cos(Math.abs(x) * Math.PI / 2.0)) * Math.signum(x);
    }
  }

  public static boolean isIdle(XboxController xboxController) {
    if (xboxController.getRawAxis(0) != 0) {
      return false;
    }
    if (xboxController.getRawAxis(1) != 0) {
      return false;
    }
    return true;
  }

  public static double map(double X, double A, double B, double C, double D) {
    return (X - A) / (B - A) * (D - C) + C;
  }

  public static double mod(double x, double r) {
    return ((x % r) + r) % r;
  }

  public static final class PHOTON_LIB {
    public static final int SCALE = 1; //INCREASE THIS TO MAKE THE SIMULATION EASIER TO SEE ON A LAPTOP
    public static final int CAM_RESOLUTION_WIDTH = 320; //Pixels 
    public static final int CAM_RESOLUTION_HEIGHT = 240; //Pixels
    public static final double MIN_TARGET_AREA = 10; //Square pixels (CHANGE)

    
    public static final double CAM_PITCH = 0.0; //Degrees (CHANGE)
    public static final double CAM_HEIGHT_OFF_GROUND = 1.0; //Meters (CHANGE)
    public static final double CHAIR_HEIGHT_OFF_GROUND = 0.8; //Meters


    public static final double FOV_HEIGHT = 59.6; //Degrees
    public static final double FOV_WIDTH = 49.7; //Degrees
    public static final double CAM_DIAG_FOV = Math.sqrt(Math.pow(FOV_HEIGHT, 2) + Math.pow(FOV_WIDTH, 2));

    public static final double MAX_LED_RANGE = 20; //Meters (CHANGE)
  }

  public static final class SHOOTER {
    public static final class SHOOTER_WHEELS {
      public static final double GEARBOX_STEP_UP = 2.4;
      public static final double WHEEL_MOI = 0.001;
      public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
      public static final double PROJECTILE_MASS = Units.lbsToKilograms(0.5);
      public static final double TARGET_SPEED    = Units.rotationsPerMinuteToRadiansPerSecond(10000);

      // x is front-to-back
      // y is left-to-right
      // z is top-to-bottom
      
      public static final class PROFILE {
        public static final double kV = 0.1; // volts per rad/s
        public static final double kA = 0.1; // volts per rad/s^2
      }
    }

    public static final class SHOOTER_PIVOT {
      public static final double GEARBOX_REDUCTION = 1.0;
      public static final double ROTATION_DELAY    = 0.3; // seconds
      public static final double ANGLE_TOLERANCE   = Units.degreesToRadians(0.5);
      public static final double ANGLE_PRECISION   = Units.degreesToRadians(3);
      public static final double HEADING_PRECISION = Units.degreesToRadians(1);
      public static final Translation3d POSITION = new Translation3d(0.0, 0.0, 0.3);
      public static final double LENGTH = 1.0;
    }
  }
}
