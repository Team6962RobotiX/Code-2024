// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean IS_BLUE_TEAM = !(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

  // ENABLED SYSTEMS
  public static final class ENABLED_SYSTEMS {
    public static final boolean ENABLE_DRIVE     = true;
    public static final boolean ENABLE_LIMELIGHT = false;
    public static final boolean ENABLE_DASHBOARD = true;
    public static final boolean ENABLE_SHOOTER   = false;
    public static final boolean ENABLE_INTAKE   = true;
    public static final boolean ENABLE_TRANSFER = true;
    public static final boolean ENABLE_AMP = true;
  }

  public static final class LOGGING {
    public static final int LOGGING_PERIOD_MS = 20;
  }

  // DEVICES
  public static final class DEVICES {
    public static final int DRIVE_XBOX_CONTROLLER = 0;
    public static final int OPERATOR_XBOX_CONTROLLER = 1;
  }

  // DASHBOARD (ShuffleBoard)
  public static final class DASHBOARD {
    public static final String TAB_NAME = "SwerveDrive";
  }

  // LIMELIGHT
  public static final class LIMELIGHT {
    public static final String[] APRILTAG_CAMERA_NAMES = {"limelight"};
  }

  // SWERVE DRIVE
  public static final class SWERVE_DRIVE {

    ///////////////////////// CONFIG /////////////////////////
    public static final double   INSPECTION_WEIGHT                  = Units.lbsToKilograms(41);
    public static final double   BATTERY_WEIGHT                     = Units.lbsToKilograms(12.89);
    public static final double   ROBOT_MASS                         = INSPECTION_WEIGHT + BATTERY_WEIGHT; // kg
    public static final double   FRICTION_COEFFICIENT               = 0.5; // 1.0 when on carpet 0.5 on KLS flooring
    public static final int      MODULE_COUNT                       = 4;
    public static final double   CHASSIS_WIDTH                      = Units.inchesToMeters(28);
    public static final double   CHASSIS_LENGTH                     = Units.inchesToMeters(28);
    public static final double   BUMPER_THICKNESS                   = Units.inchesToMeters(3.25);
    public static final double   WHEEL_TO_EDGE_DISTANCE             = Units.inchesToMeters(2.625);
    public static final double   WHEEL_RADIUS                       = Units.inchesToMeters(2.0);
    public static final double   WHEEL_WIDTH                        = Units.inchesToMeters(2.0);
    public static final double   DRIVE_MOTOR_GEARING                = 6.75;
    public static final double   STEER_MOTOR_GEARING                = 150.0 / 7.0;
    public static final double   GEARBOX_EFFICIENCY                 = 0.8;
    public static final double   BATTERY_RESISTANCE                 = 0.02; // ohms
    public static final double   BATTERY_VOLTAGE                    = 12.6; // volts
    public static final double   BROWNOUT_VOLTAGE                   = 6.8; // volts

    // DRIVING OPTIONS
    public static final double   TELEOPERATED_FINE_TUNE_DRIVE_POWER = 0.1; // Percent driving power when using d-pad
    public static final double   TELEOPERATED_DRIVE_POWER           = 0.5; // Percent driving power
    public static final double   TELEOPERATED_BOOST_POWER           = 1.0; // Percent power when using the triggers
    public static final double   TELEOPERATED_ROTATE_POWER          = 0.5; // Percent rotating power
    public static final double   VELOCITY_DEADBAND                  = 0.05; // Velocity we stop moving at
    
    // ODOMETER
    public static final Pose2d   STARTING_POSE                      = Field.pose2d(0.0, 0.0, 0.0);

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
      public static final int    SLIPLESS_CURRENT_LIMIT             = (int) ((SLIPLESS_ACCELERATION * NEO.STATS.stallCurrentAmps * ROBOT_MASS * WHEEL_RADIUS) / (4.0 * DRIVE_MOTOR_GEARING * NEO.STATS.stallTorqueNewtonMeters));
      
      public static final double MAX_MOTOR_SPEED                    = NEO.RPM * GEARBOX_EFFICIENCY;
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

      public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
        new PathConstraints(
          SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
          SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION / 2.0,
          SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY,
          SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_ACCELERATION / 2.0
        );
    }

    public static final class DRIVE_MOTOR_PROFILE {
      // FROM WPILIB SYSTEM IDENTIFICATION, FREE SPINNING
      public static final double kP                 = 0.00000; // Proportion Gain
      public static final double kI                 = 0.00000; // Integral Gain
      public static final double kD                 = 0.00000; // Derivative Gain
      public static final double kS                 = 0.00000; // volts
      public static final double kV                 = 12.0 / PHYSICS.MAX_LINEAR_VELOCITY; // volts per m/s
      public static final double kA                 = 0.10000; // volts per m/s^2, free spinning
      
      // CALCULATED
      public static final int    CURRENT_LIMIT      = PHYSICS.SLIPLESS_CURRENT_LIMIT; // Amps
      public static final double RAMP_RATE          = 0.1; // Seconds it takes to reach full power
      
      // PREFERENCE
      public static final int[]  STATUS_FRAMES      = { 10, 10, 10, 500, 500, 500, 500 }; // ms
    }

    public static final class STEER_MOTOR_PROFILE {
      // FROM WPILIB SYSTEM IDENTIFICATION
      public static final double kP                 = 0.90000; // Proportion Gain
      public static final double kI                 = 0.00000; // Integral Gain
      public static final double kD                 = 0.10000; // Derivative Gain
      public static final double kS                 = 0.00000; // volts
      public static final double kV                 = 12.0 / (NEO.STATS.freeSpeedRadPerSec / STEER_MOTOR_GEARING); // volts per rad/s
      public static final double kA                 = 0.02000; // volts per rad/s^2
      
      // CALCULATED
      public static final int    CURRENT_LIMIT      = 20; // Amps
      public static final double RAMP_RATE          = 0.1; // Seconds it takes to reach full power
      
      // PREFERENCE
      public static final int[]  STATUS_FRAMES      = { 10, 10, 10, 500, 500, 500, 500 }; // ms
    }

    // TELEOPERATED
    public static final class ABSOLUTE_ROTATION_GAINS {
      public static final double kP = 3.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(0.5);
    }
    
    // MODULES
    // In order of: front left, front right, back left, back right, where the battery is in the back

    // public static final String[] MODULE_NAMES          = { "FL", "FR", "BL", "BR" };
    // public static final double[] STEER_ENCODER_OFFSETS_PROTO = { -213.047, 24.785, -34.805, -11.602 };
    // public static final double[] STEER_ENCODER_OFFSETS_COMP  = { 69.70908 + 180.0, 88.0214 + 180.0, 57.48648 + 180.0, 191.33784 + 180.0 };

    // public static final double[] STEER_ENCODER_OFFSETS_PROTO = { -213.047, 24.785, -34.805, -11.602 };
    // public static final double[] STEER_ENCODER_OFFSETS_COMP  = { 69.70908, 88.0214, 57.48648, 191.33784 };
    
    public record MODULE_CONFIG (int ID, int CAN_DRIVE, int CAN_STEER, int CAN_ENCODER, double ENCODER_OFFSET) {}

    public static final MODULE_CONFIG[] MODULES = new MODULE_CONFIG[] {
      new MODULE_CONFIG(0, 31, 32, 33, 0.6936363333),
      new MODULE_CONFIG(1, 34, 35, 36, -0.5054961111),
      new MODULE_CONFIG(2, 37, 38, 39, 0.9096846667),
      new MODULE_CONFIG(3, 40, 41, 42, 0.531494),
      new MODULE_CONFIG(4, 43, 44, 45, -0.5917972222),
      new MODULE_CONFIG(5, 46, 47, 48, -0.1811527778),
      new MODULE_CONFIG(6, 49, 50, 51, 0.1533194444),
      new MODULE_CONFIG(7, 52, 53, 54, -0.5322277778),
      new MODULE_CONFIG(8, 55, 56, 57, 0.0), // TODO
      new MODULE_CONFIG(9, 58, 59, 60, 0.0), // TODO
    };

    public static final String[] MODULE_NAMES = {
      "front-left",
      "front-right",
      "back-left",
      "back-right"
    };

    public static final MODULE_CONFIG[] EQUIPPED_MODULES_PROTOTYPE = {
      MODULES[4], // front-left
      MODULES[5], // front-right
      MODULES[6], // back-left
      MODULES[7]  // back-right
    };

    public static final MODULE_CONFIG[] EQUIPPED_MODULES_COMPETITION = {
      MODULES[0], // front-left
      MODULES[1], // front-right
      MODULES[2], // back-left
      MODULES[3]  // back-right
    };
  }

  public static final class CAN {
    // In order of: front left, front right, back left, back right, where the battery is in the back
    public static final int PDH = 1;
    public static final int SHOOTER_WHEELS_TOP = 19;
    public static final int SHOOTER_WHEELS_BOTTOM = 26;
    public static final int SHOOTER_PIVOT = 18;
    public static final int SHOOTER_FEED = 20;
    public static final int INTAKE = 23;
    public static final int CENTERING = 27;
    public static final int TRANSFER_IN = 22;
    public static final int TRANSFER_OUT = 25;
    public static final int AMP_PIVOT = 24;
    public static final int AMP_WHEELS = 17;
  }

  public static final class DIO {
    public static final int AMP_PIVOT = 1;
    public static final int SHOOTER_PIVOT = 2;
  }
  
  public static final class NEO {
    public static final double RPM = 5880;
    public static final DCMotor STATS = new DCMotor(12.0, 3.28, 181, 1.3, Units.rotationsPerMinuteToRadiansPerSecond(RPM), 1);
    public static final double SAFE_TEMPERATURE = 60;
    public static final int SAFE_STALL_CURRENT = 40;
    public static final double SAFE_RAMP_RATE = 0.1;

    public static double maxTorqueCurrentLimited(int currentLimit) {
      return STATS.stallTorqueNewtonMeters / STATS.stallCurrentAmps * currentLimit;
    }
  }

  public static final class PHOTON_LIB {
    public static final int SCALE = 1; //INCREASE THIS TO MAKE THE SIMULATION EASIER TO SEE ON A LAPTOP
    public static final int CAM_RESOLUTION_WIDTH = 320; //Pixels 
    public static final int CAM_RESOLUTION_HEIGHT = 240; //Pixels
    public static final double MIN_TARGET_AREA = 10; //Square pixels (CHANGE)

    
    public static final double CAM_PITCH = 0.0; //Degrees (CHANGE)
    public static final double CAM_HEIGHT_OFF_GROUND = 1.0; //Meters (CHANGE)

    public static final double FOV_HEIGHT = 59.6; //Degrees
    public static final double FOV_WIDTH = 49.7; //Degrees
    public static final double CAM_DIAG_FOV = Math.sqrt(Math.pow(FOV_HEIGHT, 2) + Math.pow(FOV_WIDTH, 2));

    public static final double MAX_LED_RANGE = 20; //Meters (CHANGE)
  }

  public static final class SHOOTER {
    public static final class FEED_WHEELS {
      public static final double NOTE_DETECTION_CURRENT = 10.0;
    }

    public static final class WHEELS {
      public static final double GEARBOX_STEP_UP = 2.0;
      public static final double ENCODER_CONVERSION_FACTOR = 2.0 * Math.PI * GEARBOX_STEP_UP;
      public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
      public static final double WHEEL_MOI = 0.00018540712;
      public static final double TOTAL_MOI = 0.00018540712 * 12.0;
      public static final double PROJECTILE_MASS = Units.lbsToKilograms(0.5);

      // x is front-to-back
      // y is left-to-right
      // z is top-to-bottom
      
      public static final class PROFILE {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0; // volts per rad/s
        public static final double kV = 12.0 / (NEO.STATS.freeSpeedRadPerSec * GEARBOX_STEP_UP); // volts per rad/s
        public static final double kA = 0.0; // volts per rad/s^2
        public static final double RAMP_RATE = 0.1;
        public static final int    CURRENT_LIMIT = 40;
      }
    }

    public static final class PIVOT {
      public static final double GEARBOX_REDUCTION = 400.0;
      public static final double ENCODER_CONVERSION_FACTOR = 2.0 * Math.PI / GEARBOX_REDUCTION;
      public static final double ROTATION_DELAY = 0.3; // seconds
      public static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.5);
      public static final Rotation2d ANGLE_PRECISION = Rotation2d.fromDegrees(0.5);
      public static final Rotation2d HEADING_PRECISION = Rotation2d.fromDegrees(0.5);
      public static final Translation3d POSITION = new Translation3d(0.0, 0.0, Units.inchesToMeters(0.0));
      public static final double CoM_DISTANCE = Units.inchesToMeters(15.0);
      public static final double MASS = Units.lbsToKilograms(14.3);
      public static final double MOI = (1.0 / 3.0) * MASS * Math.pow(CoM_DISTANCE, 2.0);
      public static final double ABSOLUTE_POSITION_OFFSET = 0.0;

      public static final class PROFILE {
        public static final int    CURRENT_LIMIT = 40;
        public static final double kP = 0.0;
        public static final double RAMP_RATE = 0.1;
        public static final double MAX_ACCELERATION = (NEO.maxTorqueCurrentLimited(CURRENT_LIMIT) * GEARBOX_REDUCTION) / MOI; // rad/s^2
      }
    }
  }

  public static final class AMP {
    public static final class WHEELS {
      public static final double NOTE_DETECTION_CURRENT = 10.0;
    }

    public static final class PIVOT {
      public static final double GEARBOX_REDUCTION = 60.6666;
      public static final double ENCODER_CONVERSION_FACTOR = 2.0 * Math.PI / GEARBOX_REDUCTION;
      public static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.5);
      public static final double CoM_DISTANCE = Units.inchesToMeters(9.5);
      public static final double MASS = Units.lbsToKilograms(4.0);
      public static final double MOI = (1.0 / 3.0) * MASS * Math.pow(CoM_DISTANCE, 2.0);
      public static final double ABSOLUTE_POSITION_OFFSET = -0.944;

      public static final class PROFILE {
        public static final int    CURRENT_LIMIT = 40;
        public static final double kP = 1.0;
        public static final double RAMP_RATE = 0.1;
        public static final double MAX_ACCELERATION = 40.0; // rad/s^2
      }
    }
  }

  public static final class TRANSFER {
    public static final double NOTE_DETECTION_CURRENT = 10.0;
  }

  public static final class INTAKE {
    public static final double NOTE_DETECTION_CURRENT = 10.0;
  }
}
