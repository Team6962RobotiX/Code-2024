// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Preferences {
  public static final class SWERVE_DRIVE {
    public static final double   TELEOPERATED_FINE_TUNE_DRIVE_POWER = 0.1; // Percent driving power when using d-pad
    public static final double   TELEOPERATED_DRIVE_POWER           = 0.5; // Percent driving power
    public static final double   TELEOPERATED_BOOST_POWER           = 1.0; // Percent power when using the triggers
    public static final double   TELEOPERATED_ROTATE_POWER          = 0.5; // Percent rotating power
  }

  public final class NOTE_DETECTION {
    public static final double THRESHOLD = 1.0;
  }

  public static final class SHOOTER_FEED {
    public static final double POWER = 1.0;
  }
  
  public static final class SHOOTER_WHEELS {
    public static final double TARGET_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(8000);
  }

  public static final class SHOOTER_PIVOT {
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(63.0);
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(22.0); // 21.148 (really 21.6626)
  }

  public static final class AMP_WHEELS {
    public static final double POWER = Math.min(1.0, ((TRANSFER.OUT_POWER * Constants.AMP_WHEELS.GEARING / Constants.TRANSFER.GEARING_OUT) / (1.625 / 2.0)) * (Constants.NEO.STATS.freeSpeedRadPerSec / Constants.NEO550.STATS.freeSpeedRadPerSec));
  }

  public static final class AMP_PIVOT {
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(50.0);
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-60.0);
    public static final Rotation2d OUTPUT_ANGLE = Rotation2d.fromDegrees(50.0);
    public static final Rotation2d INTAKE_ANGLE = Rotation2d.fromDegrees(-50.0);
  }

  public static final class TRANSFER {
    public static final double IN_POWER = 0.5 * ((Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY / Constants.TRANSFER.INTAKE_RADIUS) / (Constants.NEO.STATS.freeSpeedRadPerSec / Constants.TRANSFER.INTAKE_GEARING));
    public static final double OUT_POWER = IN_POWER * Constants.TRANSFER.GEARING_OUT / Constants.TRANSFER.GEARING_IN;
  }
}