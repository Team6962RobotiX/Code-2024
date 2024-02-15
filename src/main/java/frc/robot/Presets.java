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
public final class Presets {
  public static final double NOTE_DETECTION_IMPULSE = 7.0;

  public static final class SHOOTER {
    public static final class FEED {
      public static final double POWER = 0.15;
    }
    
    public static final class WHEELS {
      public static final double TARGET_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(5000);

      public static final double LOAD_TIME = 1.0;
    }

    public static final class PIVOT {
      public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(90.0);
      public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0.0);

      public static final Rotation2d INTAKE_ANGLE = Rotation2d.fromDegrees(0);
    }
  }

  public static final class AMP {
    public static final class WHEELS {
      public static final double POWER = 0.45;

      public static final double LOAD_TIME = 1.0;
    }

    public static final class PIVOT {
      public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(45.0);
      public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-67.0);
      
      public static final Rotation2d OUTPUT_ANGLE = Rotation2d.fromDegrees(45.0);
      public static final Rotation2d INTAKE_ANGLE = Rotation2d.fromDegrees(-67.0);
    }
  }

  public static final class INTAKE {
    public static final double CENTERING_WHEEL_POWER = 0.2;
    public static final double INTAKE_ROLLER_POWER = 0.3;
  }

  public static final class TRANSFER {
    public static final double IN_POWER = 0.25;
    public static final double OUT_POWER = 0.25;
  }
}
