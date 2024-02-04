// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Field {
  public static final double WIDTH = 8.21;
  public static final double LENGTH = 16.54;

  public static final Map<String, Pose2d> AUTO_MOVE_POSITIONS = Map.of(
    "AMP", pose2D(1.85, 7.75, -90.0),
    "SOURCE", pose2D(15.4, 1.0, 120.0),
    "SPEAKER", pose2D(1.5, 5.5, 0.0),
    "TRAP", pose2D(6, WIDTH / 2, 0.0)
  );

  public static final Translation3d SPEAKER = point3D(0.0, 5.5, 2.0);
  public static final double SPEAKER_WIDTH = 1.0;
  public static final double SPEAKER_HEIGHT = 0.5;
  public static final double NOTE_THICKNESS = Units.inchesToMeters(1.0);
  public static final double NOTE_LENGTH    = Units.inchesToMeters(14.0);

  public static Pose2d pose2D(double x, double y, double degrees) {
    return flipIfRed(new Pose2d(x, y, Rotation2d.fromDegrees(degrees)), Constants.IS_BLUE_TEAM);
  }

  public static Translation2d point2D(double x, double y) {
    return flipIfRed(new Translation2d(x, y), Constants.IS_BLUE_TEAM);
  }
  
  public static Translation3d point3D(double x, double y, double z) {
    return flipIfRed(new Translation3d(x, y, z), Constants.IS_BLUE_TEAM);
  }

  public static Translation2d flipIfRed(Translation2d position, boolean isBlueTeam) {
    return new Translation2d(isBlueTeam ? position.getX() : LENGTH - position.getX(), position.getY());
  }

  public static Translation3d flipIfRed(Translation3d position, boolean isBlueTeam) {
    return new Translation3d(isBlueTeam ? position.getX() : LENGTH - position.getX(), position.getY(), position.getZ());
  }

  public static Pose2d flipIfRed(Pose2d pose, boolean isBlueTeam) {
    return new Pose2d(flipIfRed(pose.getTranslation(), isBlueTeam), isBlueTeam ? pose.getRotation() : pose.getRotation().unaryMinus());
  }
}
