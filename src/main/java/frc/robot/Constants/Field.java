// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Constants;

import java.util.Map;

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
    "AMP", pose2d(1.9, 7.5, 90.0),
    "SOURCE", pose2d(15.4, 1.0, -60.0),
    "SPEAKER", pose2d(1.5, 5.5, 180.0),
    "TRAP", pose2d(6, WIDTH / 2, 180.0),
    "TEST_POSE", new Pose2d(4.0, 3.0, Rotation2d.fromDegrees(63.0))
  );

  public static final Translation2d[] NOTE_POSITIONS = {
    point2d(Units.inchesToMeters(114), WIDTH / 2.0 + Units.inchesToMeters(57) * 2.0),
    point2d(Units.inchesToMeters(114), WIDTH / 2.0 + Units.inchesToMeters(57) * 1.0),
    point2d(Units.inchesToMeters(114), WIDTH / 2.0 + Units.inchesToMeters(57) * 0.0),
    point2d(LENGTH / 2.0, WIDTH / 2.0 + Units.inchesToMeters(66) * 2.0),
    point2d(LENGTH / 2.0, WIDTH / 2.0 + Units.inchesToMeters(66) * 1.0),
    point2d(LENGTH / 2.0, WIDTH / 2.0 + Units.inchesToMeters(66) * 0.0),
    point2d(LENGTH / 2.0, WIDTH / 2.0 + Units.inchesToMeters(66) * -1.0),
    point2d(LENGTH / 2.0, WIDTH / 2.0 + Units.inchesToMeters(66) * -2.0)
  };
  
  public static final double BLUE_WING_X = Units.inchesToMeters(231.2);
  public static final double WING_X = flipIfRed(BLUE_WING_X, Constants.IS_BLUE_TEAM);

  public static final Translation2d[] SHOT_POSITIONS = {
    point2d(BLUE_WING_X, 6.50),
    // point2d(BLUE_WING_X, 1.75),
    point2d(4.5, 5.0)
  };

  public static final Translation2d[] BLUE_STAGE_CORNERS = {
    new Translation2d(3.0, 4.0),
    new Translation2d(BLUE_WING_X, 5.8),
    new Translation2d(BLUE_WING_X, 2.4)
  };

  public static final Translation2d[] RED_STAGE_CORNERS = {
    new Translation2d(LENGTH - 3.0, 4.0),
    new Translation2d(LENGTH - BLUE_WING_X, 5.8),
    new Translation2d(LENGTH - BLUE_WING_X, 2.4)
  };


  public static final Translation3d SPEAKER = point3d(0.23, WIDTH / 2.0 + Units.inchesToMeters(57) * 1.0, 2.055);

  public static final double SPEAKER_WIDTH = 1.0;
  public static final double SPEAKER_HEIGHT = 0.45;
  public static final double SPEAKER_ANGLE = Units.degreesToRadians(14.0);
  public static final double NOTE_THICKNESS = Units.inchesToMeters(1.0);
  public static final double NOTE_LENGTH    = Units.inchesToMeters(14.0);

  public static Pose2d pose2d(double x, double y, double degrees) {
    return flipIfRed(new Pose2d(x, y, Rotation2d.fromDegrees(degrees)), Constants.IS_BLUE_TEAM);
  }

  public static Translation2d point2d(double x, double y) {
    return flipIfRed(new Translation2d(x, y), Constants.IS_BLUE_TEAM);
  }
  
  public static Translation3d point3d(double x, double y, double z) {
    return flipIfRed(new Translation3d(x, y, z), Constants.IS_BLUE_TEAM);
  }

  public static Translation2d flipIfRed(Translation2d position, boolean isBlueTeam) {
    return new Translation2d(isBlueTeam ? position.getX() : LENGTH - position.getX(), position.getY());
  }

  public static Translation3d flipIfRed(Translation3d position, boolean isBlueTeam) {
    return new Translation3d(isBlueTeam ? position.getX() : LENGTH - position.getX(), position.getY(), position.getZ());
  }

  public static Pose2d flipIfRed(Pose2d pose, boolean isBlueTeam) {
    return new Pose2d(flipIfRed(pose.getTranslation(), isBlueTeam), isBlueTeam ? pose.getRotation() : Rotation2d.fromDegrees(-(pose.getRotation().getDegrees() + 90) - 90));
  }

  public static double flipIfRed(double x, boolean isBlueTeam) {
    return isBlueTeam ? x : LENGTH - x;
  }
}
