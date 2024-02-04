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
  public static final Map<String, Pose2d> AUTO_MOVE_POSITIONS_BLUE = Map.of(
    // "AMP", new Pose2d(1.85, 7.75, Rotation2d.fromDegrees(90.0)),
    // "SOURCE", new Pose2d(15.5, 1.25, Rotation2d.fromDegrees(45.0)),
    // "TRAP", new Pose2d()
    "TEST_POS", new Pose2d(3.5, 2.0, Rotation2d.fromDegrees(65.0))
  );

  public static final Translation3d SPEAKER_RED = new Translation3d(16.5, 5.5, 2.0);
  public static final double SPEAKER_WIDTH = 1.0;
  public static final double SPEAKER_HEIGHT = 0.5;
  public static final double NOTE_THICKNESS = Units.inchesToMeters(1.0);
  public static final double NOTE_LENGTH    = Units.inchesToMeters(14.0);

}
