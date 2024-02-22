// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.Robot;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Field;
import frc.robot.subsystems.drive.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Limelight extends SubsystemBase {
  /**
   * @return Whether the limelight has any valid targets.
   
   */

  public static boolean hasTarget(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("tv").getBoolean(false);
  }

  /**
   * @return Horizontal offset from Crosshair to Target (-29.8 to 29.8 degrees).
   */
  public static double targetHorizontal(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("tx").getDouble(0.0);
  }

  /**
   * @return Vertical offset from Crosshair to Target (-24.85 to 24.85 degrees).
   */
  public static double targetVertical(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("ty").getDouble(0.0);
  }

  /**
   * @return Target area (0% to 100% of the image).
   */
  public static double targetArea(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("ta").getDouble(0.0);
  }

  /**
   * @return The pipeline's latency contribution (ms).
   */
  public static long pipelineLatency(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("tl").getInteger(0);
  }

  /**
   * @return Capture pipeline latency (ms). 
   *         Time between the end of the exposure of the middle 
   *         row of the sensor to the beginning of the tracking pipeline.
   */
  public static long captureLatency(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("cl").getInteger(0);
  }

  /**
   * @return Total camera latency (ms).
   *         Equivalent to {@link Limelight#pipelineLatency()} + {@link Limelight#captureLatency()}
   */
  public static long totalLatency(String name) {
    return captureLatency(name) + pipelineLatency(name);
  }

  /**
   * @return Sidelength of the shortest side of the fitted bounding box (pixels).
   */
  public static long shortBounding(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("tshort").getInteger(0);
  }

  /**
   * @return Sidelength of the longest side of the fitted bounding box (pixels).
   */
  public static long longBounding(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("tlong").getInteger(0);
  }

  /**
   * @return Horizontal sidelength of the rough bounding box (0 - 320 pixels).
   */
  public static long horizontalBounding(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("thor").getInteger(0);
  }

  /**
   * @return Vertical sidelength of the rough bounding box (0 - 320 pixels).
   */
  public static long verticalBounding(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("tvert").getInteger(0);
  }

  /**
   * @return True active pipeline index of the camera (0..9).
   */
  public static long pipeline(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("getpipe").getInteger(0);
  }

  /**
   * @return Class ID of the primary neural detector result or the neural classifier result.
   */
  public static long classId(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("tclass").getInteger(0);
  }

  /**
   * @pre Must be set to an Apriltag pipeline.
   * @return The average HSV color underneath the crosshair region as a Number array.
   */
  public static Number[] crosshairHSV(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("tc").getNumberArray(new Number[] {});
  }

  /**
   * @pre Must be set to an Apriltag pipeline.
   * @return Robot transform in field-space.
   */
  public static Pose3d botpose(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return intoPose3d(table.getEntry("botpose").getDoubleArray(new double[6]));
  }

  /**
   * @pre Must be set to an Apriltag pipeline.
   * @return Robot transform in field-space (blue driverstation WPILib origin).
   */
  public static Pose3d botposeBlue(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return intoPose3d(table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]));
  }

  /**
   * @pre Must be set to an Apriltag pipeline.
   * @return Robot transform in field-space (red driverstation WPILib origin).
   */
  public static Pose3d botposeRed(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return intoPose3d(table.getEntry("botpose_wpired").getDoubleArray(new double[6]));
  }

  /**
   * @pre something
   * @return ID of the primary in-view Apriltag.
   */
  public static long targetId(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    return table.getEntry("tid").getInteger(0);
  }

  private static Pose3d intoPose3d(double[] values) {
    return new Pose3d(
      new Translation3d(values[0], values[1], values[2]),
      new Rotation3d(Units.degreesToRadians(values[3]), Units.degreesToRadians(values[4]), Units.degreesToRadians(values[5]))
    );
  }

  /**
   * 
   */
  //  public static double getNoteDist(String name, double height) {
  //   double y = targetVertical(name);
  //   double x = targetHorizontal(name);
  //   if (y != 0.0) {
  //     return ((height - Field.NOTE_THICKNESS / 2.0) / (Math.tan(Math.abs(Units.degreesToRadians(y))) * Math.abs(Math.cos(Units.degreesToRadians(x)))));
  //   } else {
  //     return 0;
  //   }
  // }

  public static Translation2d getNotePosition(String name, Rotation2d pitch, Pose2d robotPose, Translation2d fieldVelocity, Translation3d cameraToRobot) {
    double y = targetVertical(name);
    double x = targetHorizontal(name);
    double dist = ((cameraToRobot.getZ() - Field.NOTE_THICKNESS / 2.0) / (Math.tan(Math.abs(Units.degreesToRadians(y) + pitch.getRadians())) * Math.abs(Math.cos(Units.degreesToRadians(x)))));
    Translation2d robotPosition = robotPose.getTranslation().minus(fieldVelocity.times(totalLatency(name) / 1000.0));
    Translation2d notePosition = robotPosition.plus(cameraToRobot.toTranslation2d().plus(new Translation2d(dist, 0)).rotateBy(robotPose.getRotation()));
    return notePosition;
  }
}