// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_MATH;
import frc.robot.subsystems.SwerveDrive;

public final class SwerveAutonomous {
  public static Command fullAuto(String pathName, HashMap<String, Command> eventMap, SwerveDrive swerveDrive) {
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(SWERVE_DRIVE.AUTONOMOUS_VELOCITY, SWERVE_DRIVE.AUTONOMOUS_ACCELERATION));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        swerveDrive::getPose, // Pose2d supplier
        swerveDrive::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
        SWERVE_MATH.getKinematics(), // SwerveDriveKinematics
        new PIDConstants(
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kI,
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kD
        ), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(
          SWERVE_DRIVE.AUTONOMOUS_ROTATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS_ROTATION_GAINS.kI,
          SWERVE_DRIVE.AUTONOMOUS_ROTATION_GAINS.kD
        ), // PID constants to correct for rotation error (used to create the rotation controller)
        swerveDrive::driveModules, // Module states consumer used to output to the drive subsystem
        eventMap, true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        swerveDrive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);
  }
}