// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveMath;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.controllers.*;
import com.pathplanner.lib.server.*;
import com.pathplanner.*;

public final class SwerveAutonomous {
  public Command fullAuto(String pathName, HashMap<String, Command> eventMap, SwerveDrive swerveDrive) {
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(SwerveDriveConstants.AUTO_MAX_VELOCITY, SwerveDriveConstants.AUTO_MAX_ACCELERATION));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerveDrive::getPose, // Pose2d supplier
        swerveDrive::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
        SwerveMath.getKinematics(), // SwerveDriveKinematics
        SwerveDriveConstants.AUTO_MOVE_PID, // PID constants to correct for translation error (used to create the X and Y PID controllers)
        SwerveDriveConstants.AUTO_ROTATE_PID, // PID constants to correct for rotation error (used to create the rotation controller)
        swerveDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap, true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        swerveDrive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);
  }
}