// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveMath;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.PathPlannerTrajectory;

public final class SwerveAutonomous {

    // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    //   return new SequentialCommandGroup(
    //       new InstantCommand(() -> {
    //         // Reset odometry for the first path you run during auto
    //         if (isFirstPath) {
    //           this.resetOdometry(traj.getInitialHolonomicPose());
    //         }
    //       }),
    //       new PPSwerveControllerCommand(
    //           traj,
    //           this::getPose, // Pose supplier
    //           this.kinematics, // SwerveDriveKinematics
    //           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
    //           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //           this::setModuleStates, // Module states consumer
    //           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //           this // Requires this drive subsystem
    //       ));
    // }

    public static SequentialCommandGroup WpiFollowPoints(SwerveDrive swerveDrive, Pose2d startPose,
            List<Translation2d> points, Pose2d endPose) {
        TrajectoryConfig TrajectoryConfig = new TrajectoryConfig(SwerveDriveConstants.AUTO_MAX_VELOCITY,
                SwerveDriveConstants.AUTO_MAX_ACCELERATION)
                .setKinematics(SwerveMath.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                points,
                endPose,
                TrajectoryConfig);

        PIDController xPID = new PIDController(
                SwerveDriveConstants.AUTO_X_PID[0],
                SwerveDriveConstants.AUTO_X_PID[1],
                SwerveDriveConstants.AUTO_X_PID[2]);
        PIDController yPID = new PIDController(
                SwerveDriveConstants.AUTO_Y_PID[0],
                SwerveDriveConstants.AUTO_Y_PID[1],
                SwerveDriveConstants.AUTO_Y_PID[2]);
        ProfiledPIDController thetaPID = new ProfiledPIDController(
                SwerveDriveConstants.AUTO_THETA_PID[0],
                SwerveDriveConstants.AUTO_THETA_PID[1],
                SwerveDriveConstants.AUTO_THETA_PID[2],
                SwerveDriveConstants.AUTO_ANGLE_CONSTRAINTS);
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        HolonomicDriveController swervePID = new HolonomicDriveController(xPID, yPID, thetaPID);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveDrive::getPose,
                SwerveMath.getKinematics(),
                swervePID,
                swerveDrive::setModuleStates,
                swerveDrive);

        return new SequentialCommandGroup(
                swerveControllerCommand, new InstantCommand(() -> swerveDrive.groundModules()));
    }
}