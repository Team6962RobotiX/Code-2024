// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class MotionRecorder extends SubsystemBase {

  private SwerveDrive swerveDrive;
  private boolean recording = false;
  private List<String> positionData = new ArrayList<>();

  /** Creates a new ExampleSubsystem. */
  public MotionRecorder(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  public CommandBase startRecording() {
    return runOnce(() -> {
      recording = true;
      System.out.println("start");
    });
  }

  public CommandBase stopRecording() {
    return runOnce(() -> {
      recording = false;
      System.out.println("stop");
      writeData();
    });
  }

  @Override
  public void periodic() {
    if (!recording) {
      return;
    }

    Pose2d pose = swerveDrive.getPose();
    ChassisSpeeds velocity = swerveDrive.getTargetChassisSpeeds();

    positionData.add(pose.getTranslation().getX() + "," + velocity.vxMetersPerSecond + "," + pose.getTranslation().getY() + "," + velocity.vyMetersPerSecond + "," + pose.getRotation().getRadians() + "," + velocity.omegaRadiansPerSecond);

    // This method will be called once per scheduler run
  }

  public void writeData() {
    try {
      File csvOutputFile = new File(SwerveDriveConstants.MOTION_RECORDING_WRITE_FILE);
      try (PrintWriter pw = new PrintWriter(csvOutputFile)) {
        positionData.stream().forEach(pw::println);
      }
    } catch (Exception e) {
      System.out.println(e);
    }
  }

  public static ControlVectorList readData() {
    ControlVectorList newPositionData = new ControlVectorList();
    try {
      try (BufferedReader br = new BufferedReader(new FileReader(SwerveDriveConstants.MOTION_RECORDING_READ_FILE))) {
        String line;
        while ((line = br.readLine()) != null) {
          String[] values = line.split(",");
          newPositionData.add(
            new ControlVector(
              new double[] {
                Double.parseDouble(values[0]),
                Double.parseDouble(values[1])
              },
              new double[] {
                Double.parseDouble(values[2]),
                Double.parseDouble(values[3])
              }
            )
          );
        }
      }
    } catch (Exception e) {
      System.out.println(e);
    }
    return newPositionData;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
