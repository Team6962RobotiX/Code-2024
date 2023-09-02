// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.subsystems.Drivetrain.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConfig;

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

    positionData.add(pose.getTranslation().getX() + "," + pose.getTranslation().getY() + "," + pose.getRotation().getDegrees());

    // This method will be called once per scheduler run
  }

  public void writeData() {
    try {
      File csvOutputFile = new File(SwerveDriveConfig.MOTION_RECORDING_WRITE_FILE);
      try (PrintWriter pw = new PrintWriter(csvOutputFile)) {
        positionData.stream().forEach(pw::println);
      }
    } catch (Exception e) {
      System.out.println(e);
    }
  }

  public static List<Pose2d> readData() {
    List<Pose2d> newPositionData = new ArrayList<>();
    try {
      try (BufferedReader br = new BufferedReader(new FileReader(SwerveDriveConfig.MOTION_RECORDING_READ_FILE))) {
        String line;
        while ((line = br.readLine()) != null) {
          String[] values = line.split(",");
          newPositionData.add(new Pose2d(
              Double.parseDouble(values[0]),
              Double.parseDouble(values[1]),
              Rotation2d.fromDegrees(Double.parseDouble(values[2]))));
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
