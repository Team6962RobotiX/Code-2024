// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivetrain.*;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class Dashboard extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private ShuffleboardTab dashboardTab = Shuffleboard.getTab(DashboardConfig.TAB_NAME);
  private ShuffleboardLayout swerveData = dashboardTab.getLayout("Swerve", BuiltInLayouts.kList).withSize(2, 5);
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final Field2d field = new Field2d();

  private GenericEntry kP = swerveData.add("SwervekP", SwerveDriveConfig.MODULE_STEER_PID[0]).getEntry();
  private GenericEntry kI = swerveData.add("SwervekI", SwerveDriveConfig.MODULE_STEER_PID[1]).getEntry();
  private GenericEntry kD = swerveData.add("SwervekD", SwerveDriveConfig.MODULE_STEER_PID[2]).getEntry();

  private ComplexWidget gyroEntry = swerveData.add(gyro).withProperties(Map.of("name", "Robot Heading"));

  private GenericEntry totalVoltage = swerveData.add("Voltage", 0)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", 0, "max", 24 * 8))
      .getEntry();

  private GenericEntry totalCurrent = swerveData.add("Current", 0)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", 0, "max", SwerveDriveConfig.TOTAL_CURRENT_LIMIT))
      .getEntry();

  private ShuffleboardLayout[] moduleDataLists = new ShuffleboardLayout[4];
  private GenericEntry[] moduleAngles = new GenericEntry[4];
  private GenericEntry[] moduleSpeeds = new GenericEntry[4];
  private GenericEntry[] moduleCurrents = new GenericEntry[4];
  private GenericEntry[] moduleVoltages = new GenericEntry[4];

  /** Creates a new ExampleSubsystem. */
  public Dashboard(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    swerveData.add("field", field);

    SwerveModule[] modules = swerveDrive.getSwerveModules();

    for (int i = 0; i < 4; i++) {
      moduleDataLists[i] = dashboardTab.getLayout(modules[i].getName() + " Module", BuiltInLayouts.kList).withSize(2, 5);
      moduleAngles[i] = moduleDataLists[i].add("Angle", 0)
          .withWidget(BuiltInWidgets.kGyro)
          .getEntry();
      moduleSpeeds[i] = moduleDataLists[i].add("Drive Speed", 0)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0, "max", SwerveDriveConfig.MAX_VELOCITY))
          .getEntry();
      moduleCurrents[i] = moduleDataLists[i].add("Current", 0)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0, "max", SwerveDriveConfig.TOTAL_CURRENT_LIMIT / 4))
          .getEntry();
      moduleVoltages[i] = moduleDataLists[i].add("Voltage", 0)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0, "max", 24 * 2))
          .getEntry();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double[] PIDValues = {
        kP.getDouble(SwerveDriveConfig.MODULE_STEER_PID[0]),
        kI.getDouble(SwerveDriveConfig.MODULE_STEER_PID[1]),
        kD.getDouble(SwerveDriveConfig.MODULE_STEER_PID[2])
    };

    swerveDrive.setPID(PIDValues);
    totalVoltage.setDouble(swerveDrive.getVoltage());
    totalCurrent.setDouble(swerveDrive.getCurrent());

    field.setRobotPose(swerveDrive.getPose());

    SwerveModule[] modules = swerveDrive.getSwerveModules();

    for (int i = 0; i < 4; i++) {
      SwerveModule module = modules[i];

      moduleAngles[i].setDouble(module.getAngle());
      moduleSpeeds[i].setDouble(Math.abs(module.getDriveVelocity()));
      moduleCurrents[i].setDouble(module.getCurrent());
      moduleVoltages[i].setDouble(module.getVoltage());
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
