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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class Dashboard extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private ShuffleboardTab dashboardTab;
  private ShuffleboardLayout swerveData;
  private final Field2d field = new Field2d();

  private ComplexWidget gyroEntry;
  private GenericEntry totalVoltage;
  private GenericEntry totalCurrent;

  private ShuffleboardLayout[] moduleDataLists = new ShuffleboardLayout[4];
  private GenericEntry[] moduleAngles = new GenericEntry[4];
  private GenericEntry[] moduleSpeeds = new GenericEntry[4];
  private GenericEntry[] moduleCurrents = new GenericEntry[4];
  private GenericEntry[] moduleVoltages = new GenericEntry[4];

  private boolean initialized = false;

  private int w = 2;
  private int h = 10;

  /** Creates a new ExampleSubsystem. */
  public Dashboard(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  public void initialize() {
    if (initialized) {
      return;
    }

    initialized = true;

    int x = 0;

    dashboardTab = Shuffleboard.getTab(DashboardConstants.TAB_NAME);

    swerveData = dashboardTab.getLayout("Swerve", BuiltInLayouts.kList).withSize(2, 6).withPosition(x, 0);

    x += 2;

    gyroEntry = swerveData.add(swerveDrive.getGyro())
        .withProperties(Map.of("name", "Robot Heading"))
        .withPosition(x, 0);

    totalVoltage = swerveData.add("Voltage", 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0.0, "max", 24.0 * 8.0))
        .getEntry();

    totalCurrent = swerveData.add("Current", 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0.0, "max", SwerveDriveConstants.TOTAL_CURRENT_LIMIT))
        .getEntry();

    // dashboardTab.add("field", field).withPosition(x, 0).withSize(3, 2);

    // x += 3;

    SwerveModule[] modules = swerveDrive.getModules();

    for (int i = 0; i < 4; i++) {
      int _x = (i + 1) * 2 + x;

      moduleDataLists[i] = dashboardTab.getLayout(modules[i].getName() + " Module", BuiltInLayouts.kList)
          .withSize(2, 6)
          .withPosition(_x, 0);

      // moduleAngles[i] = moduleDataLists[i]
      //     .add("Angle", 0)
      //     .withWidget(BuiltInWidgets.kDial)
      //     .withProperties(Map.of("min", -180.0, "max", 180.0))
      //     .getEntry();
      moduleSpeeds[i] = moduleDataLists[i]
          .add("Drive Speed", 0)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0.0, "max", SwerveDriveConstants.MOTOR_POWER_HARD_CAP))
          .getEntry();
      moduleCurrents[i] = moduleDataLists[i]
          .add("Current", 0)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0, "max", SwerveDriveConstants.TOTAL_CURRENT_LIMIT / 4))
          .getEntry();
      moduleVoltages[i] = moduleDataLists[i]
          .add("Voltage", 0)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0, "max", 24 * 2))
          .getEntry();
    }
  }

  @Override
  public void periodic() {
    if (!initialized) {
      return;
    }

    Shuffleboard.update();

    // swerveDrive.setPID(PIDValues);
    totalVoltage.setDouble(swerveDrive.getVoltage());
    totalCurrent.setDouble(swerveDrive.getCurrent());

    // field.setRobotPose(swerveDrive.getPose());

    SwerveModule[] modules = swerveDrive.getModules();

    for (int i = 0; i < 4; i++) {
      SwerveModule module = modules[i];

      // moduleAngles[i].setDouble(module.getSteerDegrees());
      moduleSpeeds[i].setDouble(Math.abs(module.getVelocity()));
      moduleCurrents[i].setDouble(module.getCurrent());
      moduleVoltages[i].setDouble(module.getVoltage());
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
