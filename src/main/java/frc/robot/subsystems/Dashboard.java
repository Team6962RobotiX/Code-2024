// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
import frc.robot.Constants.SwerveDriveConfig;

public class Dashboard extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private ShuffleboardTab dashboardTab = Shuffleboard.getTab(DashboardConfig.TAB_NAME);
  private ShuffleboardLayout swerveData = dashboardTab.getLayout("Swerve", BuiltInLayouts.kList).withSize(2, 5);
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final Field2d field = new Field2d();

  /** Creates a new ExampleSubsystem. */
  public Dashboard(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double[] PIDValues = {
        swerveData.add("SwervekP", SwerveDriveConfig.MODULE_STEER_PID[0])
            .getEntry()
            .getDouble(SwerveDriveConfig.MODULE_STEER_PID[0]),
        swerveData.add("SwervekI", SwerveDriveConfig.MODULE_STEER_PID[1])
            .getEntry()
            .getDouble(SwerveDriveConfig.MODULE_STEER_PID[1]),
        swerveData.add("SwervekD", SwerveDriveConfig.MODULE_STEER_PID[2])
            .getEntry()
            .getDouble(SwerveDriveConfig.MODULE_STEER_PID[2])
    };

    swerveDrive.setPID(PIDValues);

    swerveData.add(gyro).withProperties(Map.of("name", "Robot Heading"));

    swerveData.add("Voltage", 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0, "max", 24 * 8))
        .getEntry()
        .setDouble(swerveDrive.getVoltage());

    swerveData.add("Current", 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0, "max", SwerveDriveConfig.TOTAL_CURRENT_LIMIT))
        .getEntry()
        .setDouble(swerveDrive.getCurrent());

    field.setRobotPose(swerveDrive.getPose());
    swerveData.add("field", field);

    for (SwerveModule module : swerveDrive.getSwerveModules()) {
      ShuffleboardLayout moduleData = dashboardTab.getLayout(module.getName() + " Module", BuiltInLayouts.kList)
          .withSize(2, 5);

      moduleData.add("Angle", 0)
          .withWidget(BuiltInWidgets.kGyro)
          .getEntry()
          .setDouble(module.getAngle());

      moduleData.add("Drive Speed", 0)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0, "max", SwerveDriveConfig.MAX_VELOCITY))
          .getEntry()
          .setDouble(module.getDriveVelocity());

      moduleData.add("Current", 0)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0, "max", SwerveDriveConfig.TOTAL_CURRENT_LIMIT / 4))
          .getEntry()
          .setDouble(module.getCurrent());

      moduleData.add("Voltage", 0)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0, "max", 24 * 2))
          .getEntry()
          .setDouble(module.getVoltage());
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
