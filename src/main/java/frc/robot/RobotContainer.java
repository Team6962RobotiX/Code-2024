// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import java.util.Random;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DEVICES;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.commands.characterization.CharacterizeSwerve;
import frc.robot.commands.drive.FeedForwardCharacterization;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.Logging.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  private final XboxController XboxController = new XboxController(DEVICES.USB_XBOX_CONTROLLER);
  private final SwerveDrive swerveDrive = new SwerveDrive();
  // private final Limelight limelight = new Limelight(LimelightConfig.NAME);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    Logger.log("constants", this, Constants.class);
    Logger.autoLog("PDH", new PowerDistribution(CAN.PDH, ModuleType.kRev));
    Logger.startLog();
    swerveDrive.setDefaultCommand(new XBoxSwerve(swerveDrive, () -> XboxController));
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return new CharacterizeSwerve(swerveDrive);
    
    // return new FeedForwardCharacterization(swerveDrive, swerveDrive::runCharacterization, swerveDrive::getCharacterizationVelocity);
    
    // HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    // return swerveDrive.fullAuto("Test Path", eventMap);
    
    // int numPoints = 6;
    // List<Twist2d> randomPoints = new ArrayList<Twist2d>();
    // Random rand = new Random();
    // for (int i = 0; i < numPoints; i++) {
    //   randomPoints.add(new Twist2d(
    //     rand.nextDouble() * 16.0,
    //     rand.nextDouble() * 8.0,
    //     (rand.nextDouble() - 0.5) * Math.PI * 2.0
    //   ));
    // }
    // return swerveDrive.followTrajectoryCommand(swerveDrive.generateTrajectoryFieldRelativeBasic(randomPoints));
  }

  public void disabledPeriodic() {
  }
}
