// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.DEVICES;
import frc.robot.commands.XBoxSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.Logger;
import frc.robot.utils.SwerveAutonomous;

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
    
    Logger.logClassValues("Constants", this, Constants.class);
    if (RobotBase.isReal()) {
      DataLogManager.start();
      if (Constants.LOGGING.ENABLE_DRIVER_STATION) Logger.logDriverStation("/driverStation");
    }

    swerveDrive.setDefaultCommand(new XBoxSwerve(swerveDrive.getTeleopController(), () -> XboxController));
    
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    return SwerveAutonomous.fullAuto("Test Path", eventMap, swerveDrive);
  }

  public void disabledPeriodic() {
  }
}
