package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Constants.DEVICES;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpPivot;
import frc.robot.subsystems.amp.AmpWheels;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.FeedWheels;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPivot;
import frc.robot.subsystems.shooter.ShooterWheels;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferInWheels;
import frc.robot.subsystems.transfer.TransferOutWheels;

public class Controls {
  public static final CommandXboxController operator = new CommandXboxController(DEVICES.OPERATOR_XBOX_CONTROLLER);
  public static final CommandXboxController driver = new CommandXboxController(DEVICES.DRIVE_XBOX_CONTROLLER);

  public static void configureBindings(
      RobotStateController stateController,
      SwerveDrive swerveDrive, 
      Transfer transfer, 
      TransferInWheels transferInWheels, 
      TransferOutWheels transferOutWheels, 
      Shooter shooter, 
      ShooterWheels shooterWheels, 
      ShooterPivot shooterPivot, 
      FeedWheels feedWheels,
      Amp amp, 
      AmpPivot ampPivot, 
      AmpWheels ampWheels
      ) 
    {

    driver.a();
    driver.b();
    driver.x();
    driver.y(); // USED
    driver.start();
    driver.back();
    driver.leftBumper();
    driver.rightBumper();
    driver.leftStick(); // USED
    driver.rightStick(); // USED
    driver.povCenter(); // USED
    driver.povUp(); // USED
    driver.povDown(); // USED
    driver.povLeft(); // USED
    driver.povRight(); // USED
    driver.leftTrigger(); // USED
    driver.rightTrigger(); // USED
    swerveDrive.setDefaultCommand(new XBoxSwerve(swerveDrive, driver.getHID()));
    
    operator.a();
    operator.b();
    operator.x();
    operator.y();
    operator.start().whileTrue(stateController.setState(RobotStateController.State.PLACE_AMP));
    operator.back().whileTrue(stateController.setState(RobotStateController.State.LEAVE_AMP));
    operator.leftBumper();
    operator.rightBumper().whileTrue(stateController.setState(RobotStateController.State.INTAKE));
    operator.leftStick().whileTrue(stateController.setState(RobotStateController.State.PREPARE_AMP));
    operator.rightStick().whileTrue(stateController.setState(RobotStateController.State.PREPARE_SPEAKER));
    operator.povCenter();
    operator.povUp();
    operator.povDown();
    operator.povLeft();
    operator.povRight();
    operator.leftTrigger().whileTrue(stateController.setState(RobotStateController.State.AIM_SPEAKER));
    operator.rightTrigger().whileTrue(stateController.setState(RobotStateController.State.SHOOT_SPEAKER));
  }
}
