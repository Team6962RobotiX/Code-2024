// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DEVICES;
import frc.robot.Constants.NEO;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.DRIVE_MOTOR_PROFILE;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModule;
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

    System.out.println(Constants.SWERVE_DRIVE.PHYSICS.ROTATIONAL_INERTIA);


    // MOI = (DRIVE_MOTOR_CONFIG.kA * G * DCMotor.getNEO(1).KtNMPerAmp) / DCMotor.getNEO(1).rOhms;
    
    DCMotor motor = DCMotor.getNEO(1);
    // // torque
    // motor.getTorque(motor.getCurrent(0, 12.0));
    // double t = (NEO.STALL_TORQUE * (1.0 / SWERVE_DRIVE.DRIVE_MOTOR_GEAR_RATIO) * SWERVE_DRIVE.MODULE_COUNT) / (SWERVE_DRIVE.WHEEL_DIAMETER / 2.0) * SWERVE_DRIVE.ROBOT_MASS;
    // double f = t / r;
    // double a = f * n / m;

    // ((a * motor.KtNMPerAmp * g) / r) * n / m

    // double kV = (SWERVE_DRIVE.ROBOT_MASS * motor.rOhms * (SWERVE_DRIVE.WHEEL_DIAMETER / 2.0) * SWERVE_DRIVE.DRIVE_MOTOR_GEAR_RATIO) / (SWERVE_DRIVE.MODULE_COUNT * motor.KtNMPerAmp);

    // double t = 1 / o * v * q;
    // double f = (1 / o * v * q) / r;
    // double a = ((1 / o * v * q) / r) * n / m;

    // System.out.println(1.0 / motor.getTorque(motor.getCurrent(0, 12.0)) / (SWERVE_DRIVE.WHEEL_DIAMETER / 2.0) * 4.0 / SWERVE_DRIVE.ROBOT_MASS);
    // System.out.println(SwerveModule.calcAcceleration(NEO.STALL_CURRENT));
    // System.out.println((NEO.STALL_TORQUE * (1.0 / SWERVE_DRIVE.DRIVE_MOTOR_GEAR_RATIO) * SWERVE_DRIVE.MODULE_COUNT) / (SWERVE_DRIVE.WHEEL_DIAMETER / 2.0) / SWERVE_DRIVE.ROBOT_MASS);
    // System.out.println(1.0 / SWERVE_DRIVE.DRIVE_MOTOR_PROFILE.RAMP_RATE);
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    // return null;
    // return new CharacterizeSwerve(swerveDrive);
    
    // return new FeedForwardCharacterization(swerveDrive, swerveDrive::runCharacterization, swerveDrive::getCharacterizationVelocity);
    // return swerveDrive.followPathCommand("Test Path");
    
    int numPoints = 3;
    List<Pose2d> randomPoints = new ArrayList<Pose2d>();
    Random rand = new Random();
    for (int i = 0; i < numPoints; i++) {
      randomPoints.add(new Pose2d(
        new Translation2d(
          rand.nextDouble() * 16.0,
          rand.nextDouble() * 8.0
        ),
        Rotation2d.fromRadians(
          0.0
        )
      ));
    }

    return null;
    // return swerveDrive.followPathCommand(randomPoints);

    // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
    //     new Pose2d(rand.nextDouble() * 16.0, rand.nextDouble() * 8.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(rand.nextDouble() * 16.0, rand.nextDouble() * 8.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(rand.nextDouble() * 16.0, rand.nextDouble() * 8.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(rand.nextDouble() * 16.0, rand.nextDouble() * 8.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(rand.nextDouble() * 16.0, rand.nextDouble() * 8.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(rand.nextDouble() * 16.0, rand.nextDouble() * 8.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(rand.nextDouble() * 16.0, rand.nextDouble() * 8.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(rand.nextDouble() * 16.0, rand.nextDouble() * 8.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(rand.nextDouble() * 16.0, rand.nextDouble() * 8.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(rand.nextDouble() * 16.0, rand.nextDouble() * 8.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(rand.nextDouble() * 16.0, rand.nextDouble() * 8.0, Rotation2d.fromDegrees(0))
    // );

    // PathPlannerPath path = new PathPlannerPath(
    //     bezierPoints,
    //     new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
    //     new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    // );

    // return swerveDrive.followPathCommand(path);
  }

  public void disabledPeriodic() {
  }
}
