// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DEVICES;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.Logging.Logger;
import frc.robot.subsystems.vision.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands
  private final XboxController XboxController = new XboxController(DEVICES.USB_XBOX_CONTROLLER);
  private final SwerveDrive swerveDrive = new SwerveDrive();
  // private final Limelight limelight = new Limelight("testone");
  private final ChoreoTrajectory traj;
  private Field2d field;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    Logger.log("constants", this, Constants.class);
    Logger.autoLog("PDH", new PowerDistribution(CAN.PDH, ModuleType.kRev));
    Logger.startLog();
    swerveDrive.setDefaultCommand(new XBoxSwerve(swerveDrive, () -> XboxController));
    field = swerveDrive.getField();
    // Configure the trigger bindings
    configureBindings();

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

    // auto stuff
    traj = Choreo.getTrajectory("speaker");

    field.getObject("traj").setPoses(
      traj.getInitialPose(), traj.getFinalPose()
    );
    field.getObject("trajPoses").setPoses(
      traj.getPoses()
    );
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    // Create a proportional controller to correct errors in the robot's rotation
    var rotationController = new PIDController(Constants.SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kP, 0, 0);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    swerveDrive.resetPose(traj.getInitialPose());

    Command swerveCommand = Choreo.choreoSwerveCommand(
        // Choreo trajectory to follow
        traj,

        // A supplier that returns the current field-relative pose of the robot based on the wheel
        // and vision odometry
        () -> swerveDrive.getPose(),

        // PIDControllers for correcting errors in field-relative translation (input: X or Y error in
        // meters, output: m/s).
        new PIDController(Constants.SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP, 0.0, 0.0),
        new PIDController(Constants.SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP, 0.0, 0.0),

        // PIDController to correct for rotation error (input: heading error in radians, output: rad/s)
        rotationController,

        // A consumer which drives the robot in robot-relative coordinates
        (ChassisSpeeds speeds) -> swerveDrive.drive(speeds),
        
        // A supplier which returns whether or not to mirror the path based on alliance (this assumes
        // the path is created for the blue alliance)
        () -> false,

        // The subsystem(s) to require, typically your drive subsystem only
        swerveDrive
    );

    // Create a command sequence to follow the trajectory then stop the robot
    return Commands.sequence(
        Commands.runOnce(() -> System.out.println("===== STARTING AUTO =====")),
        Commands.runOnce(() -> swerveDrive.resetPose(traj.getInitialPose())),
        swerveCommand
        //swerveDrive.run(() -> swerveDrive.drive(0, 0, 0))
    );

    // return null;
    // return new CharacterizeSwerve(swerveDrive);
    
    // return new FeedForwardCharacterization(swerveDrive, swerveDrive::runCharacterization, swerveDrive::getCharacterizationVelocity);
    // return swerveDrive.followPathCommand("Test Path");
    
    // TODO: Do we still need this random driving code? If we want to keep it, we could move it to another
    // function instead of leaving it commented out

    // int numPoints = 3;
    // List<Pose2d> randomPoints = new ArrayList<Pose2d>();
    // Random rand = new Random();
    // for (int i = 0; i < numPoints; i++) {
    //   randomPoints.add(new Pose2d(
    //     new Translation2d(
    //       rand.nextDouble() * 16.0,
    //       rand.nextDouble() * 8.0
    //     ),
    //     Rotation2d.fromRadians(
    //       0.0
    //     )
    //   ));
    // }

    // return null;
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
