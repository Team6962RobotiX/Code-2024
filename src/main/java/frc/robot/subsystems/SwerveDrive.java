// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.LOGGING;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_MATH;
import frc.robot.utils.Logger;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule[] modules = new SwerveModule[4];
  private SwerveModuleSim[] simulatedModules = new SwerveModuleSim[4];
  private AHRS gyro;
  private SwerveDriveKinematics kinematics = SWERVE_MATH.getKinematics();
  private SwerveDrivePoseEstimator poseEstimator;
  private int moduleCount = SWERVE_DRIVE.MODULE_NAMES.length;
  private PowerDistribution PDH = new PowerDistribution(CAN.PDH, ModuleType.kRev);
  private Field2d field = new Field2d();
  private double lastTimestamp = Timer.getFPGATimestamp();
  public Rotation2d heading = Rotation2d.fromDegrees(-90);

  public SwerveDrive() {
    try {
      gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), false);
    }

    for (int i = 0; i < moduleCount; i++)
      modules[i] = new SwerveModule(i);
    for (int i = 0; i < moduleCount; i++)
      simulatedModules[i] = new SwerveModuleSim(i);
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation2d(), getModulePositions(), SWERVE_DRIVE.STARTING_POSE);

    SmartDashboard.putData("Field", field);

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();
  }

  @Override
  public void periodic() {
    double simBatteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(getCurrent());
    RoboRioSim.setVInVoltage(simBatteryVoltage);
    if (simBatteryVoltage < RoboRioSim.getBrownoutVoltage()) {
      System.out.println("BROWNOUT!");
    }

    double timeDelta = Timer.getFPGATimestamp() - lastTimestamp;
    lastTimestamp += timeDelta;

    for (SwerveModule module : modules) module.update();
    poseEstimator.update(getRotation2d(), getModulePositions());
    FieldObject2d modulesObject = field.getObject("Swerve Modules");
    Pose2d[] modulePoses = new Pose2d[4];
    for (SwerveModule module : modules) {
      modulePoses[module.id] = module.getPose(getPose());
    }
    modulesObject.setPoses(modulePoses);
    field.setRobotPose(getPose());
    if (LOGGING.ENABLE_SWERVE_DRIVE)
      log("/swerveDrive");
    if (LOGGING.ENABLE_PDH)
      Logger.logPDH("/powerDistribution", PDH);
    if (LOGGING.ENABLE_ROBOT_CONTROLLER)
      Logger.logRobotController("/robotController");

    if (gyro.isConnected() && !RobotBase.isSimulation()) {
      heading = gyro.getRotation2d();
    } else {
      heading = heading.plus(new Rotation2d(getMeasuredChassisSpeeds().omegaRadiansPerSecond * timeDelta));
    }
  }

  @Override
  public void simulationPeriodic() {
    modules = simulatedModules;
    System.out.println(SWERVE_DRIVE.DRIVE_MOTOR_MOTION_PROFILE.kFF * SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION * (60 / 12));
  }

  /**
   * Drive the robot relative to itself
   * @param forwardVelocity (measured in m/s)
   * @param strafeVelocity (measured in m/s)
   * @param angularVelocity (measured in rad/s)
   */
  public void robotOrientedDrive(double forwardVelocity, double strafeVelocity, double angularVelocity) {
    Rotation2d robotAngle = getRotation2d();
    fieldOrientedDrive(forwardVelocity * robotAngle.getCos() - strafeVelocity * robotAngle.getSin(), forwardVelocity * robotAngle.getSin() + strafeVelocity * robotAngle.getCos(), angularVelocity);
  }

  /** 
   * Drive the robot relative to the field
   * @param xVelocity left-right velocity relative to the driver (measured in m/s)
   * @param yVelocity forward-backward velocity relative to the driver (measured in m/s)
   * @param angularVelocity rotational velocity (rad/s)
   */
  public void fieldOrientedDrive(double xVelocity, double yVelocity, double angularVelocity) {
    Rotation2d robotAngle = getRotation2d();
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, robotAngle);
    SwerveModuleState moduleStates[] = kinematics.toSwerveModuleStates(speeds);
    driveModules(moduleStates);
  }

  /**
   * Tells the modules what speed and direction to run at
   * @param moduleStates The target module states
   */
  public void driveModules(SwerveModuleState[] moduleStates) {
    // Account for turning while moving not being perfectly straight
    ChassisSpeeds velocity = kinematics.toChassisSpeeds(moduleStates);
    double dtConstant = SWERVE_DRIVE.ROTATION_ERROR_COMPENSATION;
    Pose2d robotPoseVel = new Pose2d(velocity.vxMetersPerSecond * dtConstant, velocity.vyMetersPerSecond * dtConstant, Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * dtConstant));
    Twist2d twistVel = SWERVE_MATH.PoseLog(robotPoseVel);
    velocity = new ChassisSpeeds(twistVel.dx / dtConstant, twistVel.dy / dtConstant, twistVel.dtheta / dtConstant);
    moduleStates = kinematics.toSwerveModuleStates(velocity);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveModule.motorPowerToDriveVelocity(SWERVE_DRIVE.MOTOR_POWER_HARD_CAP));
    for (int i = 0; i < moduleCount; i++)
      modules[i].drive(moduleStates[i]);
  }

  /**
   * This creates an "X" pattern with the wheels which makes the robot very hard to move
   */
  public void parkModules() {
    modules[0].drive(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[1].drive(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[2].drive(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[3].drive(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
  }

  /**
   * Resets the odometer position to a given position
   * @param pose Desired position
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void addVisionMeasurement(Pose2d visionMeasurement) {
    poseEstimator.addVisionMeasurement(visionMeasurement, Timer.getFPGATimestamp());
  }

  /**
   * Stops all motors on all modules
   */
  public void stopModules() {
    for (SwerveModule module : modules)
      module.stop();
  }

  /**
   * @return Target chassis x, y, and rotational velocity
   */
  public ChassisSpeeds getTargetChassisSpeeds() {
    return kinematics.toChassisSpeeds(getTargetModuleStates());
  }

  /**
   * @return Measured chassis x, y, and rotational velocity
   */
  public ChassisSpeeds getMeasuredChassisSpeeds() {
    return kinematics.toChassisSpeeds(getMeasuredModuleStates());
  }

  /**
   * @return Measured module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[moduleCount];
    for (int i = 0; i < moduleCount; i++) {
      positions[i] = modules[i].getModulePosition();
    }
    return positions;
  }

  /**
   * @return Target module states (speed and direction)
   */
  public SwerveModuleState[] getTargetModuleStates() {
    SwerveModuleState[] targetStates = new SwerveModuleState[moduleCount];
    for (int i = 0; i < moduleCount; i++) {
      targetStates[i] = modules[i].getTargetState();
    }
    return targetStates;
  }

  /**
   * @return Measured module states (speed and direction)
   */
  public SwerveModuleState[] getMeasuredModuleStates() {
    SwerveModuleState[] measuredStates = new SwerveModuleState[moduleCount];
    for (int i = 0; i < moduleCount; i++) {
      measuredStates[i] = modules[i].getMeasuredState();
    }
    return measuredStates;
  }

  /**
   * @return All SwerveModule objects
   */
  public SwerveModule[] getModules() {
    return modules;
  }

  /**
   * @return Total current through all modules
   */
  public double getCurrent() {
    double totalCurrent = 0.0;
    for (SwerveModule module : modules)
      totalCurrent += module.getCurrent();
    return totalCurrent;
  }

  /**
   * @return NavX AHRS IMU Gyro
   */
  public AHRS getGyro() {
    return gyro;
  }

  /**
   * @return Resets gyro heading
   */
  public void setHeading(Rotation2d newHeading) {
    heading = newHeading;
    gyro.reset();
    gyro.setAngleAdjustment(newHeading.getDegrees());
  }

  public void zeroHeading() {
    setHeading(SWERVE_DRIVE.STARTING_ANGLE_OFFSET);
  }

  /**
   * @return Returns gyro heading as a Rotation2d
   */
  public Rotation2d getRotation2d() {
    return heading;
  }

  /**
   * @return Get gyro heading in radians (-pi - pi)
   */
  public double getHeading() {
    return SWERVE_MATH.clampRadians(getRotation2d().getRadians());
  }

  /**
   * @return Get pose on the field from odometer as a Pose2d
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Logs all modules and module states to network tables for saving later
   */
  public void log(String path) {
    Logger.logNavX(path + "/gyro", getGyro());
    Logger.logValue(path + "/heading", getRotation2d().getRadians());
    Logger.logValue(path + "/totalCurrent", getCurrent());
    Logger.logPose(path + "/pose", getPose());
    Logger.logModuleStates(path + "/moduleStates", getTargetModuleStates(), getMeasuredModuleStates(), getModulePositions());

    for (SwerveModule module : modules) {
      module.log(path + "/modules/" + module.getName());
    }
  }

  /**
   * @param driveVelocity drive velocity in m/s
   * @return rotational velocity in rad/s
   */
  public static double wheelVelocityToRotationalVelocity(double driveVelocity) {
    return driveVelocity / Math.hypot(SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE / 2.0);
  }

  /**
   * @param rotationalVelocity rotational velocity in rad/s
   * @return drive velocity in m/s
   */
  public static double rotationalVelocityToWheelVelocity(double rotationalVelocity) {
    return rotationalVelocity * Math.hypot(SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE / 2.0);
  }

  public Command fullAuto(String pathName, HashMap<String, Command> eventMap) {
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(SWERVE_DRIVE.AUTONOMOUS_VELOCITY, SWERVE_DRIVE.AUTONOMOUS_ACCELERATION));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(this::getPose, // Pose2d supplier
        this::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
        SWERVE_MATH.getKinematics(), // SwerveDriveKinematics
        new PIDConstants(
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kI,
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kD
        ), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(
          SWERVE_DRIVE.AUTONOMOUS_ROTATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS_ROTATION_GAINS.kI,
          SWERVE_DRIVE.AUTONOMOUS_ROTATION_GAINS.kD
        ), // PID constants to correct for rotation error (used to create the rotation controller)
        this::driveModules, // Module states consumer used to output to the drive subsystem
        eventMap, true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        this // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup).andThen(this::stopModules, this);
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
    return new SequentialCommandGroup(new PPSwerveControllerCommand(traj, this::getPose, // Pose supplier
        SWERVE_MATH.getKinematics(), // SwerveDriveKinematics
        new PIDController(
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kI,
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kD
        ), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        new PIDController(
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kI,
          SWERVE_DRIVE.AUTONOMOUS_TRANSLATION_GAINS.kD
        ), // Y controller (usually the same values as X controller)
        new PIDController(
          SWERVE_DRIVE.AUTONOMOUS_ROTATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS_ROTATION_GAINS.kI,
          SWERVE_DRIVE.AUTONOMOUS_ROTATION_GAINS.kD
        ), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        this::driveModules, // Module states consumer
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        this // Requires this drive subsystem
    ));
  }

  public PathPlannerTrajectory generateTrajectoryFieldRelative(List<PathPoint> pathPoints) {
    return PathPlanner.generatePath(
      new PathConstraints(SWERVE_DRIVE.AUTONOMOUS_VELOCITY, SWERVE_DRIVE.AUTONOMOUS_ACCELERATION), 
      pathPoints
    );
  }

  public PathPlannerTrajectory generateTrajectoryRobotRelative(List<PathPoint> pathPoints) {
    List<PathPoint> newPoints = new ArrayList<>();
    for (PathPoint point : pathPoints) {
      newPoints.add(
        new PathPoint(
          point.position.plus(getPose().getTranslation()),
          point.heading,
          point.holonomicRotation,
          point.velocityOverride
        )
      );
    }
    return PathPlanner.generatePath(
      new PathConstraints(SWERVE_DRIVE.AUTONOMOUS_VELOCITY, SWERVE_DRIVE.AUTONOMOUS_ACCELERATION), 
      newPoints
    );
  }
}