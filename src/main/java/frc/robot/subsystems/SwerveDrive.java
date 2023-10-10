// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.LOGGING;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SwerveMath;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkMaxPIDFTuner;
import frc.robot.utils.SwerveModule;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule[] modules = new SwerveModule[4];
  private AHRS gyro;
  private SwerveDriveKinematics kinematics = SwerveMath.getKinematics();
  private SwerveDriveOdometry odometer;
  private int moduleCount = SWERVE_DRIVE.MODULE_NAMES.length;
  private PowerDistribution PDH = new PowerDistribution(CAN.PDH, ModuleType.kRev);;

  private SparkMaxPIDFTuner drivePIDFTuner;
  private SparkMaxPIDFTuner steerPIDFTuner;

  public SwerveDrive() {
    try {
      gyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), false);
    }

    for (int i = 0; i < moduleCount; i++) modules[i] = new SwerveModule(i);

    odometer = new SwerveDriveOdometry(
        kinematics,
        getRotation2d(),
        getModulePositions(),
        SWERVE_DRIVE.STARTING_POSE);

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {}
    }).start();

    drivePIDFTuner = new SparkMaxPIDFTuner(
      "Swerve Drive",
      modules[0].getDrivePIDFController(),
      new SparkMaxPIDController[] { 
        modules[1].getDrivePIDFController(),
        modules[2].getDrivePIDFController(),
        modules[3].getDrivePIDFController()
      }, 
      modules[0]::getTargetVelocity, 
      modules[0]::getVelocity
    );

    steerPIDFTuner = new SparkMaxPIDFTuner(
      "Swerve Steer",
      modules[0].getSteerPIDFController(),
      new SparkMaxPIDController[] { 
        modules[1].getSteerPIDFController(),
        modules[2].getSteerPIDFController(),
        modules[3].getSteerPIDFController()
      }, 
      modules[0]::getTargetAngle, 
      modules[0]::getSteerRadians
    );
  }

  @Override
  public void periodic() {
    odometer.update(getRotation2d(), getModulePositions());
    if (LOGGING.ENABLE_DRIVE) log("/swerveDrive");
    if (LOGGING.ENABLE_PDH) Logger.logPDH("/powerDistribution", PDH);
    if (LOGGING.ENABLE_ROBOT_CONTROLLER) Logger.logRobotController("/robotController");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
  /**
   * Drive the robot relative to itself
   * @param forwardVelocity (measured in m/s)
   * @param strafeVelocity (measured in m/s)
   * @param angularVelocity (measured in rad/s)
   */
  public void robotOrientedDrive(double forwardVelocity, double strafeVelocity, double angularVelocity) {
    Rotation2d robotAngle = getRotation2d();
    fieldOrientedDrive(
        forwardVelocity * robotAngle.getCos() - strafeVelocity * robotAngle.getSin(),
        forwardVelocity * robotAngle.getSin() + strafeVelocity * robotAngle.getCos(),
        angularVelocity);
  }

  
  /** 
   * Drive the robot relative to the field
   * @param xVelocity left-right velocity relative to the driver (measured in m/s)
   * @param yVelocity forward-backward velocity relative to the driver (measured in m/s)
   * @param angularVelocity rotational velocity (rad/s)
   */
  public void fieldOrientedDrive(double xVelocity, double yVelocity, double angularVelocity) { // m/s and rad/s    
    Rotation2d robotAngle = getRotation2d();
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, robotAngle);
    SwerveModuleState moduleStates[] = kinematics.toSwerveModuleStates(speeds);
    
    boolean moving = false;
    for (SwerveModuleState moduleState : moduleStates) {
      if (moduleState.speedMetersPerSecond > SWERVE_DRIVE.VELOCITY_DEADBAND) {
        moving = true;
        break;
      }
    }

    if (!moving && !gyro.isMoving()) {
      groundModules();
      return;
    }
      
    driveModules(moduleStates);
  }

  
  /**
   * Tells the modules what speed and direction to run at
   * @param moduleStates The target module states
   */
  public void driveModules(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveModule.powerToDriveVelocity(SWERVE_DRIVE.MOTOR_POWER_HARD_CAP));
    for (int i = 0; i < moduleCount; i++) modules[i].drive(moduleStates[i]);
  }

  /**
   * This creates an "X" pattern with the wheels which makes the robot very hard to move
   */
  public void groundModules() {
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
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Stops all motors on all modules
   */
  public void stopModules() {
    for (SwerveModule module : modules) module.stop();
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
    for (SwerveModule module : modules) totalCurrent += module.getCurrent();
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
  public void zeroHeading() {
    gyro.reset();
    gyro.setAngleAdjustment(SWERVE_DRIVE.STARTING_ANGLE_OFFSET);
  }

  /**
   * @return Returns gyro heading as a Rotation2d
   */
  public Rotation2d getRotation2d() {
    if (gyro.isConnected()) return new Rotation2d();
    return gyro.getRotation2d();
  }

  /**
   * @return Get gyro heading in degrees (-180 - 180)
   */
  public double getHeading() {
    return SwerveMath.clampDegrees(getRotation2d().getDegrees());
  }

  /**
   * @return Get pose on the field from odometer as a Pose2d
   */
  public Pose2d getPose() {
    return odometer.getPoseMeters();
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
  // Convert rotation velocity in rad/s to wheel velocity in m/s
  public static double rotationalVelocityToWheelVelocity(double rotationalVelocity) {
    return rotationalVelocity * Math.hypot(SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE / 2.0);
  }
}