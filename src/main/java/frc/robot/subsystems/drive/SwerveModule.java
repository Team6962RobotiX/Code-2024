// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.CAN;
import frc.robot.Constants.LOGGING;
import frc.robot.Constants.NEO;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.DRIVE_MOTOR_CONFIG;
import frc.robot.Constants.SWERVE_DRIVE.STEER_MOTOR_CONFIG;
import frc.robot.Constants.SWERVE_MATH;
import frc.robot.util.SparkMax;
import frc.robot.util.SparkMaxEncoder;
import frc.robot.util.SparkMaxPID;
import frc.robot.util.Logging.ErrorLogging;
import frc.robot.util.Logging.Logger;

public class SwerveModule extends MotorSafety {
  private SparkMax driveMotor, steerMotor;
  private RelativeEncoder driveEncoder, steerEncoder;
  private CANCoder absoluteSteerEncoder;
  private SparkMaxPIDController driveController, steerController;
  private SwerveModuleState state = new SwerveModuleState();
  private String name;
  
  public int id;

  public SwerveModule(int id) {
    this.id = id;
    name = SWERVE_DRIVE.MODULE_NAMES[id];

    Logger.autoLog("current", this::getCurrent);
    Logger.autoLog("measuredAngle", this::getSteerRadians);
    Logger.autoLog("measuredVelocity", this::getVelocity);
    Logger.autoLog("targetAngle", this::getTargetAngle);
    Logger.autoLog("targetVelocity", this::getTargetVelocity);
    Logger.autoLog("getAbsoluteSteerRadians", this::getAbsoluteSteerRadians);
    Logger.autoLog("measuredState", this::getMeasuredState);

    if (RobotBase.isSimulation()) return;
    // MOTOR SETUP
    driveMotor = new SparkMax(CAN.SWERVE_DRIVE_SPARK_MAX[id], MotorType.kBrushless);
    steerMotor = new SparkMax(CAN.SWERVE_STEER_SPARK_MAX[id], MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();

    driveMotor.setIdleMode(IdleMode.kBrake);
    steerMotor.setIdleMode(IdleMode.kBrake);

    steerMotor.setInverted(true);

    driveMotor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, DRIVE_MOTOR_CONFIG.currentLimit);
    steerMotor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, STEER_MOTOR_CONFIG.currentLimit);

    driveMotor.enableVoltageCompensation(12.0);
    steerMotor.enableVoltageCompensation(12.0);

    // ENCODER SETUP
    absoluteSteerEncoder = new CANCoder(CAN.SWERVE_STEER_CANCODERS[id]);
    steerEncoder = steerMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION);
    driveEncoder.setVelocityConversionFactor(SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION / 60.0);
    steerEncoder.setPositionConversionFactor(SWERVE_DRIVE.STEER_MOTOR_RADIANS_PER_REVOLUTION);
    steerEncoder.setVelocityConversionFactor(SWERVE_DRIVE.STEER_MOTOR_RADIANS_PER_REVOLUTION / 60.0);

    CANCoderConfiguration CANCoderConfig = new CANCoderConfiguration();
    CANCoderConfig.magnetOffsetDegrees = SWERVE_DRIVE.STEER_ENCODER_OFFSETS[id];
    CANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    absoluteSteerEncoder.configAllSettings(CANCoderConfig);


    // DRIVE MOTION PROFILING
    driveController = driveMotor.getPIDController();
    driveController.setP (DRIVE_MOTOR_CONFIG.kP,  0);
    driveController.setI (DRIVE_MOTOR_CONFIG.kI,  0);
    driveController.setD (DRIVE_MOTOR_CONFIG.kD,  0);
    driveController.setFF(DRIVE_MOTOR_CONFIG.kFF * driveEncoder.getVelocityConversionFactor(), 0);
    driveController.setSmartMotionMaxVelocity(DRIVE_MOTOR_CONFIG.maxVelocity / driveEncoder.getVelocityConversionFactor(), 0);
    driveController.setSmartMotionMaxAccel(DRIVE_MOTOR_CONFIG.maxAcceleration / driveEncoder.getVelocityConversionFactor(), 0);
    driveController.setSmartMotionAllowedClosedLoopError(0.05, 0);

    driveController.setOutputRange(-SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, 0);
    driveController.setFeedbackDevice(driveEncoder);

    // STEER MOTION PROFILING
    steerController = steerMotor.getPIDController();
    steerController.setP (STEER_MOTOR_CONFIG.kP,  0);
    steerController.setI (STEER_MOTOR_CONFIG.kI,  0);
    steerController.setD (STEER_MOTOR_CONFIG.kD,  0);
    steerController.setFF(STEER_MOTOR_CONFIG.kFF * steerEncoder.getVelocityConversionFactor(), 0);
    steerController.setSmartMotionMaxVelocity(STEER_MOTOR_CONFIG.maxVelocity / steerEncoder.getVelocityConversionFactor(), 0);
    steerController.setSmartMotionMaxAccel(STEER_MOTOR_CONFIG.maxAcceleration / steerEncoder.getVelocityConversionFactor(), 0);
    steerController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    steerController.setSmartMotionAllowedClosedLoopError(0.05, 0);

    steerController.setOutputRange(-SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, 0);
    steerController.setFeedbackDevice(steerEncoder);

    // steerController.setPositionPIDWrappingEnabled(true);
    // steerController.setPositionPIDWrappingMaxInput(Math.PI / steerEncoder.getPositionConversionFactor());
    // steerController.setPositionPIDWrappingMinInput(-Math.PI / steerEncoder.getPositionConversionFactor());

    // SAVE SETTINGS
    driveMotor.burnFlash();
    steerMotor.burnFlash(); 

    seedSteerEncoder();
  }

  /**
   * Drive the module with a state.
   * @param state The state (velocity and direction) we want to drive the module
   */
  public void drive(SwerveModuleState state, boolean raw) {
    // state = SwerveModuleState.optimize(state, getSteerRotation2d());
    if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) state.speedMetersPerSecond *= Math.cos(SWERVE_MATH.angleDistance(state.angle.getRadians(), getSteerRadians()));
    this.state = state;
    
    // Use onboard motion profiling controllers
    // driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    steerController.setReference(state.angle.getRadians(), ControlType.kSmartMotion);

    feed();
  }

  public void drive(SwerveModuleState state) {
    drive(state, false);
  }
  
  public void update() {
  }

  /**
   * Seeds the position of the built-in relative encoder with the absolute position of the steer CANCoder.
   * This is because the CANCoder polls at a lower rate than we'd like, so we essentially turn the relative encoder into an fast-updating absolute encoder.
   * Also the built-in SparkMaxPIDControllers require a compatible encoder to run the faster 1kHz closed loop 
   */
  public void seedSteerEncoder() {
    // steerEncoder.setPosition(getAbsoluteSteerRadians());
  }

  /**
   * @return Steering direction in radians (-PI - PI)
   */
  public double getSteerRadians() {
    // if (Math.abs(steerEncoder.getVelocity()) < SWERVE_DRIVE.VELOCITY_DEADBAND && SWERVE_MATH.angleDistance(this.state.angle.getRadians(), steerEncoder.getPosition()) < Units.degreesToRadians(1.0)) seedSteerEncoder();
    // if (Math.abs(steerEncoder.getPosition()) > Math.PI) steerEncoder.setPosition(SWERVE_MATH.clampRadians(steerEncoder.getPosition()));
    return steerEncoder.getPosition();
  }

  /**
   * @return Steering direction from absolute encoder (CANCoder) in radians (-PI - PI)
   */
  public double getAbsoluteSteerRadians() {
    return Units.degreesToRadians(absoluteSteerEncoder.getAbsolutePosition());
  }

  /**
   * @return Steering direction in degrees (-180 - 180)
   */
  public Rotation2d getSteerRotation2d() {
    return Rotation2d.fromRadians(getSteerRadians());
  }

  /**
   * @return Drive wheel velocity in m/s
   */
  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  /**
   * @return Name of the module (set in constants file)
   */
  public String getName() {
    return name;
  }

  /**
   * @return The total current of both motors (measured in Amps)
   */
  public double getCurrent() {
    return driveMotor.getOutputCurrent() + steerMotor.getOutputCurrent();
  }

  /**
   * Stops power to both motors (not persistent)
   */
  public void stop() {
    steerMotor.stopMotor();
    driveMotor.stopMotor();
  }

  /**
   * For WPILib MotorSafety Class
   */
  public void stopMotor() {
    stop();
  }

  public String getDescription() {
    return getName() + " Swerve Module";
  }
  
  /**
   * @return The target SwerveModuleState
   */
  public SwerveModuleState getTargetState() {
    return state;
  }

  /**
   * @return The measured SwerveModuleState
   */
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(getVelocity(), getSteerRotation2d());
  }

  /**
   * @return The measured SwerveModulePosition
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getSteerRotation2d());
  }

  /**
   * @param power Motor power (-1.0 - 1.0)
   * @return Drive velocity
   */
  public static double motorPowerToDriveVelocity(double power) {
    return power / DRIVE_MOTOR_CONFIG.kFF;
  }

  public double getTargetVelocity() {
    return state.speedMetersPerSecond;
  }

  public double getTargetAngle() {
    return state.angle.getRadians();
  }

  public void runCharacterization(double volts) {
    steerController.setReference(0.0, ControlType.kPosition);
    driveMotor.setVoltage(volts);
    feed();
  }

  public Pose2d getPose(Pose2d robotPose) {
    Pose2d relativePose = new Pose2d();
    if (id == 0) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getSteerRotation2d()
    );
    if (id == 1) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getSteerRotation2d()
    );
    if (id == 2) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getSteerRotation2d()
    );
    if (id == 3) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getSteerRotation2d()
    );
    return relativePose.relativeTo(new Pose2d(
      new Translation2d(),
      robotPose.getRotation().times(-1.0)
    )).relativeTo( new Pose2d(
      -robotPose.getX(),
      -robotPose.getY(),
      new Rotation2d()
    ));
  }
}