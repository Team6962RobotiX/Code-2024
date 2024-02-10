// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.NEO;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.DRIVE_MOTOR_PROFILE;
import frc.robot.Constants.SWERVE_DRIVE.MODULE_CONFIG;
import frc.robot.Constants.SWERVE_DRIVE.STEER_MOTOR_PROFILE;
import frc.robot.util.ConfigUtils;
import frc.robot.util.MathUtils.SwerveMath;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax driveMotor, steerMotor;
  private RelativeEncoder driveEncoder, steerEncoder;
  private CANcoder absoluteSteerEncoder;
  private SparkPIDController drivePID, steerPID;
  private SwerveModuleState targetState = new SwerveModuleState();
  private SwerveModuleState lastDrivenState = new SwerveModuleState();
  private MODULE_CONFIG config;
  private String name;
  private int corner;

  private boolean isCalibrating = false;
  
  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(
    DRIVE_MOTOR_PROFILE.kS,
    DRIVE_MOTOR_PROFILE.kV,
    DRIVE_MOTOR_PROFILE.kA
  );

  public SwerveModule(MODULE_CONFIG config, int corner, String name) {
    this.config = config;
    this.corner = corner;
    this.name = name;

    if (RobotBase.isSimulation()) return;

    driveMotor           = new CANSparkMax(config.CAN_DRIVE(), MotorType.kBrushless);
    steerMotor           = new CANSparkMax(config.CAN_STEER(), MotorType.kBrushless);
    absoluteSteerEncoder = new CANcoder(config.CAN_ENCODER());
    steerEncoder         = steerMotor.getEncoder();
    driveEncoder         = driveMotor.getEncoder();
    drivePID             = driveMotor.getPIDController();
    steerPID             = steerMotor.getPIDController();

    double encoderOffset = config.ENCODER_OFFSET();
    switch (corner) {
      case 0:
        encoderOffset += 0.0;
        break;
      case 1:
        encoderOffset += 0.25;
        break;
      case 2:
        encoderOffset += -0.25;
        break;
      case 3:
        encoderOffset += 0.5;
        break;
      default:
    }
    encoderOffset %= 2;
    encoderOffset = (encoderOffset > 1.0) ? encoderOffset - 2.0 : (encoderOffset < -1.0) ? encoderOffset + 2.0 : encoderOffset;

    System.out.println(encoderOffset);

    MagnetSensorConfigs magConfig = new MagnetSensorConfigs();
    magConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
    magConfig.withMagnetOffset(encoderOffset);
    
    // Configure a lot of stuff, handling REVLibErrors gracefully
    ConfigUtils.configure(List.of(
      // Reset the drive motor controller to factory defaults
      () -> driveMotor.restoreFactoryDefaults(),
      // Make the drive motor automatically stop when idle instead of coasting
      () -> driveMotor.setIdleMode(IdleMode.kBrake),
      // Configure the drive motor to use 12V
      () -> driveMotor.enableVoltageCompensation(12.0),
      // Configure the drive motor to limit its current to prevent stalling
      () -> driveMotor.setSmartCurrentLimit(Math.min(DRIVE_MOTOR_PROFILE.CURRENT_LIMIT, NEO.SAFE_STALL_CURRENT), DRIVE_MOTOR_PROFILE.CURRENT_LIMIT),
      // Set the rate at which the drive motor can change speeds
      () -> driveMotor.setClosedLoopRampRate(DRIVE_MOTOR_PROFILE.RAMP_RATE),
      // Configure the drive motor encoder based on the wheel size and gearbox, allowing it to convert between
      // motor rotations and meters traveled on the field
      () -> driveEncoder.setPositionConversionFactor(SWERVE_DRIVE.DRIVE_ENCODER_CONVERSION_FACTOR),
      () -> driveEncoder.setVelocityConversionFactor(SWERVE_DRIVE.DRIVE_ENCODER_CONVERSION_FACTOR / 60.0),
      // Set the drive motor's proportional gain constant
      () -> drivePID.setP(DRIVE_MOTOR_PROFILE.kP, 0),
      () -> drivePID.setI(DRIVE_MOTOR_PROFILE.kI, 0),
      () -> drivePID.setD(DRIVE_MOTOR_PROFILE.kD, 0),
      () -> drivePID.setOutputRange(-SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, 0),
      // Write the drive motor configuration settings to flash
      () -> driveMotor.burnFlash(),

      // Reset the steering motor controller to factory defaults
      () -> steerMotor.restoreFactoryDefaults(),
      // Invert the steering motor
      () -> { steerMotor.setInverted(true); return true; },
      // Make the steering motor automatically stop when idle instead of continuing to turn
      () -> steerMotor.setIdleMode(IdleMode.kBrake),
      // Configure the steering motor to use 12V
      () -> steerMotor.enableVoltageCompensation(12.0),
      // Configure the steering motor to limit its current to prevent stalling
      () -> steerMotor.setSmartCurrentLimit(Math.min(STEER_MOTOR_PROFILE.CURRENT_LIMIT, NEO.SAFE_STALL_CURRENT), STEER_MOTOR_PROFILE.CURRENT_LIMIT),
      // Set the rate at which the steering motor can change speeds
      () -> steerMotor.setClosedLoopRampRate(STEER_MOTOR_PROFILE.RAMP_RATE),
      // Configure the steering motor encoder based on the gearbox, allowing it to convert between
      // motor rotations and change in the angle of the wheel
      () -> steerEncoder.setPositionConversionFactor(SWERVE_DRIVE.STEER_ENCODER_CONVERSION_FACTOR),
      () -> steerEncoder.setVelocityConversionFactor(SWERVE_DRIVE.STEER_ENCODER_CONVERSION_FACTOR / 60.0),
      // Set the steering motor's PID gain constants
      () -> steerPID.setP(STEER_MOTOR_PROFILE.kP, 0),
      () -> steerPID.setI(STEER_MOTOR_PROFILE.kI, 0),
      () -> steerPID.setD(STEER_MOTOR_PROFILE.kD, 0),
      // Because the steering motor turns the wheel in a circle, we need to wrap the PID controller's input
      // from -π to π.
      () -> steerPID.setPositionPIDWrappingEnabled(true),
      () -> steerPID.setPositionPIDWrappingMinInput(-Math.PI),
      () -> steerPID.setPositionPIDWrappingMaxInput(Math.PI),
      () -> steerPID.setOutputRange(-SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, SWERVE_DRIVE.MOTOR_POWER_HARD_CAP, 0),
      // Write the steering motor configuration settings to flash
      () -> steerMotor.burnFlash(),

      () -> absoluteSteerEncoder.getConfigurator().apply(magConfig)
    ));

    // driveMotor.setClosedLoopRampRate(Math.max((NEO.FREE_SPEED / 60.0) / ((9.80 * SWERVE_DRIVE.COEFFICIENT_OF_FRICTION) / SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION), DRIVE_MOTOR_CONFIG.RAMP_RATE))

    seedSteerEncoder();

    String logPath = "module_" + name + "/";
    Logger.autoLog(logPath + "current",                 () -> getTotalCurrent());
    Logger.autoLog(logPath + "driveVoltage",            () -> driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
    Logger.autoLog(logPath + "steerVoltage",            () -> steerMotor.getAppliedOutput() * steerMotor.getBusVoltage());
    Logger.autoLog(logPath + "getAbsoluteSteerDegrees", () -> getTrueSteerDirection().getDegrees());
    Logger.autoLog(logPath + "measuredState",           () -> getMeasuredState());
    Logger.autoLog(logPath + "measuredAngle",           () -> getMeasuredState().angle.getDegrees());
    Logger.autoLog(logPath + "measuredVelocity",        () -> getMeasuredState().speedMetersPerSecond);
    Logger.autoLog(logPath + "targetState",             () -> getTargetState());
    Logger.autoLog(logPath + "targetAngle",             () -> getTargetState().angle.getDegrees());
    Logger.autoLog(logPath + "targetVelocity",          () -> getTargetState().speedMetersPerSecond);

    StatusChecks.addCheck(name + " Swerve Module Drive Motor", () -> driveMotor.getFaults() == 0);
    StatusChecks.addCheck(name + " Swerve Module Steer Motor", () -> steerMotor.getFaults() == 0);
    StatusChecks.addCheck(name + " Swerve Module CanCoder", () -> absoluteSteerEncoder.getFaultField().getValue() == 0);
  }


  public void periodic() {
    if (isCalibrating) return;

    if (Math.abs(getMeasuredState().speedMetersPerSecond) < SWERVE_DRIVE.VELOCITY_DEADBAND && SwerveMath.angleDistance(getMeasuredState().angle.getRadians(), getTargetState().angle.getRadians()) < Units.degreesToRadians(1.0)) {
      seedSteerEncoder();
    }
    
    drive(targetState);
  }
  
  public void drive(SwerveModuleState state) {
    double speedMetersPerSecond = state.speedMetersPerSecond;
    double radians = state.angle.getRadians();
    
    if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) {
      speedMetersPerSecond *= Math.cos(SwerveMath.angleDistance(radians, getMeasuredState().angle.getRadians()));
    }
    
    double wheelAcceleration = (speedMetersPerSecond - lastDrivenState.speedMetersPerSecond) / 0.02;
    
    drivePID.setReference(
      speedMetersPerSecond,
      ControlType.kVelocity, 
      0,
      driveFF.calculate(speedMetersPerSecond, 0.0),
      ArbFFUnits.kVoltage
    );
    
    steerPID.setReference(
      radians,
      ControlType.kPosition,
      0
    );

    lastDrivenState = new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromRadians(radians));
  }
    
  public void setTargetState(SwerveModuleState state) {
    targetState = SwerveModuleState.optimize(state, getMeasuredState().angle);
  }
    
  public void stop() {
    targetState = new SwerveModuleState(0.0, getMeasuredState().angle);
    // steerMotor.stopMotor();
    // driveMotor.stopMotor();
  }
  
  /**
   * Seeds the position of the built-in relative encoder with the absolute position of the steer CANCoder.
   * This is because the CANCoder polls at a lower rate than we'd like, so we essentially turn the relative encoder into an fast-updating absolute encoder.
   * Also the built-in SparkMaxPIDControllers require a compatible encoder to run the faster 1kHz closed loop 
   */
  public void seedSteerEncoder() {
    steerEncoder.setPosition(getTrueSteerDirection().getRadians());
  }
  
  private Rotation2d getTrueSteerDirection() {
    return Rotation2d.fromRotations(absoluteSteerEncoder.getAbsolutePosition().getValue());
  }

  public SwerveModuleState getTargetState() {
    return targetState;
  }
  
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromRadians(MathUtil.angleModulus(steerEncoder.getPosition())));
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getMeasuredState().angle);
  }

  public static double calcWheelVelocity(double power) {
    return (power * 12.0) / DRIVE_MOTOR_PROFILE.kV;
  }

  public double getTotalCurrent() {
    return driveMotor.getOutputCurrent() + steerMotor.getOutputCurrent();
  }
  
  public Pose2d getPose(Pose2d robotPose) {
    Pose2d relativePose = new Pose2d();
    if (corner == 0) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 1) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 2) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 3) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
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

  public Command calibrateSteerMotor() {
    SysIdRoutine calibrationRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
          steerMotor.setVoltage(volts.in(Volts));
        },
        log -> {
          log.motor("module-steer-" + name)
              .voltage(Volts.of(steerMotor.getBusVoltage() * steerMotor.getAppliedOutput()))
              .angularPosition(Radians.of(steerEncoder.getPosition()))
              .angularVelocity(RadiansPerSecond.of(steerEncoder.getVelocity()));
        },
        this
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> doCalibrationPrep()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> steerMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> steerMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> steerMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> steerMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> undoCalibrationPrep())
    );
  }

  public Command calibrateDriveMotor() {
    SysIdRoutine calibrationRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
          driveMotor.setVoltage(volts.in(Volts));
        },
        log -> {
          log.motor("module-drive-" + name)
              .voltage(Volts.of(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput()))
              .linearPosition(Meters.of(driveEncoder.getPosition()))
              .linearVelocity(MetersPerSecond.of(driveEncoder.getVelocity()));
        },
        this
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> doCalibrationPrep()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> driveMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> driveMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> driveMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> driveMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> undoCalibrationPrep())
    );
  }

  private void doCalibrationPrep() {
    driveMotor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, NEO.SAFE_STALL_CURRENT);
    steerMotor.setSmartCurrentLimit(NEO.SAFE_STALL_CURRENT, NEO.SAFE_STALL_CURRENT);
    isCalibrating = true;
  }

  private void undoCalibrationPrep() {
    driveMotor.setSmartCurrentLimit(Math.min(DRIVE_MOTOR_PROFILE.CURRENT_LIMIT, NEO.SAFE_STALL_CURRENT), DRIVE_MOTOR_PROFILE.CURRENT_LIMIT);
    steerMotor.setSmartCurrentLimit(Math.min(STEER_MOTOR_PROFILE.CURRENT_LIMIT, NEO.SAFE_STALL_CURRENT), STEER_MOTOR_PROFILE.CURRENT_LIMIT);
    isCalibrating = false;
  }
}