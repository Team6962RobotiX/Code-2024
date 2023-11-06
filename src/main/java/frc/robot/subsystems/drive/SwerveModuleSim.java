// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.NEO;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.DRIVE_MOTOR_CONFIG;
import frc.robot.Constants.SWERVE_DRIVE.STEER_MOTOR_CONFIG;
import frc.robot.Constants.SWERVE_MATH;

public class SwerveModuleSim extends SwerveModule {
  private FlywheelSim driveMotor = new FlywheelSim(
    LinearSystemId.identifyVelocitySystem(DRIVE_MOTOR_CONFIG.kV * 12.0 * (SWERVE_DRIVE.WHEEL_DIAMETER / 2.0), DRIVE_MOTOR_CONFIG.kA * 12.0 * (SWERVE_DRIVE.WHEEL_DIAMETER / 2.0)),
    DCMotor.getNEO(1),
    1.0 / SWERVE_DRIVE.DRIVE_MOTOR_GEAR_RATIO
  );
  
  private FlywheelSim steerMotor = new FlywheelSim(
    LinearSystemId.identifyVelocitySystem(STEER_MOTOR_CONFIG.kV * 12.0, STEER_MOTOR_CONFIG.kA * 12.0),
    DCMotor.getNEO(1),
    1.0 / SWERVE_DRIVE.STEER_MOTOR_GEAR_RATIO
  );

  private PIDController drivePID = new PIDController(
    DRIVE_MOTOR_CONFIG.kP,
    DRIVE_MOTOR_CONFIG.kI,
    DRIVE_MOTOR_CONFIG.kD
  );
  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(
    DRIVE_MOTOR_CONFIG.kS,
    DRIVE_MOTOR_CONFIG.kV
  );
  private PIDController steerPID = new PIDController(
    STEER_MOTOR_CONFIG.kP, 
    STEER_MOTOR_CONFIG.kI,
    STEER_MOTOR_CONFIG.kD
  );

  private SlewRateLimiter steerRampRateLimiter = new SlewRateLimiter(12.0 / STEER_MOTOR_CONFIG.RAMP_RATE);
  private SlewRateLimiter driveRampRateLimiter = new SlewRateLimiter(12.0 / DRIVE_MOTOR_CONFIG.RAMP_RATE);

  private double drivePosition = 0.0;
  private double steerRadians = (Math.random() * 2.0 * Math.PI) - Math.PI;
  
  SwerveModuleState drivenState = new SwerveModuleState();

  public SwerveModuleSim(int id, SwerveDrive swerveDrive) {
    super(id, swerveDrive);
    steerPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setWheelSpeed(double speedMetersPerSecond) {
    double speedMultiple = 1.0;
    if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) {
      speedMultiple = Math.cos(SWERVE_MATH.angleDistance(drivenState.angle.getRadians(), getMeasuredState().angle.getRadians()));
    }
    speedMetersPerSecond *= speedMultiple;

    double acceleration = (super.getTargetState().speedMetersPerSecond - drivenState.speedMetersPerSecond) / 0.02;
    if (Math.abs(acceleration) > SWERVE_DRIVE.ACCELERATION) {
      acceleration = SWERVE_DRIVE.ACCELERATION * Math.signum(acceleration);
    }

    double volts = MathUtil.clamp(
      12.0 * (driveFF.calculate(speedMetersPerSecond) + drivePID.calculate(getMeasuredState().speedMetersPerSecond, speedMetersPerSecond) + acceleration * DRIVE_MOTOR_CONFIG.kA), 
      -12.0, 12.0
    );
    
    double availableVoltage = RobotController.getBatteryVoltage();
    volts = MathUtil.clamp(volts, -availableVoltage, availableVoltage);
    double currentLimit = DRIVE_MOTOR_CONFIG.CURRENT_LIMIT;
    if (Math.abs(driveMotor.getAngularVelocityRadPerSec()) < 0.001) currentLimit = Math.min(NEO.SAFE_STALL_CURRENT, currentLimit);
    // if (driveMotor.getCurrentDrawAmps() > currentLimit) volts = volts / driveMotor.getCurrentDrawAmps() * currentLimit;
    volts = driveRampRateLimiter.calculate(volts);
    driveMotor.setInputVoltage(volts);

    driveMotor.update(0.02);
    drivePosition += getMeasuredState().speedMetersPerSecond * 0.02;
  }

  public void setWheelAngle(double radians) {
    double volts = MathUtil.clamp(
      12.0 * (steerPID.calculate(getMeasuredState().angle.getRadians(), radians)), 
      -12.0, 12.0
    );
    double availableVoltage = RobotController.getBatteryVoltage();
    volts = MathUtil.clamp(volts, -availableVoltage, availableVoltage);
    double currentLimit = STEER_MOTOR_CONFIG.CURRENT_LIMIT;
    if (Math.abs(steerMotor.getAngularVelocityRadPerSec()) < 0.001) currentLimit = Math.min(NEO.SAFE_STALL_CURRENT, currentLimit);
    if (steerMotor.getCurrentDrawAmps() > currentLimit) volts = volts / steerMotor.getCurrentDrawAmps() * currentLimit;
    volts = steerRampRateLimiter.calculate(volts);
    steerMotor.setInputVoltage(volts);

    steerMotor.update(0.02);
    steerRadians += steerMotor.getAngularVelocityRadPerSec() * 0.02;
  }
  
  public double getTotalCurrent() {
    return driveMotor.getCurrentDrawAmps() + steerMotor.getCurrentDrawAmps();
  }

  public void periodic() {
    drivenState = getAccelerationLimitedState();

    setWheelSpeed(drivenState.speedMetersPerSecond);
    setWheelAngle(drivenState.angle.getRadians());
  }

  @Override
  public void stop() {
    super.setTargetState(new SwerveModuleState(0.0, getMeasuredState().angle));
    steerMotor.setInputVoltage(0.0);
    driveMotor.setInputVoltage(0.0);
  }

  @Override
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(driveMotor.getAngularVelocityRadPerSec() * (SWERVE_DRIVE.WHEEL_DIAMETER / 2.0), Rotation2d.fromRadians(steerRadians));
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(drivePosition, getMeasuredState().angle);
  }

  @Override
  public void setVolts(double volts) {
    steerRadians = 0.0;
    driveMotor.setInputVoltage(volts);
    driveMotor.update(0.02);
    drivePosition += getMeasuredState().speedMetersPerSecond * 0.02;
  }

  public static double wheelMOI(double radius, double mass) {
    return (1.0 / 2.0) * mass * Math.pow(radius, 2.0);
  }

  public static double steerWheelMOI(double radius, double mass, double width) {
    return (1.0 / 4.0) * mass * Math.pow(radius, 2.0) + (1.0 / 12.0) * mass * Math.pow(width, 2.0);
  }
}