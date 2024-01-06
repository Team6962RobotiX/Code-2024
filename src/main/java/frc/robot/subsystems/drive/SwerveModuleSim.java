// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.DRIVE_MOTOR_PROFILE;
import frc.robot.Constants.SWERVE_DRIVE.STEER_MOTOR_PROFILE;
import frc.robot.Constants.SWERVE_MATH;
import frc.robot.util.Logging.Logger;

public class SwerveModuleSim extends SwerveModule {
  private FlywheelSim driveMotor = new FlywheelSim(
    LinearSystemId.identifyVelocitySystem(DRIVE_MOTOR_PROFILE.kV * (SWERVE_DRIVE.WHEEL_DIAMETER / 2.0), DRIVE_MOTOR_PROFILE.kA * (SWERVE_DRIVE.WHEEL_DIAMETER / 2.0)),
    DCMotor.getNEO(1),
    1.0 / SWERVE_DRIVE.DRIVE_MOTOR_GEAR_RATIO
  );
  
  private FlywheelSim steerMotor = new FlywheelSim(
    LinearSystemId.identifyVelocitySystem(STEER_MOTOR_PROFILE.kV, STEER_MOTOR_PROFILE.kA),
    DCMotor.getNEO(1),
    1.0 / SWERVE_DRIVE.STEER_MOTOR_GEAR_RATIO
  );

  private PIDController drivePID = new PIDController(
    DRIVE_MOTOR_PROFILE.kP,
    DRIVE_MOTOR_PROFILE.kI,
    DRIVE_MOTOR_PROFILE.kD
  );
  private PIDController steerPID = new PIDController(
    STEER_MOTOR_PROFILE.kP, 
    STEER_MOTOR_PROFILE.kI,
    STEER_MOTOR_PROFILE.kD
  );
  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(
    DRIVE_MOTOR_PROFILE.kS,
    DRIVE_MOTOR_PROFILE.kV,
    DRIVE_MOTOR_PROFILE.kA
  );
      
  private double driveVoltRamp = 0.0;
  private double steerVoltRamp = 0.0;
  private double drivePosition = 0.0;
  private double steerRadians = (Math.random() * 2.0 * Math.PI) - Math.PI;
  
  public SwerveModuleSim(int id) {
    super(id);
    steerPID.enableContinuousInput(-Math.PI, Math.PI);
    System.out.println(DRIVE_MOTOR_PROFILE.RAMP_RATE);

    String logPath = "module_" + SWERVE_DRIVE.MODULE_NAMES[id] + "/";
    Logger.autoLog(logPath + "current",                 () -> getTotalCurrent());
    Logger.autoLog(logPath + "getAbsoluteSteerDegrees", () -> getMeasuredState().angle.getDegrees());
    Logger.autoLog(logPath + "measuredState",           () -> getMeasuredState());
    Logger.autoLog(logPath + "measuredAngle",           () -> getMeasuredState().angle.getDegrees());
    Logger.autoLog(logPath + "measuredVelocity",        () -> getMeasuredState().speedMetersPerSecond);
    Logger.autoLog(logPath + "drivenState",             () -> getDrivenState());
    Logger.autoLog(logPath + "drivenAngle",             () -> getDrivenState().angle.getDegrees());
    Logger.autoLog(logPath + "drivenVelocity",          () -> getDrivenState().speedMetersPerSecond);
    Logger.autoLog(logPath + "targetState",             () -> getTargetState());
    Logger.autoLog(logPath + "targetAngle",             () -> getTargetState().angle.getDegrees());
    Logger.autoLog(logPath + "targetVelocity",          () -> getTargetState().speedMetersPerSecond);
  }

  @Override
  public void drive(SwerveModuleState state) {
    double speedMetersPerSecond = state.speedMetersPerSecond;
    double radians = state.angle.getRadians();
    
    double speedMultiple = 1.0;
    if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) {
      speedMultiple = Math.cos(SWERVE_MATH.angleDistance(getDrivenState().angle.getRadians(), getMeasuredState().angle.getRadians()));
    }
    speedMetersPerSecond *= speedMultiple;

    for (int i = 0; i < 20; i++) {
      double availableVoltage = RobotController.getBatteryVoltage();

      double driveVolts = MathUtil.clamp(
        driveFF.calculate(speedMetersPerSecond, getWheelAcceleration()) + 12.0 * drivePID.calculate(getMeasuredState().speedMetersPerSecond, speedMetersPerSecond), 
        -availableVoltage, availableVoltage
      );
      
      double steerVolts = MathUtil.clamp(
        12.0 * steerPID.calculate(getMeasuredState().angle.getRadians(), radians), 
        -availableVoltage, availableVoltage
      );
      
      driveVoltRamp += (MathUtil.clamp(driveVolts - driveVoltRamp, -12.0 / DRIVE_MOTOR_PROFILE.RAMP_RATE / 1000.0, 12.0 / DRIVE_MOTOR_PROFILE.RAMP_RATE / 1000.0));
      driveVolts = driveVoltRamp;

      steerVoltRamp += (MathUtil.clamp(steerVolts - steerVoltRamp, -12.0 / STEER_MOTOR_PROFILE.RAMP_RATE / 1000.0, 12.0 / STEER_MOTOR_PROFILE.RAMP_RATE / 1000.0));
      steerVolts = steerVoltRamp;

      driveMotor.setInputVoltage(driveVolts);
      steerMotor.setInputVoltage(steerVolts);

      driveMotor.update(1.0 / 1000.0);
      steerMotor.update(1.0 / 1000.0);

      drivePosition += getMeasuredState().speedMetersPerSecond * (1.0 / 1000.0);
      steerRadians += steerMotor.getAngularVelocityRadPerSec() * (1.0 / 1000.0);
    }
  }

  @Override
  public void seedSteerEncoder() {
    return;
  }
  
  @Override
  public double getTotalCurrent() {
    return driveMotor.getCurrentDrawAmps() + steerMotor.getCurrentDrawAmps();
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