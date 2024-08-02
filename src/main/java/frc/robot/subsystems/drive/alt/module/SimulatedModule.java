// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.alt.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.alt.SwerveConfig;
import frc.robot.util.software.Logging.Logger;

public class SimulatedModule extends SubsystemBase implements SwerveModule {
  private FlywheelSim driveMotor, steerMotor;
  private PIDController drivePID, steerPID;
  private SimpleMotorFeedforward driveFeedforward;
      
  private double driveVoltRamp = 0.0;
  private double steerVoltRamp = 0.0;
  private double drivePosition = 0.0;
  private double steerRadians = (Math.random() * 2.0 * Math.PI) - Math.PI;

  private SwerveConfig configuration;

  private SwerveModuleState targetState;
  
  public SimulatedModule(SwerveConfig config, int corner) {
    configuration = config;

    steerPID.enableContinuousInput(-Math.PI, Math.PI);

    SwerveConfig.MotorProfile driveMotorProfile = config.driveMotorProfile();
    SwerveConfig.MotorProfile steerMotorProfile = config.steerMotorProfile();

    driveMotor = new FlywheelSim(
      LinearSystemId.identifyVelocitySystem(
        config.wheelRadius().times(driveMotorProfile.kV()).in(Meters),
        config.wheelRadius().times(driveMotorProfile.kA()).in(Meters)
      ),
      config.swerveMotorInfo().stats(),
      config.driveMotorGearing()
    );

    steerMotor = new FlywheelSim(
      LinearSystemId.identifyVelocitySystem(
        steerMotorProfile.kV(),
        steerMotorProfile.kA()
      ),
      config.swerveMotorInfo().stats(),
      config.steerMotorGearing()
    );

    drivePID = new PIDController(
      driveMotorProfile.kP(),
      driveMotorProfile.kI(),
      driveMotorProfile.kD()
    );

    steerPID = new PIDController(
      steerMotorProfile.kP(), 
      steerMotorProfile.kI(),
      steerMotorProfile.kD()
    );

    driveFeedforward = new SimpleMotorFeedforward(
      driveMotorProfile.kS(),
      driveMotorProfile.kV(),
      driveMotorProfile.kA()
    );

    String logPath = SwerveModule.getLogPath(corner);
    Logger.autoLog(this, logPath + "current",                 () -> getTotalCurrent());
    Logger.autoLog(this, logPath + "getAbsoluteSteerDegrees", () -> getMeasuredState().angle.getDegrees());
    Logger.autoLog(this, logPath + "measuredState",           () -> getMeasuredState());
    Logger.autoLog(this, logPath + "measuredAngle",           () -> getMeasuredState().angle.getDegrees());
    Logger.autoLog(this, logPath + "measuredVelocity",        () -> getMeasuredState().speedMetersPerSecond);
    Logger.autoLog(this, logPath + "targetState",             () -> getTargetState());
    Logger.autoLog(this, logPath + "targetAngle",             () -> getTargetState().angle.getDegrees());
    Logger.autoLog(this, logPath + "targetVelocity",          () -> getTargetState().speedMetersPerSecond);
  }

  @Override
  public void periodic() {
    if (targetState != null) {
      driveState(targetState);
    }
  }

  public void drive(SwerveModuleState state) {
    targetState = state;
  }
  
  private void driveState(SwerveModuleState state) {
    double speedMetersPerSecond = state.speedMetersPerSecond;
    double radians = state.angle.getRadians();
    
    for (int i = 0; i < 20; i++) {
      double driveVolts = driveFeedforward.calculate(speedMetersPerSecond, 0.0) + 12.0 * drivePID.calculate(getMeasuredState().speedMetersPerSecond, speedMetersPerSecond);
      double steerVolts = 12.0 * steerPID.calculate(getMeasuredState().angle.getRadians(), radians);

      double maxDriveRampRate = configuration.maxLinearWheelSpeed().in(MetersPerSecond) / configuration.maxRobotAcceleration().in(MetersPerSecondPerSecond);
      
      driveVoltRamp += (MathUtil.clamp(driveVolts - driveVoltRamp, -12.0 / maxDriveRampRate / 1000.0, 12.0 / maxDriveRampRate / 1000.0));
      driveVolts = driveVoltRamp;

      double maxSteerRampRate = configuration.swerveMotorInfo().maxRampRate().in(Amps.per(Second));
      
      steerVoltRamp += (MathUtil.clamp(steerVolts - steerVoltRamp, -12.0 / maxSteerRampRate / 1000.0, 12.0 / maxSteerRampRate / 1000.0));
      steerVolts = steerVoltRamp;

      driveMotor.setInputVoltage(MathUtil.clamp(driveVolts, -12.0, 12.0));
      steerMotor.setInputVoltage(MathUtil.clamp(steerVolts, -12.0, 12.0));

      driveMotor.update(1.0 / 1000.0);
      steerMotor.update(1.0 / 1000.0);

      drivePosition += getMeasuredState().speedMetersPerSecond * (1.0 / 1000.0);
      steerRadians += steerMotor.getAngularVelocityRadPerSec() * (1.0 / 1000.0);
      steerRadians = MathUtil.angleModulus(steerRadians);
    }
  }

  @Override
  public Measure<Voltage> getTotalCurrent() {
    return Volts.of(
      driveMotor.getCurrentDrawAmps() + 
      steerMotor.getCurrentDrawAmps()
    );
  }
  
  @Override
  public void stop() {
    drive(new SwerveModuleState(0.0, getMeasuredState().angle));
    steerMotor.setInputVoltage(0.0);
    driveMotor.setInputVoltage(0.0);
  }

  @Override
  public SwerveModuleState getTargetState() {
    return targetState;
  }

  @Override
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(driveMotor.getAngularVelocityRadPerSec() * configuration.wheelRadius().in(Meters), Rotation2d.fromRadians(steerRadians));
  }

  @Override
  public SwerveModulePosition getMeasuredPosition() {
    return new SwerveModulePosition(drivePosition, getMeasuredState().angle);
  }

  @Override
  public Command calibrateDriveMotor() {
      throw new UnsupportedOperationException("Cannot calibrate simulated motor");
  }

  @Override
  public Command calibrateSteerMotor() {
      throw new UnsupportedOperationException("Cannot calibrate simulated motor");
  }
}