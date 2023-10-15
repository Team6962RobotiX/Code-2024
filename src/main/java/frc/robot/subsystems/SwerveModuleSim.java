// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.DRIVE_MOTOR_MOTION_PROFILE;
import frc.robot.Constants.SWERVE_DRIVE.STEER_MOTOR_MOTION_PROFILE;
import frc.robot.Constants.SWERVE_MATH;
import frc.robot.utils.Logger;

public class SwerveModuleSim extends SwerveModule {
  private FlywheelSim driveMotor = new FlywheelSim(
    DCMotor.getNEO(1),
    1.0 / SWERVE_DRIVE.DRIVE_MOTOR_GEAR_RATIO,
    ((1.0 / 2.0) * SWERVE_DRIVE.WHEEL_MASS * Math.pow(SWERVE_DRIVE.WHEEL_DIAMETER / 2.0, 2.0)) / Math.pow(SWERVE_DRIVE.DRIVE_MOTOR_GEAR_RATIO, 2.0)
  );
  private FlywheelSim steerMotor = new FlywheelSim(
    DCMotor.getNEO(1),
    1.0 / SWERVE_DRIVE.STEER_MOTOR_GEAR_RATIO,
    ((1.0 / 4.0) * SWERVE_DRIVE.WHEEL_MASS * Math.pow(SWERVE_DRIVE.WHEEL_DIAMETER / 2.0, 2.0) + (1.0 / 12.0) * SWERVE_DRIVE.WHEEL_MASS * Math.pow(SWERVE_DRIVE.WHEEL_WIDTH, 2.0)) / Math.pow(SWERVE_DRIVE.STEER_MOTOR_GEAR_RATIO, 2.0)
  );
  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(
    0.0,
    DRIVE_MOTOR_MOTION_PROFILE.kFF,
    0.0
  );
  private PIDController driveController = new PIDController(
    DRIVE_MOTOR_MOTION_PROFILE.kP, 
    DRIVE_MOTOR_MOTION_PROFILE.kI,
    DRIVE_MOTOR_MOTION_PROFILE.kD
  );
  private PIDController steerController = new PIDController(
    STEER_MOTOR_MOTION_PROFILE.kP, 
    STEER_MOTOR_MOTION_PROFILE.kI,
    STEER_MOTOR_MOTION_PROFILE.kD
  );
  private SwerveModuleState state = new SwerveModuleState();
  private String name;
  public int id;

  private double steerRadians = (Math.random() * 2.0 * Math.PI) - Math.PI;
  private double drivePosition = (Math.random() * 2.0 * Math.PI) - Math.PI;
  private double lastTimestamp = Timer.getFPGATimestamp();

  private SlewRateLimiter driveMotorRampRateLimiter = new SlewRateLimiter(12.0 / SWERVE_DRIVE.DRIVE_MOTOR_RAMP_RATE);
  private SlewRateLimiter steerMotorRampRateLimiter = new SlewRateLimiter(12.0 / SWERVE_DRIVE.STEER_MOTOR_RAMP_RATE);

  public SwerveModuleSim(int id) {
    super(id, true);
    this.id = id;
    name = SWERVE_DRIVE.MODULE_NAMES[id];
    steerController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Drive the module with a state.
   * @param state The state (velocity and direction) we want to drive the module
   */
  @Override
  public void drive(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getSteerRotation2d());
    if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) state.speedMetersPerSecond *= Math.cos(SWERVE_MATH.angleDistance(state.angle.getRadians(), getSteerRadians()));
    this.state = state;

    double driveVolts = MathUtil.clamp(12.0 * (DRIVE_MOTOR_MOTION_PROFILE.kFF * state.speedMetersPerSecond + driveController.calculate(getVelocity(), state.speedMetersPerSecond)), -12.0, 12.0);
    double steerVolts = MathUtil.clamp(12.0 * (steerController.calculate(getSteerRadians(), state.angle.getRadians())), -12.0, 12.0);
    
    driveVolts = driveMotorRampRateLimiter.calculate(driveVolts);
    steerVolts = steerMotorRampRateLimiter.calculate(steerVolts);
    
    driveMotor.setInputVoltage(driveVolts);
    steerMotor.setInputVoltage(steerVolts);

    if (driveMotor.getCurrentDrawAmps() > SWERVE_DRIVE.DRIVE_MOTOR_CURRENT_LIMIT) {
      driveMotor.setInputVoltage(driveVolts / driveMotor.getCurrentDrawAmps() * SWERVE_DRIVE.DRIVE_MOTOR_CURRENT_LIMIT);
    }

    if (steerMotor.getCurrentDrawAmps() > SWERVE_DRIVE.STEER_MOTOR_CURRENT_LIMIT) {
      steerMotor.setInputVoltage(steerVolts / steerMotor.getCurrentDrawAmps() * SWERVE_DRIVE.STEER_MOTOR_CURRENT_LIMIT);
    }

    feed();
  }

  public void execute() {
    double timeDelta = Timer.getFPGATimestamp() - lastTimestamp;
    lastTimestamp += timeDelta;
    
    driveMotor.update(timeDelta);
    steerMotor.update(timeDelta);
    
    steerRadians += steerMotor.getAngularVelocityRadPerSec() * timeDelta;
    drivePosition += getVelocity() * timeDelta;
    steerRadians = SWERVE_MATH.clampRadians(steerRadians);
  }
  
  /**
   * @return Steering direction in radians (-PI - PI)
   */
  @Override
  public double getSteerRadians() {
    return steerRadians;
  }

  /**
   * @return Steering direction from absolute encoder (CANCoder) in radians (-PI - PI)
   */
  @Override
  public double getAbsoluteSteerRadians() {
    return steerRadians;
  }

  public double getDriveMotorPosition() {
    return drivePosition;
  }

  /**
   * @return Steering direction in degrees (-180 - 180)
   */
  @Override
  public Rotation2d getSteerRotation2d() {
    return Rotation2d.fromRadians(getSteerRadians());
  }

  /**
   * @return Drive wheel velocity in m/s
   */
  @Override
  public double getVelocity() {
    return driveMotor.getAngularVelocityRadPerSec() * SWERVE_DRIVE.WHEEL_DIAMETER / 2.0;
  }

  /**
   * @return Name of the module (set in constants file)
   */
  @Override
  public String getName() {
    return name;
  }

  /**
   * @return The total current of both motors (measured in Amps)
   */
  @Override
  public double getCurrent() {
    return driveMotor.getCurrentDrawAmps() + steerMotor.getCurrentDrawAmps();
  }

  /**
   * Stops power to both motors (not persistent)
   */
  @Override
  public void stop() {
    double timeDelta = Timer.getFPGATimestamp() - lastTimestamp;
    lastTimestamp += timeDelta;
    steerRadians += steerMotor.getAngularVelocityRadPerSec() * timeDelta;
    drivePosition += getVelocity() * timeDelta;
    steerRadians = SWERVE_MATH.clampRadians(steerRadians);
    
    steerMotor.setInputVoltage(0.0);
    driveMotor.setInputVoltage(0.0);

    driveMotor.update(timeDelta);
    steerMotor.update(timeDelta);
  }

  /**
   * For WPILib MotorSafety Class
   */
  @Override
  public void stopMotor() {
    stop();
  }

  @Override
  public String getDescription() {
    return getName() + " Swerve Module";
  }
  
  /**
   * @return The target SwerveModuleState
   */
  @Override
  public SwerveModuleState getTargetState() {
    return state;
  }

  /**
   * @return The measured SwerveModuleState
   */
  @Override
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(getVelocity(), getSteerRotation2d());
  }

  /**
   * @return The measured SwerveModulePosition
   */
  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDriveMotorPosition(), getSteerRotation2d());
  }

  /**
   * Logs all values, motor controllers, and encoders to network tables to be saved later
   */
  @Override
  public void log(String path) {
    Logger.logValue(path + "/steerDegrees", getSteerRotation2d().getDegrees());
    Logger.logValue(path + "/velocity", getVelocity());
    Logger.logValue(path + "/name", name);
  }

  /**
   * @param power Motor power (-1.0 - 1.0)
   * @return Drive velocity
   */
  public static double motorPowerToDriveVelocity(double power) {
    return power / DRIVE_MOTOR_MOTION_PROFILE.kFF;
  }

  @Override
  public double getTargetVelocity() {
    return state.speedMetersPerSecond;
  }

  @Override
  public double getTargetAngle() {
    return state.angle.getRadians();
  }
}