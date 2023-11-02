// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.NEO;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.DRIVE_MOTOR_CONFIG;
import frc.robot.Constants.SWERVE_DRIVE.STEER_MOTOR_CONFIG;
import frc.robot.Constants.SWERVE_MATH;

public class SwerveModuleSim extends SwerveModule {
  // private double steerMOI = ((1.0 / 4.0) * SWERVE_DRIVE.WHEEL_MASS * Math.pow(SWERVE_DRIVE.WHEEL_DIAMETER / 2.0, 2.0) + (1.0 / 12.0) * SWERVE_DRIVE.WHEEL_MASS * Math.pow(SWERVE_DRIVE.WHEEL_WIDTH, 2.0));
  // private double driveMOI = ((1.0 / 2.0) * SWERVE_DRIVE.WHEEL_MASS * Math.pow(SWERVE_DRIVE.WHEEL_DIAMETER / 2.0, 2.0));
  private double steerMOI = 0.01;
  private double driveMOI = 0.01;
  private FlywheelSim driveMotor = new FlywheelSim(
    DCMotor.getNEO(1),
    1.0 / SWERVE_DRIVE.DRIVE_MOTOR_GEAR_RATIO,
    driveMOI
  );
  
  private FlywheelSim steerMotor = new FlywheelSim(
    DCMotor.getNEO(1),
    1.0 / SWERVE_DRIVE.STEER_MOTOR_GEAR_RATIO,
    steerMOI
  );

  private PIDController drivePID = new PIDController(
    DRIVE_MOTOR_CONFIG.kP, 
    DRIVE_MOTOR_CONFIG.kI,
    DRIVE_MOTOR_CONFIG.kD
  );
  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(
    0.0,
    DRIVE_MOTOR_CONFIG.kFF
  );
  private PIDController steerPID = new PIDController(
    STEER_MOTOR_CONFIG.kP, 
    STEER_MOTOR_CONFIG.kI,
    STEER_MOTOR_CONFIG.kD
  );
  private SlewRateLimiter steerRampRateLimiter = new SlewRateLimiter(12.0 / STEER_MOTOR_CONFIG.RAMP_RATE);
  private SlewRateLimiter driveRampRateLimiter = new SlewRateLimiter(12.0 / DRIVE_MOTOR_CONFIG.RAMP_RATE);
  private SwerveModuleState state = new SwerveModuleState();
  private SwerveModuleState trueState = new SwerveModuleState();
  private SwerveModulePosition truePosition = new SwerveModulePosition();

  public int id;
  private SwerveDrive swerveDrive;
  
  private double steerRadians = (Math.random() * 2.0 * Math.PI) - Math.PI;
  private Pose2d previousPose = new Pose2d();
  private Translation2d previousVelocity = new Translation2d();
  private double previousWheelVelocity = 0.0;
  private double drivePosition = 0.0;
  private double lastUpdate = 0.0;
  
  public SwerveModuleSim(int id, SwerveDrive swerveDrive) {
    super(id);
    this.id = id;
    this.swerveDrive = swerveDrive;
    steerPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void drive(SwerveModuleState state) {
    double timeDelta = Timer.getFPGATimestamp() - lastUpdate;
    lastUpdate += timeDelta;
    state = SwerveModuleState.optimize(state, getMeasuredState().angle);
    if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) state.speedMetersPerSecond *= Math.cos(SWERVE_MATH.angleDistance(state.angle.getRadians(), getMeasuredState().angle.getRadians()));
    this.state = state;
  }

  public void driveMotorVolts(double volts) {
    double availableVoltage = RoboRioSim.getVInVoltage();
    volts = MathUtil.clamp(volts, -availableVoltage, availableVoltage);
    double currentLimit = DRIVE_MOTOR_CONFIG.CURRENT_LIMIT;
    if (Math.abs(driveMotor.getAngularVelocityRadPerSec()) < 0.001) currentLimit = Math.min(NEO.SAFE_STALL_CURRENT, currentLimit);
    // if (driveMotor.getCurrentDrawAmps() > currentLimit) volts = volts / driveMotor.getCurrentDrawAmps() * currentLimit;
    volts = driveRampRateLimiter.calculate(volts);
    driveMotor.setInputVoltage(volts);
  }

  public void steerMotorVolts(double volts) {
    double availableVoltage = RoboRioSim.getVInVoltage();
    volts = MathUtil.clamp(volts, -availableVoltage, availableVoltage);
    double currentLimit = STEER_MOTOR_CONFIG.CURRENT_LIMIT;
    if (Math.abs(steerMotor.getAngularVelocityRadPerSec()) < 0.001) currentLimit = Math.min(NEO.SAFE_STALL_CURRENT, currentLimit);
    if (steerMotor.getCurrentDrawAmps() > currentLimit) volts = volts / steerMotor.getCurrentDrawAmps() * currentLimit;
    volts = steerRampRateLimiter.calculate(volts);
    steerMotor.setInputVoltage(volts);
  }

  @Override
  public void periodic() {
    if (lastUpdate == 0) {
      previousPose = getPose(swerveDrive.getPose());
      lastUpdate = Timer.getFPGATimestamp();
    }

    double timeDelta = 1.0 / 1000.0;
    
    while (lastUpdate < Timer.getFPGATimestamp()) {
      lastUpdate += timeDelta;

      double driveVolts = MathUtil.clamp(
        12.0 * (driveFF.calculate(state.speedMetersPerSecond) + drivePID.calculate(getMeasuredState().speedMetersPerSecond, state.speedMetersPerSecond)), 
        -12.0, 12.0
      );
      
      double steerVolts = MathUtil.clamp(
        12.0 * (steerPID.calculate(getMeasuredState().angle.getRadians(), state.angle.getRadians())), 
        -12.0, 12.0
      );

      driveMotorVolts(driveVolts);
      steerMotorVolts(steerVolts);

      driveMotor.update(timeDelta);
      steerMotor.update(timeDelta);
      
      steerRadians += steerMotor.getAngularVelocityRadPerSec() * timeDelta;
      
      // System.out.println(((Math.min(force, frictionForce) * Math.signum(acceleration)) / SWERVE_DRIVE.ROBOT_MASS) * timeDelta * timeDelta);
            
      // System.out.println(getMeasuredState().speedMetersPerSecond);

      // Translation2d velocity = swerveDrive.getPose().minus(previousPose).getTranslation().div(timeDelta);
      // Translation2d acceleration = velocity.minus(previousVelocity).div(timeDelta);
      // double wheelAccel = (getMeasuredState().speedMetersPerSecond - previousWheelVelocity) / timeDelta;

      // trueState = slipCalculation(velocity, acceleration, wheelAccel, timeDelta);
      // truePosition.distanceMeters += trueState.speedMetersPerSecond * timeDelta;
      // truePosition.angle = trueState.angle;

      // previousVelocity = velocity;
      // previousPose = getPose(swerveDrive.getPose());
      // previousWheelVelocity = getMeasuredState().speedMetersPerSecond;
    }
  }

  public SwerveModuleState slipCalculation(Translation2d velocity, Translation2d acceleration, double wheelAccel, double timeDelta) {
    
    // Translation2d wheelVelocity = new Translation2d(getMeasuredState().speedMetersPerSecond, getMeasuredState().angle);
    // Translation2d totalVelocity = velocity.plus(wheelVelocity);
    
    Translation2d wheelAcceleration = new Translation2d(wheelAccel, getMeasuredState().angle);
    Translation2d totalAcceleration = acceleration.plus(wheelAcceleration);
    
    double force = totalAcceleration.getNorm() * SWERVE_DRIVE.ROBOT_MASS;
    double frictionForce = 9.80 * SWERVE_DRIVE.STATIC_FRICTION * SWERVE_DRIVE.ROBOT_MASS;
    
    if (force > frictionForce) {
      force = 9.80 * SWERVE_DRIVE.KINETIC_FRICTION * SWERVE_DRIVE.ROBOT_MASS;
    }

    velocity = velocity.plus(totalAcceleration.div(totalAcceleration.getNorm()).times(force / SWERVE_DRIVE.ROBOT_MASS).times(timeDelta));
    return new SwerveModuleState(velocity.getNorm(), velocity.getAngle());
  }
  
  public double getTotalCurrent() {
    return driveMotor.getCurrentDrawAmps() + steerMotor.getCurrentDrawAmps();
  }

  @Override
  public void stop() {    
    steerMotor.setInputVoltage(0.0);
    driveMotor.setInputVoltage(0.0);
  }
  
  @Override
  public SwerveModuleState getTargetState() {
    return state;
  }

  @Override
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(driveMotor.getAngularVelocityRadPerSec() * (SWERVE_DRIVE.WHEEL_DIAMETER / 2.0), Rotation2d.fromRadians(steerRadians));
  }

  public SwerveModuleState getTrueState() {
    return trueState;
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(drivePosition, getMeasuredState().angle);
  }

  @Override
  public void runCharacterization(double volts) {
    steerRadians = 0.0;
    driveMotorVolts(volts);
  }
}