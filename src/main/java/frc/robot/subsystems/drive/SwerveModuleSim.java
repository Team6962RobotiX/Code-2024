// // // Copyright (c) FIRST and other WPILib contributors.
// // // Open Source Software; you can modify and/or share it under the terms of
// // // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.drive;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import edu.wpi.first.wpilibj.simulation.RoboRioSim;
// import frc.robot.Constants.NEO;
// import frc.robot.Constants.SWERVE_DRIVE;
// import frc.robot.Constants.SWERVE_MATH;
// import frc.robot.Constants.SWERVE_DRIVE.DRIVE_MOTOR_CONFIG;
// import frc.robot.Constants.SWERVE_DRIVE.STEER_MOTOR_CONFIG;

// public class SwerveModuleSim extends SwerveModule {
//   private FlywheelSim driveMotor = new FlywheelSim(
//     DCMotor.getNEO(1),
//     1.0 / SWERVE_DRIVE.DRIVE_MOTOR_GEAR_RATIO,
//     ((1.0 / 2.0) * SWERVE_DRIVE.WHEEL_MASS * Math.pow(SWERVE_DRIVE.WHEEL_DIAMETER / 2.0, 2.0)) / Math.pow(SWERVE_DRIVE.DRIVE_MOTOR_GEAR_RATIO, 2.0)
//   );
//   private FlywheelSim steerMotor = new FlywheelSim(
//     DCMotor.getNEO(1),
//     1.0 / SWERVE_DRIVE.STEER_MOTOR_GEAR_RATIO,
//     ((1.0 / 4.0) * SWERVE_DRIVE.WHEEL_MASS * Math.pow(SWERVE_DRIVE.WHEEL_DIAMETER / 2.0, 2.0) + (1.0 / 12.0) * SWERVE_DRIVE.WHEEL_MASS * Math.pow(SWERVE_DRIVE.WHEEL_WIDTH, 2.0)) / Math.pow(SWERVE_DRIVE.STEER_MOTOR_GEAR_RATIO, 2.0)
//   );
//   private PIDController driveController = new PIDController(
//     DRIVE_MOTOR_CONFIG.kP, 
//     DRIVE_MOTOR_CONFIG.kI,
//     DRIVE_MOTOR_CONFIG.kD
//   );
//   private PIDController steerController = new PIDController(
//     STEER_MOTOR_CONFIG.kP, 
//     STEER_MOTOR_CONFIG.kI,
//     STEER_MOTOR_CONFIG.kD
//   );
//   private Constraints steerConstraints = new Constraints(steerRPMToRadPerSec(STEER_MOTOR_CONFIG.maxRPM), steerRPMToRadPerSec(STEER_MOTOR_CONFIG.maxAccel));
//   private SlewRateLimiter driveAccelLimiter = new SlewRateLimiter(driveRPMToMetersPerSec(DRIVE_MOTOR_CONFIG.maxAccel));
//   private SwerveModuleState state = new SwerveModuleState();
//   private String name;
//   public int id;

//   private double steerRadians = (Math.random() * 2.0 * Math.PI) - Math.PI;
//   private double drivePosition = 0;
//   private double lastUpdate = Timer.getFPGATimestamp();
//   private double lastDrive = Timer.getFPGATimestamp();
//   private double tempV = 0.0;
  
//   public SwerveModuleSim(int id) {
//     super(id);
//     this.id = id;
//     name = SWERVE_DRIVE.MODULE_NAMES[id];
//     steerController.enableContinuousInput(-Math.PI, Math.PI);
//   }

//   /**
//    * Drive the module with a state.
//    * @param state The state (velocity and direction) we want to drive the module
//    */
//   @Override
//   public void drive(SwerveModuleState state) {
//     double timeDelta = Timer.getFPGATimestamp() - lastDrive;
//     lastDrive += timeDelta;

//     state = SwerveModuleState.optimize(state, getSteerRotation2d());
//     this.state = state;
//     // System.out.println(targetAngle);
//   }

//   public double getClosestSteerPosition(double targetSteerRadians) {
//     double baseAngle = steerRadians - getSteerRadians();
//     double angle1 = baseAngle + targetSteerRadians;
//     double angle2 = baseAngle + (targetSteerRadians - Math.PI * 2.0);
//     double angle3 = baseAngle + (targetSteerRadians + Math.PI * 2.0);
//     double angle1Dist = Math.abs(angle1 - steerRadians);
//     double angle2Dist = Math.abs(angle2 - steerRadians);
//     double angle3Dist = Math.abs(angle3 - steerRadians);
//     if (angle1Dist == Math.min(Math.min(angle1Dist, angle2Dist), angle3Dist)) {
//       return angle1;
//     }
//     if (angle2Dist == Math.min(Math.min(angle1Dist, angle2Dist), angle3Dist)) {
//       return angle2;
//     }
//     if (angle3Dist == Math.min(Math.min(angle1Dist, angle2Dist), angle3Dist)) {
//       return angle3;
//     }
//     return angle1;
//   }

//   public void driveMotorVolts(double volts) {
//     double availableVoltage = RoboRioSim.getVInVoltage();
//     volts = MathUtil.clamp(volts, -availableVoltage, availableVoltage);
//     // volts = driveMotorRampRateLimiter.calculate(volts);
//     driveMotor.setInputVoltage(volts);
//     double currentLimit = DRIVE_MOTOR_CONFIG.currentLimit;
//     if (Math.abs(driveMotor.getAngularVelocityRadPerSec()) < 0.001) currentLimit = NEO.SAFE_STALL_CURRENT;
//     if (driveMotor.getCurrentDrawAmps() > currentLimit) driveMotor.setInputVoltage(volts / driveMotor.getCurrentDrawAmps() * currentLimit);
//     feed();
//   }

//   public void steerMotorVolts(double volts) {
//     double availableVoltage = RoboRioSim.getVInVoltage();
//     volts = MathUtil.clamp(volts, -availableVoltage, availableVoltage);
//     // volts = steerMotorRampRateLimiter.calculate(volts);
//     steerMotor.setInputVoltage(volts);
//     double currentLimit = STEER_MOTOR_CONFIG.currentLimit;
//     if (Math.abs(steerMotor.getAngularVelocityRadPerSec()) < 0.001) currentLimit = NEO.SAFE_STALL_CURRENT;
//     if (steerMotor.getCurrentDrawAmps() > currentLimit) steerMotor.setInputVoltage(volts / steerMotor.getCurrentDrawAmps() * currentLimit);
//   }

//   @Override
//   public void update() {
//     double timeDelta = 1.0 / 1000.0;

//     while (lastUpdate < Timer.getFPGATimestamp()) {
//       lastUpdate += timeDelta;
//       double targetSpeed = state.speedMetersPerSecond;
//       if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) targetSpeed *= Math.cos(SWERVE_MATH.angleDistance(state.angle.getRadians(), getSteerRadians()));
      
//       double targetAngle = getClosestSteerPosition(state.angle.getRadians());
//       TrapezoidProfile steerProfile = new TrapezoidProfile(steerConstraints, new State(targetAngle, 0), new State(steerRadians, steerMotor.getAngularVelocityRadPerSec()));
//       State targetSteerState = steerProfile.calculate(timeDelta);

//       double steerVolts = MathUtil.clamp(
//         12.0 * (STEER_MOTOR_CONFIG.kFF * steerRadPerSecToRPM(targetSteerState.velocity)), 
//         -12.0, 12.0
//       );
      
//       State targetDriveState = new State(0.0, driveAccelLimiter.calculate(targetSpeed));

//       double driveVolts = MathUtil.clamp(
//         12.0 * (DRIVE_MOTOR_CONFIG.kFF * driveMetersPerSecToRPM(targetDriveState.velocity)), 
//         -12.0, 12.0
//       );

//       driveMotorVolts(driveVolts);
//       steerMotorVolts(steerVolts);

//       driveMotor.update(timeDelta);
//       steerMotor.update(timeDelta);
    
//       steerRadians += steerMotor.getAngularVelocityRadPerSec() * timeDelta;
//       drivePosition += getVelocity() * timeDelta;
//     }
//   }

//   public double driveRPMToMetersPerSec(double RPM) {
//     return RPM * SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION / 60.0;
//   }

//   public double driveMetersPerSecToRPM(double metersPerSec) {
//     return metersPerSec / SWERVE_DRIVE.DRIVE_MOTOR_METERS_PER_REVOLUTION * 60.0;
//   }

//   public double steerRPMToRadPerSec(double RPM) {
//     return RPM * SWERVE_DRIVE.STEER_MOTOR_RADIANS_PER_REVOLUTION / 60.0;
//   }

//   public double steerRadPerSecToRPM(double RPS) {
//     return RPS / SWERVE_DRIVE.STEER_MOTOR_RADIANS_PER_REVOLUTION * 60.0;
//   }
  
//   /**
//    * @return Steering direction in radians (-PI - PI)
//    */
//   @Override
//   public double getSteerRadians() {
//     return SWERVE_MATH.clampRadians(steerRadians);
//   }

//   /**
//    * @return Steering direction from absolute encoder (CANCoder) in radians (-PI - PI)
//    */
//   @Override
//   public double getAbsoluteSteerRadians() {
//     return getSteerRadians();
//   }

//   public double getDriveMotorPosition() {
//     return drivePosition;
//   }

//   /**
//    * @return Steering direction in degrees (-180 - 180)
//    */
//   @Override
//   public Rotation2d getSteerRotation2d() {
//     return Rotation2d.fromRadians(getSteerRadians());
//   }

//   /**
//    * @return Drive wheel velocity in m/s
//    */
//   @Override
//   public double getVelocity() {
//     return driveMotor.getAngularVelocityRadPerSec() * SWERVE_DRIVE.WHEEL_DIAMETER / 2.0;
//   }

//   public double getVelocityRPM() {
//     return getVelocity();
//   }

//   /**
//    * @return Name of the module (set in constants file)
//    */
//   @Override
//   public String getName() {
//     return name;
//   }

//   /**
//    * @return The total current of both motors (measured in Amps)
//    */
//   @Override
//   public double getCurrent() {
//     return driveMotor.getCurrentDrawAmps() + steerMotor.getCurrentDrawAmps();
//   }

//   /**
//    * Stops power to both motors (not persistent)
//    */
//   @Override
//   public void stop() {
//     double timeDelta = Timer.getFPGATimestamp() - lastUpdate;
//     lastUpdate += timeDelta;
//     steerRadians += steerMotor.getAngularVelocityRadPerSec() * timeDelta;
//     drivePosition += getVelocity() * timeDelta;
//     steerRadians = SWERVE_MATH.clampRadians(steerRadians);
    
//     steerMotor.setInputVoltage(0.0);
//     driveMotor.setInputVoltage(0.0);

//     driveMotor.update(timeDelta);
//     steerMotor.update(timeDelta);
//   }

//   /**
//    * For WPILib MotorSafety Class
//    */
//   @Override
//   public void stopMotor() {
//     stop();
//   }

//   @Override
//   public String getDescription() {
//     return getName() + " Swerve Module";
//   }
  
//   /**
//    * @return The target SwerveModuleState
//    */
//   @Override
//   public SwerveModuleState getTargetState() {
//     return state;
//   }

//   /**
//    * @return The measured SwerveModuleState
//    */
//   @Override
//   public SwerveModuleState getMeasuredState() {
//     return new SwerveModuleState(getVelocity(), getSteerRotation2d());
//   }

//   /**
//    * @return The measured SwerveModulePosition
//    */
//   @Override
//   public SwerveModulePosition getModulePosition() {
//     return new SwerveModulePosition(getDriveMotorPosition(), getSteerRotation2d());
//   }

//   @Override
//   public void runCharacterization(double volts) {
//     steerRadians = 0.0;
//     driveMotorVolts(volts);
//   }

//   @Override
//   public double getTargetVelocity() {
//     return state.speedMetersPerSecond;
//   }

//   @Override
//   public double getTargetAngle() {
//     return state.angle.getRadians();
//   }
// }