// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.SHOOTER;
import frc.robot.Constants.SHOOTER.SHOOTER_PIVOT;
import frc.robot.Constants.SHOOTER.SHOOTER_WHEELS;
import frc.robot.Constants.SWERVE_DRIVE;

public class ShooterWheels extends SubsystemBase {
  // The plant holds a state-space model of our flywheel. This system has the following properties:
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  private final LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.identifyVelocitySystem(SHOOTER_WHEELS.PROFILE.kV, SHOOTER_WHEELS.PROFILE.kA);
  
  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> observer =
    new KalmanFilter<>(
      Nat.N1(),
      Nat.N1(),
      flywheelPlant,
      VecBuilder.fill(3.0), // How accurate we think our model is
      VecBuilder.fill(0.01), // How accurate we think our encoder
      // data is
      0.020);
  
  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> controller =
    new LinearQuadraticRegulator<>(
      flywheelPlant,
      VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
      // this to more heavily penalize state excursion, or make the controller behave more
      // aggressively.
      VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive. 12 is a good
      // starting point because that is the (approximate) maximum voltage of a battery.
      0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
      // lower if using notifiers.
  
  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> loop = new LinearSystemLoop<>(flywheelPlant, controller, observer, 12.0, 0.020);
  private double targetAngularVelocity = 0.0;
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private boolean isCalibrating = false;
  private SwerveDrive swerveDrive;

  public ShooterWheels(SwerveDrive swerveDrive) {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    motor = new CANSparkMax(CAN.SHOOTER_WHEELS, MotorType.kBrushless);
    encoder = motor.getEncoder();
    encoder.setVelocityConversionFactor(SHOOTER_WHEELS.GEARBOX_STEP_UP * 2.0 * Math.PI / 60.0);
    this.swerveDrive = swerveDrive;

    String logPath = "shooter-wheels/";
    Logger.autoLog(logPath + "current",                 () -> motor.getOutputCurrent());
    Logger.autoLog(logPath + "appliedOutput",           () -> motor.getAppliedOutput());
    Logger.autoLog(logPath + "motorTemperature",        () -> motor.getMotorTemperature());
    Logger.autoLog(logPath + "position",                () -> encoder.getPosition());
    Logger.autoLog(logPath + "velocity",                () -> getVelocity());

    StatusChecks.addCheck("Shooter Wheels Motor", () -> motor.getFaults() == 0);
  }

  public Rotation2d calculatePivotAngle(Translation3d targetPoint) {

    double targetHeight = targetPoint.getZ() - SHOOTER_PIVOT.POSITION.getZ();
    
    double targetDistance = ShooterPivot.getShooterLocationOnField(swerveDrive.getPose()).toTranslation2d().getDistance(targetPoint.toTranslation2d());

    double projectileVelocity = getProjectileVelocity();

    double gravity = 9.80;
    return Rotation2d.fromRadians(Math.atan((Math.pow(projectileVelocity, 2.0) - Math.sqrt(Math.pow(projectileVelocity, 4.0) - gravity * (gravity * Math.pow(targetDistance, 2.0) + 2.0 * targetHeight * Math.pow(projectileVelocity, 2.0)))) / (gravity * targetDistance)));
  }

  public double getProjectileVelocity() {
    // Derived from https://www.reca.lc/flywheel
    double shooterWheelSurfaceSpeed = getVelocity() * SHOOTER_WHEELS.WHEEL_RADIUS;
    double speedTransferPercentage = (SHOOTER_WHEELS.WHEEL_MOI * 20.0) / (SHOOTER_WHEELS.PROJECTILE_MASS * SHOOTER_WHEELS.WHEEL_RADIUS * 2.0 * SHOOTER_WHEELS.WHEEL_RADIUS * 2.0 * 7.0 + SHOOTER_WHEELS.WHEEL_MOI * 40.0);
    return shooterWheelSurfaceSpeed * speedTransferPercentage;
  }

  public void setTargetAngularVelocity(double angularVelocity) {
    targetAngularVelocity = angularVelocity;
  }

  public double getVelocity() {
    return SHOOTER_WHEELS.TARGET_SPEED;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    if (isCalibrating) return;
    loop.setNextR(VecBuilder.fill(targetAngularVelocity));
    loop.correct(VecBuilder.fill(getVelocity()));
    loop.predict(0.020);
    double nextVoltage = loop.getU(0);
    motor.setVoltage(nextVoltage);
  }

  @Override
  public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
  }

  public Command calibrate() {
    SysIdRoutine calibrationRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
          motor.set(volts.in(Volts) / RobotController.getBatteryVoltage());
        },
        log -> {
          log.motor("shooter-wheels")
            .voltage(Volts.of(motor.get() * RobotController.getBatteryVoltage()))
            .linearPosition(Meters.of(encoder.getPosition()))
            .linearVelocity(MetersPerSecond.of(encoder.getVelocity()));
        },
        this
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> isCalibrating = true),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> isCalibrating = false)
    );
  }
}
