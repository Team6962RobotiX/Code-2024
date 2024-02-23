package frc.robot.util.hardware.MotionControl;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.NEO;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;

/*
 * Uses oboard 1kHz PID, Feedforward, and Trapazoidal Profiles to
 * control a pivot mechanism precisely, smoothly, and accurately
 */

public class PivotController {
  private Rotation2d targetAngle;
  State setpointState;
  // Feedfoward controller, read: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html
  private ArmFeedforward feedforward;
  // Trapezoidal Profile controller, read: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
  private TrapezoidProfile profile;
  // Onboard spark max PID controller. Runs at 1kHz
  private SparkPIDController pid;
  // CAN Spark Max motor controller;
  private CANSparkMax motor;
  // Built-in relative NEO encoder
  private RelativeEncoder encoder;
  // Rev absolute through-bore encoder
  private DutyCycleEncoder absoluteEncoder;

  private Rotation2d minAngle, maxAngle;

  private double encoderOffset = 0.0;

  private boolean reversed;

  public PivotController(SubsystemBase subsystem, CANSparkMax motor, int absoluteEncoderDIO, double absolutePositionOffset, double kP, double gearing, double maxAcceleration, Rotation2d minAngle, Rotation2d maxAngle, boolean reversed) {
    this(subsystem, motor, absoluteEncoderDIO, absolutePositionOffset, kP, 0.0, 0.0, 0.0, 0.0, 12.0 / (NEO.STATS.freeSpeedRadPerSec / gearing), 0.0, gearing, NEO.STATS.freeSpeedRadPerSec / gearing, maxAcceleration, minAngle, maxAngle, reversed);
  }

  public PivotController(SubsystemBase subsystem, CANSparkMax motor, int absoluteEncoderDIO, double absolutePositionOffset, double kP, double kI, double kD, double kS, double kG, double kV, double kA, double gearing, double maxVelocity, double maxAcceleration, Rotation2d minAngle, Rotation2d maxAngle, boolean reversed) {
    feedforward = new ArmFeedforward(kS, kG, kV, kA);
    profile = new TrapezoidProfile(
      new Constraints(maxVelocity, maxAcceleration)
    );
    pid = motor.getPIDController();
    encoder = motor.getEncoder();
    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderDIO);

    this.motor = motor;
    this.minAngle = minAngle;
    this.maxAngle = maxAngle;

    this.reversed = reversed;
    encoderOffset = absolutePositionOffset;

    SparkMaxUtil.configureEncoder(motor, 2.0 * Math.PI / gearing);
    SparkMaxUtil.configurePID(subsystem, motor, kP, kI, kD, 0.0, true);

    Logger.autoLog(subsystem, "absolutePosition",                 () -> getAbsolutePosition().getRadians());
    Logger.autoLog(subsystem, "rawAbsolutePosition",              () -> absoluteEncoder.getAbsolutePosition());
    
    StatusChecks.addCheck(subsystem, "absoluteEncoderConnected", () -> absoluteEncoder.isConnected());
    StatusChecks.addCheck(subsystem, "absoluteEncoderUpdated",   () -> absoluteEncoder.getAbsolutePosition() != 0.0);
  }

  public void run() {
    if (targetAngle == null) return; // If we havent set a target angle yet, do nothing
    if (!absoluteEncoder.isConnected()) {
      motor.stopMotor();
      return;
    }

    // System.out.println(getPosition().getDegrees());

    // Re-seed the relative encoder with the absolute encoder when not moving
    if (Math.abs(encoder.getVelocity()) < 0.001) {
      encoder.setPosition(getAbsolutePosition().getRadians());
    }
    
    if (setpointState == null) {
      setpointState = new State(getAbsolutePosition().getRadians(), getVelocity().getRadians());
    };

    // Calculate the setpoint following a trapazoidal profile (smooth ramp up and down acceleration curves)
    State targetState = new State(targetAngle.getRadians(), 0.0);


    setpointState = profile.calculate(0.02, setpointState, targetState);

    // Set onboard PID controller to follow
    pid.setReference(
      setpointState.position,
      ControlType.kPosition,
      0,
      feedforward.calculate(setpointState.position, setpointState.velocity),
      ArbFFUnits.kVoltage
    );
  }

  public void setTargetAngle(Rotation2d angle) {
    if (angle.getRadians() < minAngle.getRadians()) {
        targetAngle = minAngle;
    } else if (angle.getRadians() > maxAngle.getRadians()) {
        targetAngle = maxAngle;
    } else {
        targetAngle = angle;
    }
  }

  public Rotation2d getTargetAngle() {
    return targetAngle;
  }

  public boolean isPastLimit() {
    return encoder.getPosition() > maxAngle.getRadians() || encoder.getPosition() < minAngle.getRadians();
  }

  public Rotation2d getAbsolutePosition() {
    double factor = 1;
    if (reversed) {
      factor = -1;
    }
    // Map absolute encoder position from 0 - 1 rotations to -pi - pi radians, where 0 is straight out
    double absoluteAngle = (absoluteEncoder.getAbsolutePosition() + encoderOffset) * factor;
    while (absoluteAngle < 0) absoluteAngle++;
    absoluteAngle %= 1.0;
    
    absoluteAngle *= Math.PI * 2.0;
    if (absoluteAngle > Math.PI) {
      absoluteAngle -= Math.PI * 2.0;
    }

    return Rotation2d.fromRadians(absoluteAngle);
  }

  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(encoder.getPosition());
  }

  public Rotation2d getVelocity() {
    return Rotation2d.fromRadians(encoder.getVelocity());
  }
}