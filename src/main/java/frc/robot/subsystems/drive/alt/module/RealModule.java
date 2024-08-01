// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.alt.module;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.Constants.Constants.SWERVE_DRIVE.DRIVE_MOTOR_PROFILE;
import frc.robot.Constants.Constants.SWERVE_DRIVE.PHYSICS;
import frc.robot.Constants.Constants.SWERVE_DRIVE.STEER_MOTOR_PROFILE;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.subsystems.drive.alt.SwerveConfig;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;

public class RealModule extends SubsystemBase implements SwerveModule {
    private int corner;

    private CANSparkMax driveMotor, steerMotor;
    private RelativeEncoder driveEncoder, steerEncoder;
    private CANcoder absoluteSteerEncoder;
    private SparkPIDController drivePID, steerPID;

    private SwerveModuleState targetState = new SwerveModuleState();

    private SwerveModuleState measuredState = new SwerveModuleState();
    private SwerveModulePosition measuredPosition = new SwerveModulePosition();
    
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        DRIVE_MOTOR_PROFILE.kS,
        DRIVE_MOTOR_PROFILE.kV,
        DRIVE_MOTOR_PROFILE.kA
    );

    private SwerveConfig.Module config;

    public RealModule(SwerveConfig.Module config, int corner) {
        this.corner = corner;
        this.config = config;

        driveMotor = new CANSparkMax(config.driveMotorId(), MotorType.kBrushless);
        steerMotor = new CANSparkMax(config.steerMotorId(), MotorType.kBrushless);

        absoluteSteerEncoder = new CANcoder(config.absoluteEncoderId());
        steerEncoder = steerMotor.getEncoder();
        driveEncoder = driveMotor.getEncoder();

        drivePID = driveMotor.getPIDController();
        steerPID = steerMotor.getPIDController();

        BaseStatusSignal.setUpdateFrequencyForAll(50, absoluteSteerEncoder.getAbsolutePosition(), absoluteSteerEncoder.getFaultField(), absoluteSteerEncoder.getVersion());
        absoluteSteerEncoder.optimizeBusUtilization();

        SparkMaxUtil.configureAndLog(this, driveMotor, false, CANSparkMax.IdleMode.kBrake, PHYSICS.SLIPLESS_CURRENT_LIMIT, PHYSICS.SLIPLESS_CURRENT_LIMIT);
        SparkMaxUtil.configureAndLog(this, steerMotor, true, CANSparkMax.IdleMode.kCoast);
        SparkMaxUtil.configureEncoder(driveMotor, SWERVE_DRIVE.DRIVE_ENCODER_CONVERSION_FACTOR);
        SparkMaxUtil.configureEncoder(steerMotor, SWERVE_DRIVE.STEER_ENCODER_CONVERSION_FACTOR);
        SparkMaxUtil.configurePID(this, driveMotor, DRIVE_MOTOR_PROFILE.kP, DRIVE_MOTOR_PROFILE.kI, DRIVE_MOTOR_PROFILE.kD, 0.0, false);
        SparkMaxUtil.configurePID(this, steerMotor, STEER_MOTOR_PROFILE.kP, STEER_MOTOR_PROFILE.kI, STEER_MOTOR_PROFILE.kD, 0.0, true);

        driveMotor.setClosedLoopRampRate(SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY / SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION);
        driveMotor.setOpenLoopRampRate(SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY / SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION);
        
        SparkMaxUtil.configureCANStatusFrames(driveMotor, true, true);
        SparkMaxUtil.configureCANStatusFrames(steerMotor, false, true);
        
        seedRelativeSteerEncoder();

        String logPath = SwerveModule.getLogPath(corner);
        Logger.autoLog(this, logPath + "measuredState", () -> getMeasuredState());
        Logger.autoLog(this, logPath + "measuredAngle", () -> getMeasuredState().angle.getDegrees());
        Logger.autoLog(this, logPath + "measuredVelocity", () -> getMeasuredState().speedMetersPerSecond);
        Logger.autoLog(this, logPath + "targetState", () -> getTargetState());
        Logger.autoLog(this, logPath + "targetAngle", () -> getTargetState().angle.getDegrees());
        Logger.autoLog(this, logPath + "targetVelocity", () -> getTargetState().speedMetersPerSecond);

        StatusChecks.addCheck(this, logPath + "canCoderHasFaults", () -> absoluteSteerEncoder.getFaultField().getValue() == 0);
        StatusChecks.addCheck(this, logPath + "canCoderIsConnected", () -> absoluteSteerEncoder.getVersion().getValue() != 0);

        setDefaultCommand(driveModule());
    }

    @Override
    public void periodic() {
        Rotation2d steerDirection = new Rotation2d(Rotations.of(absoluteSteerEncoder.getAbsolutePosition().getValue()).plus(config.absoluteEncoderOffset()));
        double driveVelocity = driveEncoder.getVelocity();
        double drivePosition = driveEncoder.getPosition();

        measuredState = new SwerveModuleState(driveVelocity, steerDirection);
        measuredPosition = new SwerveModulePosition(drivePosition, steerDirection);
    }
    
    /**
     * Drives the module to a target state (wheel speed and orientation).
     */
    private void driveState(SwerveModuleState targetState) {
        drivePID.setReference(
            targetState.speedMetersPerSecond,
            CANSparkBase.ControlType.kVelocity,
            0,
            driveFeedforward.calculate(targetState.speedMetersPerSecond)
        );

        steerPID.setReference(
            targetState.angle.getRadians(),
            CANSparkBase.ControlType.kPosition
        );

        if (Math.abs(steerMotor.getAppliedOutput()) < 0.1 && targetState.speedMetersPerSecond == 0) {
            seedRelativeSteerEncoder();
        }

        if (RobotContainer.getVoltage() < VOLTAGE_LADDER.SWERVE_DRIVE) {
            driveMotor.stopMotor();
            steerMotor.stopMotor();
        }
    }

    public void drive(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getMeasuredState().angle);
    }
    
    /**
     * Stops the module by setting the target module state to the current wheel position.
     */
    public void stop() {
        targetState = new SwerveModuleState(0.0, getMeasuredState().angle);
    }
    
    /**
     * Seeds the position of the relative encoder built-in to the steering motor with the
     * absolute position of the steer CANCoder. The absolute encoders update much less frequently
     * than the relative encoders, so by seeding the relative encoder with the absolute encoder's
     * position, we can get more frequent updates of the wheel rotation.
     */
    private void seedRelativeSteerEncoder() {
        steerEncoder.setPosition(getMeasuredState().angle.getRadians());
    }

    /**
     * Gets the target module state (wheel speed and orientation).
     */
    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Gets the measured module state (wheel speed and orientation).
     */
    public SwerveModuleState getMeasuredState() {
        return measuredState;
    }

    /**
     * Gets the measured module position (wheel position and orientation).
     */
    public SwerveModulePosition getMeasuredPosition() {
        return measuredPosition;
    }

    /**
     * Gets the total current draw of the module.
     */
    public Measure<Voltage> getTotalCurrent() {
        return Volts.of(driveMotor.getOutputCurrent() + steerMotor.getOutputCurrent());
    }

    /**
     * Creates a command that drives the module to the target state. This is used as the default command, as
     * an alternative to calibration commands.
     */
    private Command driveModule() {
        return Commands.run(() -> driveState(targetState));
    }

    /**
     * Creates a command that runs a system identification routine on the steering motor.
     */
    public Command calibrateSteerMotor() {
        return calibrateMotor(steerMotor, steerEncoder, "steer");
    }

    /**
     * Creates a command that runs a system identification routine on the drive motor.
     */
    public Command calibrateDriveMotor() {
        return calibrateMotor(driveMotor, driveEncoder, "drive");
    }

    private Command calibrateMotor(CANSparkMax motor, RelativeEncoder encoder, String loggingLabel) {
        SysIdRoutine routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> motor.setVoltage(volts.in(Volts)),
                        log -> log.motor("module-" + corner + "-" + loggingLabel)
                                .voltage(Volts.of(motor.getBusVoltage() * motor.getAppliedOutput()))
                                .angularPosition(Radians.of(encoder.getPosition()))
                                .angularVelocity(RadiansPerSecond.of(encoder.getVelocity())),
                        this
                )
        );

        Command command = Commands.sequence(
            Commands.runOnce(() -> SparkMaxUtil.configureCANStatusFrames(motor, true, true)),
            Commands.waitSeconds(1.0),
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.runOnce(() -> steerMotor.stopMotor()),
            Commands.waitSeconds(1.0),
            routine.quasistatic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(() -> steerMotor.stopMotor()),
            Commands.waitSeconds(1.0),
            routine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.runOnce(() -> steerMotor.stopMotor()),
            Commands.waitSeconds(1.0),
            routine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(() -> steerMotor.stopMotor()),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> SparkMaxUtil.configureCANStatusFrames(motor, false, false))
        );

        command.addRequirements(this);

        return command;
    }
}