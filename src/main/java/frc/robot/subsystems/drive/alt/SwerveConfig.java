package frc.robot.subsystems.drive.alt;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.drive.alt.units.Force.Newtons;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import frc.robot.subsystems.drive.alt.units.Force;

public record SwerveConfig(
    Measure<Distance> trackWidth,
    Measure<Distance> wheelBase,
    /**
     * The equipped swerve modules, in the order of front left, front right, back left, back right.
    */
    Module[] equippedModules,
    Measure<Distance> wheelRadius,
    Measure<Velocity<Angle>> maxWheelSpeed,
    Measure<Velocity<Velocity<Distance>>> maxRobotAcceleration,
    MotorProfile driveMotorProfile,
    MotorProfile steerMotorProfile,
    MotorInfo swerveMotorInfo,
    double driveMotorGearing,
    double steerMotorGearing
) {
    public Measure<Velocity<Distance>> maxLinearWheelSpeed() {
        return MetersPerSecond.of(maxWheelSpeed.in(RotationsPerSecond) * wheelRadius.in(Meters));
    }

    public Measure<Distance> driveRadius() {
        return Meters.of(Math.hypot(wheelBase().in(Meters) / 2.0, trackWidth().in(Meters) / 2.0));
    }

    public static record MotorInfo(
        DCMotor stats,
        Measure<Temperature> maxTemperature,
        Measure<Current> maxStallCurrent,
        Measure<Current> maxFreeCurrent,
        Measure<Velocity<Current>> maxRampRate
    ) {
        public Measure<Force> maxTorqueCurrentLimited(Measure<Current> currentLimit) {
            return Newtons.of(stats.stallTorqueNewtonMeters / stats.stallCurrentAmps * currentLimit.in(Amps));
        }
    }

    public static record Module(
        /**
         * The CAN ID of the drive motor.
         */
        int driveMotorId,
        /**
         * The CAN ID of the steer motor.
         */
        int steerMotorId,
        /**
         * The CAN ID of the absolute encoder.
         */
        int absoluteEncoderId,
        /**
         * The offset of the absolute encoder.
        */
        Measure<Angle> absoluteEncoderOffset
    ) {
        public Module(int driveMotorId, int steerMotorId, int absoluteEncoderId) {
            this(driveMotorId, steerMotorId, absoluteEncoderId, Degrees.of(0));
        }
    }

    public static record MotorProfile(
        // Feedforward control
        double kS,
        double kV,
        double kA,

        // Feedback control
        double kP,
        double kI,
        double kD
    ) {
    }
}