package frc.robot.subsystems.drive.alt;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

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
    MotorProfile steerMotorProfile
) {
    public Measure<Velocity<Distance>> maxLinearWheelSpeed() {
        return MetersPerSecond.of(maxWheelSpeed.in(RotationsPerSecond) * wheelRadius.in(Meters));
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

    public static record PID(
        double kP,
        double kI,
        double kD
    ) {
    }

    public static record MotorProfile(
        double kS,
        double kV,
        double kA
    ) {
    }
}