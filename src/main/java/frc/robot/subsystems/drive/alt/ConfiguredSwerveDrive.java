package frc.robot.subsystems.drive.alt;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Constants.NEO;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.Constants.Constants.SWERVE_DRIVE.MODULE_CONFIG;
import frc.robot.Constants.Field;
import frc.robot.util.software.MathUtils;

public class ConfiguredSwerveDrive extends SwerveDrive {
    private static SwerveConfig config;

    static {
        MODULE_CONFIG[] equippedModules = SWERVE_DRIVE.IS_PROTOTYPE_CHASSIS ? SWERVE_DRIVE.EQUIPPED_MODULES_PROTOTYPE : SWERVE_DRIVE.EQUIPPED_MODULES_COMPETITION;
        SwerveConfig.Module[] moduleConfigs = new SwerveConfig.Module[equippedModules.length];

        for (int i = 0; i < equippedModules.length; i++) {
            MODULE_CONFIG module = equippedModules[i];
            moduleConfigs[i] = new SwerveConfig.Module(
                module.CAN_DRIVE(),
                module.CAN_STEER(),
                module.CAN_ENCODER(),
                Rotations.of(module.ENCODER_OFFSET())
            );
        }

        config = new SwerveConfig(
            Meters.of(SWERVE_DRIVE.TRACKWIDTH),
            Meters.of(SWERVE_DRIVE.WHEELBASE),
            moduleConfigs,
            Meters.of(SWERVE_DRIVE.WHEEL_RADIUS),
            RotationsPerSecond.of(SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY),
            MetersPerSecondPerSecond.of(SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION),
            new SwerveConfig.MotorProfile(
                SWERVE_DRIVE.DRIVE_MOTOR_PROFILE.kS,
                SWERVE_DRIVE.DRIVE_MOTOR_PROFILE.kV,
                SWERVE_DRIVE.DRIVE_MOTOR_PROFILE.kA,
                SWERVE_DRIVE.DRIVE_MOTOR_PROFILE.kP,
                SWERVE_DRIVE.DRIVE_MOTOR_PROFILE.kI,
                SWERVE_DRIVE.DRIVE_MOTOR_PROFILE.kD
            ),
            new SwerveConfig.MotorProfile(
                SWERVE_DRIVE.STEER_MOTOR_PROFILE.kS,
                SWERVE_DRIVE.STEER_MOTOR_PROFILE.kV,
                SWERVE_DRIVE.STEER_MOTOR_PROFILE.kA,
                SWERVE_DRIVE.STEER_MOTOR_PROFILE.kP,
                SWERVE_DRIVE.STEER_MOTOR_PROFILE.kI,
                SWERVE_DRIVE.STEER_MOTOR_PROFILE.kD
            ),
            new SwerveConfig.MotorInfo(
                NEO.STATS,
                Celsius.of(NEO.SAFE_TEMPERATURE),
                Amps.of(NEO.SAFE_STALL_CURRENT),
                Amps.of(NEO.SAFE_FREE_CURRENT),
                Amps.per(Second).of(NEO.SAFE_RAMP_RATE)
            ),
            SWERVE_DRIVE.DRIVE_MOTOR_GEARING,
            SWERVE_DRIVE.STEER_MOTOR_GEARING,
            new SwerveConfig.PIDConstants(
                SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kP,
                SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kI,
                SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kD
            ),
            new SwerveConfig.PIDConstants(
                SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP,
                SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI,
                SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD
            ),
            new SwerveConfig.PIDConstants(
                SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kP,
                SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kI,
                SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kD
            ),
            SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS
        );
    }

    public ConfiguredSwerveDrive() {
        super(config);
    }

    // TODO: Look at optimization used in original SwerveDrive class
    public boolean isUnderStage() {
        Translation2d futurePose = getFuturePose().getTranslation();
        Translation2d currentPose = getEstimatedPose().getTranslation();

        return MathUtils.isInsideTriangle(Field.BLUE_STAGE_CORNERS[0], Field.BLUE_STAGE_CORNERS[1], Field.BLUE_STAGE_CORNERS[2], futurePose) ||
            MathUtils.isInsideTriangle(Field.RED_STAGE_CORNERS[0], Field.RED_STAGE_CORNERS[1], Field.RED_STAGE_CORNERS[2], futurePose) ||
            MathUtils.isInsideTriangle(Field.BLUE_STAGE_CORNERS[0], Field.BLUE_STAGE_CORNERS[1], Field.BLUE_STAGE_CORNERS[2], currentPose) ||
            MathUtils.isInsideTriangle(Field.RED_STAGE_CORNERS[0], Field.RED_STAGE_CORNERS[1], Field.RED_STAGE_CORNERS[2], currentPose);
    }
}
