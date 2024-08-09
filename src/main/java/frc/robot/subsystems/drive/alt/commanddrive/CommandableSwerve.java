package frc.robot.subsystems.drive.alt.commanddrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.alt.SwerveDrive;

/**
 * An abstraction for a swerve drive that can be used with CommandDrive.
 */
public interface CommandableSwerve {
    public void driveModules(SwerveModuleState[] states);
    public Pose2d getEstimatedPose();
    public ChassisSpeeds getEstimatedSpeeds();
    public SwerveModuleState[] getModuleStates();

    public static CommandableSwerve wrapAltSwerve(SwerveDrive altSwerve) {
        return new CommandableSwerve() {
            @Override
            public void driveModules(SwerveModuleState[] states) {
                altSwerve.drive(states);
            }

            @Override
            public Pose2d getEstimatedPose() {
                return altSwerve.getEstimatedPose();
            }

            @Override
            public ChassisSpeeds getEstimatedSpeeds() {
                return altSwerve.getEstimatedSpeeds();
            }

            @Override
            public SwerveModuleState[] getModuleStates() {
                return altSwerve.getModuleStates();
            }
        };
    }

    public static CommandableSwerve wrapOriginalSwerve(frc.robot.subsystems.drive.SwerveDrive swerveDrive) {
        return new CommandableSwerve() {
            @Override
            public void driveModules(SwerveModuleState[] states) {
                for (int i = 0; i < 4; i++) {
                    swerveDrive.modules[i].setTargetState(states[i]);
                }
            }

            @Override
            public Pose2d getEstimatedPose() {
                return swerveDrive.getPose();
            }

            @Override
            public ChassisSpeeds getEstimatedSpeeds() {
                return new ChassisSpeeds(
                    swerveDrive.getFieldVelocity().getX(),
                    swerveDrive.getFieldVelocity().getY(),
                    swerveDrive.getRotationalVelocity()
                );
            }

            @Override
            public SwerveModuleState[] getModuleStates() {
                SwerveModuleState[] states = new SwerveModuleState[4];

                for (int i = 0; i < 4; i++) {
                    states[i] = swerveDrive.modules[i].getMeasuredState();
                }

                return states;
            }
        };
    }
}