package frc.robot.subsystems.drive.alt.commanddrive;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandDriveBase extends SubsystemBase {
    private RotationReceiver rotationReceiver;
    private TranslationReceiver translationReceiver;
    private CombinedReceiver combinedReceiver;
    private Consumer<SwerveModuleState[]> drive;
    private SwerveDriveKinematics kinematics;

    public CommandDriveBase(SwerveDriveKinematics kinematics, Consumer<SwerveModuleState[]> drive) {
        rotationReceiver = new RotationReceiver();
        translationReceiver = new TranslationReceiver();
        combinedReceiver = new CombinedReceiver(kinematics);

        this.drive = drive;
        this.kinematics = kinematics;
    }

    public RotationReceiver getRotationReceiver() {
        return rotationReceiver;
    }

    public TranslationReceiver getTranslationReceiver() {
        return translationReceiver;
    }

    public CombinedReceiver getCombinedReceiver() {
        return combinedReceiver;
    }

    @Override
    public void periodic() {
        if ((rotationReceiver.isActive() || translationReceiver.isActive()) && combinedReceiver.isActive()) {
            throw new IllegalStateException("Cannot have both rotation and translation receivers active at the same time as a combined receiver");
        }

        if (rotationReceiver.isActive() || translationReceiver.isActive()) {
            Translation2d velocity = translationReceiver.getVelocity();
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                velocity.getX(),
                velocity.getY(),
                rotationReceiver.getAngularVelocity().getRadians()
            );

            drive.accept(kinematics.toSwerveModuleStates(chassisSpeeds));
        } else if (combinedReceiver.isActive()) {
            drive.accept(combinedReceiver.getStates());
        }
    }

    public static class RotationReceiver extends Receiver {
        private Rotation2d angularVelocity = new Rotation2d();

        public Rotation2d getAngularVelocity() {
            return isActive() ? new Rotation2d() : angularVelocity;
        }

        public void setAngularVelocity(Rotation2d angularVelocity) {
            this.angularVelocity = angularVelocity;
        }
    }

    public static class TranslationReceiver extends Receiver {
        private Translation2d velocity = new Translation2d();

        public Translation2d getVelocity() {
            return isActive() ? new Translation2d() : velocity;
        }

        public void setTranslationVelocity(Translation2d velocity) {
            this.velocity = velocity;
        }
    }

    public static class CombinedReceiver extends Receiver {
        private SwerveModuleState[] states = new SwerveModuleState[4];
        private SwerveDriveKinematics kinematics;

        public CombinedReceiver(SwerveDriveKinematics kinematics) {
            this.kinematics = kinematics;
        }

        public SwerveModuleState[] getStates() {
            return isActive() ? new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            } : states;
        }

        public void setModuleStates(SwerveModuleState[] states) {
            this.states = states;
        }

        public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
            this.states = toSwerveModuleStates(chassisSpeeds, kinematics);
        }
    }

    public static class Receiver extends SubsystemBase {
        protected boolean active = false;

        public Receiver() {
            setDefaultCommand(new Command() {
                @Override
                public void execute() {
                    active = false;
                }
            });
        }

        public boolean isActive() {
            return active;
        }
    }

    public static SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds, SwerveDriveKinematics kinematics) {
        return kinematics.toSwerveModuleStates(speeds);
    }
}
