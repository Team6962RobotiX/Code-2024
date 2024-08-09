package frc.robot.subsystems.drive.alt.commanddrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.alt.SwerveConfig;
import frc.robot.subsystems.drive.alt.field.FieldElement;

public class CommandDrive extends CommandDriveBase {
    private SwerveConfig swerveConfig;
    private CommandableSwerve swerveDrive;
    private PIDController headingPID;
    private Rotation2d headingCalculation;
    private AutoPath autoPath = new AutoPath();

    public CommandDrive(SwerveConfig swerveConfig, SwerveDriveKinematics kinematics, CommandableSwerve swerveDrive) {
        super(kinematics, swerveDrive::driveModules);

        this.swerveDrive = swerveDrive;
        this.swerveConfig = swerveConfig;
        
        headingPID = new PIDController(
            swerveConfig.headingPID().kP(),
            swerveConfig.headingPID().kI(),
            swerveConfig.headingPID().kD()
        );

        headingPID.enableContinuousInput(-Math.PI / 2, Math.PI / 2);
        headingPID.setTolerance(Rotation2d.fromDegrees(1).getRadians());

        configurePathplanner();
    }

    @Override
    public void periodic() {
        super.periodic();

        headingCalculation = Rotation2d.fromRadians(headingPID.calculate(
            swerveDrive.getEstimatedPose().getRotation().getRadians()
        ));
    }

    public FieldElement getAutoPath() {
        return autoPath;
    }

    public Command drive(ChassisSpeeds speeds) {
        return driveSpeeds(() -> speeds);
    }

    public Command driveSpeeds(Supplier<ChassisSpeeds> speeds) {
        return Commands.run(() -> {
            getCombinedReceiver().setChassisSpeeds(speeds.get());
        }, getCombinedReceiver(), getRotationReceiver(), getTranslationReceiver());
    }

    public Command drive(SwerveModuleState[] states) {
        return driveModules(() -> states);
    }

    public Command driveModules(Supplier<SwerveModuleState[]> states) {
        return Commands.run(() -> {
            getCombinedReceiver().setModuleStates(states.get());
        }, getCombinedReceiver(), getRotationReceiver(), getTranslationReceiver());
    }

    public Command drive(Translation2d translationVelocity) {
        return driveTranslation(() -> translationVelocity);
    }

    public Command driveTranslation(Supplier<Translation2d> translationVelocity) {
        return Commands.run(() -> {
            getTranslationReceiver().setTranslationVelocity(translationVelocity.get());
        }, getTranslationReceiver());
    }

    public Command drive(Rotation2d rotationVelocity) {
        return driveRotation(() -> rotationVelocity);
    }

    public Command driveRotation(Supplier<Rotation2d> rotationVelocity) {
        return Commands.run(() -> {
            getRotationReceiver().setAngularVelocity(rotationVelocity.get());
        }, getRotationReceiver());
    }

    public Command driveHeading(Rotation2d heading) {
        return driveHeading(() -> heading);
    }

    public Command driveHeading(Supplier<Rotation2d> heading) {
        return Commands.run(() -> {
            headingPID.setSetpoint(heading.get().getRadians());
            getRotationReceiver().setAngularVelocity(headingCalculation);
        }, getRotationReceiver());
    }

    public Command facePoint(Translation2d point) {
        return facePoint(() -> point);
    }

    public Command facePoint(Supplier<Translation2d> point) {
        return Commands.run(() -> {
            Translation2d robotPosition = swerveDrive.getEstimatedPose().getTranslation();
            Translation2d offset = point.get().minus(robotPosition);

            Rotation2d heading = offset.getAngle();

            headingPID.setSetpoint(heading.getRadians());
            getRotationReceiver().setAngularVelocity(headingCalculation);
        }, getRotationReceiver());
    }

    /**
     * Returns whether the robot is at the target heading set by
     * {@link #driveHeading(Rotation2d)} or {@link #facePoint(Translation2d)}.
     */
    public boolean atTargetHeading() {
        return headingPID.atSetpoint();
    }

    /**
     * Stops the robot without rotating the wheels.
     * @return A command that stops the robot.
     */
    public Command stop() {
        return Commands.run(() -> {
            SwerveModuleState[] stopped = new SwerveModuleState[4];
            SwerveModuleState[] current = swerveDrive.getModuleStates();

            for (int i = 0; i < current.length; i++) {
                stopped[i] = new SwerveModuleState(0, current[i].angle);
            }

            getCombinedReceiver().setModuleStates(stopped);
        }, getCombinedReceiver(), getRotationReceiver(), getTranslationReceiver());
    }

    /**
     * Parks the robot by putting the wheels in an {@code X} shape and stopping
     * them, making the robot much harder to push.
     * @return A command that parks the robot.
     */
    public Command park() {
        return Commands.run(() -> {
            getCombinedReceiver().setModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
            });
        }, getCombinedReceiver(), getRotationReceiver(), getTranslationReceiver());
    }

    private ChassisSpeeds pathSpeeds;
    private void configurePathplanner() {
        AutoBuilder.configureHolonomic(
            this.swerveDrive::getEstimatedPose, // Robot pose supplier
            (Pose2d pose) -> {
                throw new UnsupportedOperationException("Resetting odometry is not supported. Do not give an auto path a starting pose.");
            }, // Method to reset odometry (will be called if your auto has a starting pose)
            this.swerveDrive::getEstimatedSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (ChassisSpeeds speeds) -> {
                pathSpeeds = speeds;
            }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                swerveConfig.pathTranslationPID().toPathplannerPIDConstants(), // Translation PID constants
                swerveConfig.pathRotationPID().toPathplannerPIDConstants(), // Rotation PID constants
                swerveConfig.maxLinearWheelSpeed().in(MetersPerSecond), // Max module speed, in m/s
                swerveConfig.driveRadius().in(Meters),
                new ReplanningConfig(true, true)
            ),
            () -> false, // Should flip paths
            null // Reference to this subsystem to set requirements
        );

        PathPlannerLogging.setLogTargetPoseCallback(autoPath::setTargetPose);
        PathPlannerLogging.setLogActivePathCallback(autoPath::setActivePath);
    }

    private Command simplePathfind(Pose2d targetPose) {
        Pose2d estimatedPose = swerveDrive.getEstimatedPose();
        Rotation2d angle = targetPose.getTranslation().minus(estimatedPose.getTranslation()).getAngle();

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(estimatedPose.getTranslation(), angle),
            new Pose2d(targetPose.getTranslation(), angle)
        );

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            swerveConfig.pathConstraints(),
            new GoalEndState(
                0.0,
                targetPose.getRotation(),
                true
            )
        );

        return Commands.sequence(
            AutoBuilder.followPath(path),
            Commands.race(
                driveHeading(targetPose.getRotation()),
                Commands.waitUntil(this::atTargetHeading)
            )
        );
    }

    public class PathfindTo extends Command {
        private Command command;
        private Pose2d pose;
        private CombinedReceiver combinedReceiver;
      
        public PathfindTo(Pose2d pose, CombinedReceiver receiver, RotationReceiver rotationReceiver, TranslationReceiver translationReceiver) {
            this.pose = pose;
            this.combinedReceiver = receiver;

            combinedReceiver.setChassisSpeeds(new ChassisSpeeds());

            addRequirements(receiver, rotationReceiver, translationReceiver);
        }
      
        @Override
        public void initialize() {
            command = simplePathfind(pose);

            command.schedule();
        }

        @Override
        public void execute() {
            combinedReceiver.setChassisSpeeds(pathSpeeds);
        }

        @Override
        public void end(boolean interrupted) {
            command.cancel();
        }

        @Override
        public boolean isFinished() {
            return command.isScheduled() && command.isFinished();
        }
    }

    public Command pathfindTo(Pose2d pose) {
        return new PathfindTo(pose, getCombinedReceiver(), getRotationReceiver(), getTranslationReceiver());
    }
}