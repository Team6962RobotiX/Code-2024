package frc.robot.subsystems.drive.alt.controllerdrive;

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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.alt.SwerveConfig;
import frc.robot.subsystems.drive.alt.SwerveDrive;
import frc.robot.subsystems.drive.alt.field.FieldElement;

public class ControllerDrive extends SubsystemBase {
    private RotationController rotationController;
    private TranslationController translationController;
    private boolean parked;

    private PIDController headingPID;
    private Rotation2d headingCalculation;
    private SwerveDrive swerveDrive;
    private SwerveConfig swerveConfig;

    private Command runningCommand = null;

    private AutoPath autoPath = new AutoPath();

    public ControllerDrive(SwerveConfig swerveConfig, SwerveDrive swerveDrive) {
        this.swerveConfig = swerveConfig;

        headingPID = new PIDController(
            swerveConfig.headingPID().kP(),
            swerveConfig.headingPID().kI(),
            swerveConfig.headingPID().kD()
        );

        headingPID.enableContinuousInput(-Math.PI / 2, Math.PI / 2);

        this.swerveDrive = swerveDrive;

        configurePathplanner();
    }

    @Override
    public void periodic() {
        headingCalculation = Rotation2d.fromRadians(headingPID.calculate(swerveDrive.getEstimatedPose().getRotation().getRadians()));

        if (parked) {
            swerveDrive.drive(new SwerveModuleState[] {
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
            });
        } else {
            if (rotationController != null) rotationController.update();
            if (translationController != null) translationController.update();

            Rotation2d rotation = rotationController != null ? rotationController.getAngularVelocity() : Rotation2d.fromDegrees(0);
            Translation2d translation = translationController != null ? translationController.getTranslationVelocity() : new Translation2d(0, 0);

            swerveDrive.drive(new ChassisSpeeds(translation.getX(), translation.getY(), rotation.getRadians()));
        }
    }

    private void setRotationController(RotationController controller) {
        if (translationController == rotationController) translationController = null;
        rotationController = controller;
        if (controller != null) parked = false;
        if (runningCommand != null) {
            runningCommand.cancel();
            runningCommand = null;
        }
    }

    private void setTranslationController(TranslationController controller) {
        if (translationController == rotationController) rotationController = null;
        translationController = controller;
        if (controller != null) parked = false;
        if (runningCommand != null) {
            runningCommand.cancel();
            runningCommand = null;
        }
    }

    public void rotate(Rotation2d velocity) {
        rotate(() -> velocity);
    }

    public void rotate(Supplier<Rotation2d> velocity) {
        setRotationController(new RotationController() {
            @Override
            public Rotation2d getAngularVelocity() {
                return velocity.get();
            }
        });
    }

    public void faceHeading(Rotation2d heading) {
        faceHeading(() -> heading);
    }

    public void faceHeading(Supplier<Rotation2d> heading) {
        headingPID.setSetpoint(heading.get().getRadians());

        rotate(() -> {
            headingPID.setSetpoint(heading.get().getRadians());

            return headingCalculation;
        });
    }

    public void lookAt(Translation2d point) {
        lookAt(() -> point);
    }

    public void lookAt(Supplier<Translation2d> point) {
        faceHeading(() -> point.get().minus(swerveDrive.getEstimatedPose().getTranslation()).getAngle());
    }

    public void drive(Translation2d velocity) {
        driveTranslation(() -> velocity);
    }

    public void driveTranslation(Supplier<Translation2d> velocity) {
        setTranslationController(new TranslationController() {
            @Override
            public Translation2d getTranslationVelocity() {
                return velocity.get();
            }
        });
    }

    public void drive(ChassisSpeeds speeds) {
        driveSpeeds(() -> speeds);
    }

    public void driveSpeeds(Supplier<ChassisSpeeds> speeds) {
        SpeedsController controller = new SpeedsController(speeds);

        setTranslationController(controller);
        setRotationController(controller);
    }

    public void park() {
        setRotationController(null);
        setTranslationController(null);
        parked = true;
    }

    public void stop() {
        setRotationController(null);
        setTranslationController(null);
    }

    public void stopTranslation() {
        setTranslationController(null);
    }

    public void stopRotation() {
        setRotationController(null);
    }
    
    public boolean isStopped() {
        return rotationController == null && translationController == null;
    }

    public boolean isParked() {
        return parked;
    }

    public Command pathfindTo(Pose2d targetPose) {
        return pathfindTo(() -> targetPose);
    }

    public Command pathfindTo(Supplier<Pose2d> targetPose) {
        return new PathfindTo(targetPose, this);
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
            runOnce(() -> faceHeading(targetPose.getRotation()))
        );
    }

    private static class PathfindTo extends Command {
        private Command command;
        private ControllerDrive swerveController;
        private Supplier<Pose2d> poseSupplier;
      
        public PathfindTo(Supplier<Pose2d> poseSupplier, ControllerDrive swerveController) {
            this.poseSupplier = poseSupplier;
            this.swerveController = swerveController;

            swerveController.runningCommand = this;
        }
      
        @Override
        public void initialize() {
            Pose2d pose = poseSupplier.get();

            command = swerveController.simplePathfind(pose);

            command.schedule();
        }

        @Override
        public void execute() {
        }

        @Override
        public void end(boolean interrupted) {
            command.cancel();

            swerveController.runningCommand = null;
        }

        @Override
        public boolean isFinished() {
            return command.isScheduled() && command.isFinished();
        }
    }

    public FieldElement getAutoPath() {
        return autoPath;
    }

    private interface Controller {
        // TODO: Add back in start() and end(), update PathfindTo to use a Controller
        // public default void start() {}
        public default void update() {}
        // public default void end() {}
    }

    private interface RotationController extends Controller {
        public Rotation2d getAngularVelocity();
    }

    private interface TranslationController extends Controller {
        public Translation2d getTranslationVelocity();
    }

    private class SpeedsController implements RotationController, TranslationController {
        private Supplier<ChassisSpeeds> speeds;

        public SpeedsController(Supplier<ChassisSpeeds> speeds) {
            this.speeds = speeds;
        }

        @Override
        public Rotation2d getAngularVelocity() {
            return Rotation2d.fromRadians(speeds.get().omegaRadiansPerSecond);
        }

        @Override
        public Translation2d getTranslationVelocity() {
            return new Translation2d(speeds.get().vxMetersPerSecond, speeds.get().vyMetersPerSecond);
        }
    }

    private void configurePathplanner() {
        AutoBuilder.configureHolonomic(
            this.swerveDrive::getEstimatedPose, // Robot pose supplier
            (Pose2d pose) -> {
                throw new UnsupportedOperationException("Resetting odometry is not supported. Do not give an auto path a starting pose.");
            }, // Method to reset odometry (will be called if your auto has a starting pose)
            this.swerveDrive::getEstimatedSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this.swerveDrive::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                swerveConfig.pathTranslationPID().toPathplannerPIDConstants(), // Translation PID constants
                swerveConfig.pathRotationPID().toPathplannerPIDConstants(), // Rotation PID constants
                swerveConfig.maxLinearWheelSpeed().in(MetersPerSecond), // Max module speed, in m/s
                swerveConfig.driveRadius().in(Meters),
                new ReplanningConfig()
            ),
            () -> false, // Should flip paths
            this // Reference to this subsystem to set requirements
        );

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            autoPath.setTargetPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            autoPath.setPath(poses);
        });
    }

    private static class AutoPath implements FieldElement {
        private Pose2d targetPose;
        private List<Pose2d> path;

        public void setTargetPose(Pose2d targetPose) {
            this.targetPose = targetPose;
        }

        public void setPath(List<Pose2d> path) {
            this.path = path;
        }

        @Override
        public void updateFieldElement(Field2d field) {
            if (targetPose != null) {
                field.getObject("Target Pose").setPose(targetPose);
            }

            if (path != null) {
                field.getObject("Active Path").setPoses(path);
            }
        }
    }
}