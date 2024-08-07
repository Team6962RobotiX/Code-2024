package frc.robot.subsystems.drive.alt;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.subsystems.drive.alt.field.FieldElement;

public abstract class SwerveDrive extends SubsystemBase implements FieldElement {
    // TODO: Improve design, move to seperate class
    private Supplier<Translation2d> translationSupplier;
    private boolean translationSupplierRepeats;
    private Supplier<Rotation2d> headingSupplier;
    private boolean headingSupplierRepeats;
    private Supplier<SwerveModuleState[]> statesSupplier;
    private boolean statesSupplierRepeats;
    private Supplier<ChassisSpeeds> speedsSupplier;
    private boolean speedsSupplierRepeats;

    private DriveManager driveManager;
    private PoseEstimator poseEstimator;
    private Gyroscope gyroscope;

    private SwerveDriveKinematics kinematics;

    private Translation2d[] moduleTranslations;

    private SwerveConfig configuration;

    private AutoPath autoPath = new AutoPath();

    public SwerveDrive(SwerveConfig config) {
        configuration = config;

        double wheelBase = configuration.wheelBase().in(Meters);
        double trackWidth = configuration.trackWidth().in(Meters);

        moduleTranslations = new Translation2d[] {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

        kinematics = new SwerveDriveKinematics(moduleTranslations);

        gyroscope = new Gyroscope();

        driveManager = new DriveManager(config, kinematics);
        poseEstimator = new PoseEstimator(kinematics, gyroscope, driveManager::getModulePositions);

        AutoBuilder.configureHolonomic(
            this::getEstimatedPose, // Robot pose supplier
            (Pose2d pose) -> {
                throw new UnsupportedOperationException("Resetting odometry is not supported. Do not give an auto path a starting pose.");
            }, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getEstimatedSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD), // Translation PID constants
                new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kD), // Rotation PID constants
                SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY, // Max module speed, in m/s
                SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
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

    public void drive(ChassisSpeeds speeds) {
        drive(speeds, false);
    }

    public void drive(ChassisSpeeds speeds, boolean repeating) {
        driveChassisSpeeds(() -> speeds, repeating);
    }

    public void drive(SwerveModuleState[] states) {
        drive(states, false);
    }

    public void drive(SwerveModuleState[] states, boolean repeating) {
        driveModuleStates(() -> states, repeating);
    }

    public void drive(Translation2d translation) {
        drive(translation, false);
    }

    public void drive(Translation2d translation, boolean repeating) {
        driveTranslation(() -> translation, repeating);
    }

    public void drive(Rotation2d heading) {
        drive(heading, false);
    }

    public void drive(Rotation2d heading, boolean repeating) {
        driveHeading(() -> heading, repeating);
    }

    public void driveModuleStates(Supplier<SwerveModuleState[]> statesSupplier, boolean repeating) {
        this.statesSupplier = statesSupplier;
        this.statesSupplierRepeats = repeating;
        headingSupplier = null;
        translationSupplier = null;
        speedsSupplier = null;
    }

    public void driveHeading(Supplier<Rotation2d> headingSupplier, boolean repeating) {
        this.headingSupplier = headingSupplier;
        this.headingSupplierRepeats = repeating;
        statesSupplier = null;
        speedsSupplier = null;
    }

    public void driveTranslation(Supplier<Translation2d> translationSupplier, boolean repeating) {
        this.translationSupplier = translationSupplier;
        this.translationSupplierRepeats = repeating;
        statesSupplier = null;
        speedsSupplier = null;
    }

    public void driveChassisSpeeds(Supplier<ChassisSpeeds> speedsSupplier, boolean repeating) {
        this.speedsSupplier = speedsSupplier;
        this.speedsSupplierRepeats = repeating;
        statesSupplier = null;
        translationSupplier = null;
        headingSupplier = null;
    }

    public void stop() {
        headingSupplier = null;
        translationSupplier = null;
        statesSupplier = null;
        speedsSupplier = null;
    }

    public void park() {
        drive(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45.0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45.0))
        });
    }

    @Override
    public void periodic() {
        boolean usingHeadingTranslation = headingSupplier != null || translationSupplier != null;
        boolean usingStates = statesSupplier != null;
        boolean usingSpeeds = speedsSupplier != null;

        if ((usingHeadingTranslation && usingStates) || (usingHeadingTranslation && usingSpeeds) || (usingStates && usingSpeeds)) {
            throw new IllegalStateException("Cannot use multiple types of drive suppliers at once");
        }

        if (usingHeadingTranslation) {
            Translation2d translation = translationSupplier == null ? null : translationSupplier.get();
            Rotation2d heading = headingSupplier == null ? null : headingSupplier.get();

            driveManager.drive(new ChassisSpeeds(
                translationSupplier == null ? 0.0 : translation.getX(),
                translationSupplier == null ? 0.0 : translation.getY(),
                headingSupplier == null ? 0.0 : heading.getRadians()
            ));
        } else if (usingStates) {
            driveManager.drive(statesSupplier.get());
        } else if (usingSpeeds) {
            driveManager.drive(speedsSupplier.get());
        } else {
            driveManager.stop();
        }

        if (headingSupplier != null && !headingSupplierRepeats) headingSupplier = null;
        if (translationSupplier != null && !translationSupplierRepeats) translationSupplier = null;
        if (statesSupplier != null && !statesSupplierRepeats) statesSupplier = null;
        if (speedsSupplier != null && !speedsSupplierRepeats) speedsSupplier = null;

        poseEstimator.update();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPose();
    }

    public Pose2d getEstimatedPose(double timestamp) {
        return poseEstimator.getEstimatedPose(timestamp);
    }

    public ChassisSpeeds getEstimatedSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            kinematics.toChassisSpeeds(driveManager.getModuleStates()),
            getEstimatedPose().getRotation()
        );
    }

    public Pose2d getFuturePose() {
        ChassisSpeeds speeds = getEstimatedSpeeds();
        Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        Translation2d futurePosition = getEstimatedPose().getTranslation();

        futurePosition = futurePosition.plus(velocity.times(velocity.getNorm()).div(2 * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION));

        return new Pose2d(futurePosition, getEstimatedPose().getRotation());
    }

    @Override
    public void updateFieldElement(Field2d field) {
        // Get the current estimated position of the robot
        Pose2d robotPose = poseEstimator.getEstimatedPose();

        // Update the field with the new robot pose
        field.getObject("Robot").setPose(robotPose);

        // Get the current orientations of the swerve modules
        SwerveModulePosition[] modulePositions = driveManager.getModulePositions();

        // Create an array to hold the absolute poses of the swerve modules
        Pose2d[] modulePoses = new Pose2d[modulePositions.length];
        
        for (int i = 0; i < modulePositions.length; i++) {
            // This calculates the pose of the swerve module relative to the robot. The bitwise operations
            // represent the following logic:
            //
            //  * If the first bit is set (modules 0 or 2), the module is on the front end of the robot (+X)
            //  * If the second bit is set (modules 2 or 3), the module is on the left side of the robot (+Y)
            //
            // This means that the swerve modules are positioned at these relative positions to the robot:
            //
            //     (front)
            //     0 --- 1
            //     |     |
            //     |     |
            //     2 --- 3
            //
            // See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
            // for more information on the coordinate system
            Pose2d modulePose = new Pose2d(
                ((i & 2) == 0 ? 1 : -1) * configuration.wheelBase().in(Meters) / 2.0,
                ((i & 1) == 0 ? 1 : -1) * configuration.trackWidth().in(Meters) / 2.0,
                modulePositions[i].angle
            );

            // Add the module pose to the robot pose to get the absolute pose of the module
            modulePoses[i] = robotPose.plus(new Transform2d(modulePose.getTranslation(), modulePose.getRotation()));
        }

        // Update the field with the new module positions
        field.getObject("Swerve Modules").setPoses(modulePoses);
    }

    public Measure<Velocity<Distance>> getWheelVelocity(double powerFraction) {
        return configuration.maxLinearWheelSpeed().times(powerFraction);
    }

    public void lookAt(Supplier<Translation2d> point, Rotation2d rotationOffset, boolean repeating) {
        driveHeading(() -> point.get().minus(getEstimatedPose().getTranslation()).getAngle().plus(rotationOffset), repeating);
    }

    public Command createLookAtCommand(Supplier<Translation2d> point, Rotation2d rotationOffset) {
        return new LookAtCommand(point, rotationOffset, this);
    }

    // TODO: Improve design of LookAtCommand to not need to be inside the SwerveDrive class
    private static class LookAtCommand extends Command {
        private SwerveDrive swerveDrive;
        private Supplier<Rotation2d> runningSupplier;

        public LookAtCommand(Supplier<Translation2d> point, Rotation2d rotationOffset, SwerveDrive swerveDrive) {
            this.swerveDrive = swerveDrive;

            runningSupplier = () -> point.get().minus(swerveDrive.getEstimatedPose().getTranslation()).getAngle().plus(rotationOffset);

            swerveDrive.driveHeading(runningSupplier, true);
        }

        @Override
        public void execute() {
        }

        @Override
        public boolean isFinished() {
            return runningSupplier == swerveDrive.headingSupplier;
        }
    }

    private Command simplePathfind(Pose2d pose) {
        Rotation2d angle = pose.getTranslation().minus(getEstimatedPose().getTranslation()).getAngle();

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(getEstimatedPose().getTranslation(), angle),
            new Pose2d(pose.getTranslation(), angle)
        );

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
            new GoalEndState(
                0.0,
                pose.getRotation(),
                true
            )
        );

        return Commands.sequence(
            AutoBuilder.followPath(path),
            runOnce(() -> drive(pose.getRotation()))
        );
    }

    public Command pathfindTo(Pose2d pose) {
        return new PathfindTo(() -> pose, this);
    }

    public Command pathfindTo(Supplier<Pose2d> poseSupplier) {
        return new PathfindTo(poseSupplier, this);
    }

    private static class PathfindTo extends Command {
        private Command command;
        private SwerveDrive swerveDrive;
        private Supplier<Pose2d> poseSupplier;
      
        public PathfindTo(Supplier<Pose2d> poseSupplier, SwerveDrive swerveDrive) {
            this.poseSupplier = poseSupplier;
            this.swerveDrive = swerveDrive;
        }
      
        @Override
        public void initialize() {
            Pose2d pose = poseSupplier.get();

            command = swerveDrive.simplePathfind(pose);

            command.schedule();
        }

        @Override
        public void execute() {
        }

        @Override
        public void end(boolean interrupted) {
            command.cancel();
        }
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
                field.getObject("PathfindTo Target Pose").setPose(targetPose);
            }

            if (path != null) {
                field.getObject("PathfindTo Active Path").setPoses(path);
            }
        }
    }

    /**
     * Converts the speed of a wheel moving to the angular velocity of the robot as if it's
     * rotating in place
     * @param wheelSpeed Drive velocity in m/s
     * @return Equivalent rotational velocity in rad/s
     * @see #toWheelSpeed(double)
     */
    public double toAngularSpeed(double wheelSpeed) {
        return wheelSpeed / configuration.driveRadius().in(Meters);
    }

    public Measure<Velocity<Angle>> toAngularSpeed(Measure<Velocity<Distance>> wheelSpeed) {
        return RadiansPerSecond.of(toAngularSpeed(wheelSpeed.in(MetersPerSecond)));
    }

    /**
     * Converts the angular velocity of the robot to the speed of a wheel moving as if the
     * robot is rotating in place
     * @param angularVelocity Rotational velocity in rad/s
     * @return Equivalent drive velocity in m/s
     * @see #toAngularSpeed(double)
     */
    public double toWheelSpeed(double angularVelocity) {
        return angularVelocity * configuration.driveRadius().in(Meters);
    }

    public Measure<Velocity<Distance>> toWheelSpeed(Measure<Velocity<Angle>> angularVelocity) {
        return MetersPerSecond.of(toWheelSpeed(angularVelocity.in(RadiansPerSecond)));
    }
}
