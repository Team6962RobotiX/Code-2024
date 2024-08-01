package frc.robot.subsystems.drive.alt;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.SWERVE_DRIVE;

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
    private PoseManager poseManager;
    private AHRS gyroscope;

    private SwerveDriveKinematics kinematics;

    private Translation2d[] moduleTranslations;

    private SwerveConfig configuration;

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

        driveManager = new DriveManager(config.equippedModules(), kinematics);
        poseManager = new PoseManager(kinematics, () -> gyroscope, () -> driveManager.getModulePositions());
        gyroscope = new AHRS();
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

        poseManager.update();
    }

    @Override
    public void updateFieldElement(Field2d field) {
        // Get the current estimated position of the robot
        Pose2d robotPose = poseManager.getEstimatedPosition();

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
            // This means that the swerve modules are positioned at the corners of the robot:
            //
            //     (front)
            //     0 --- 1
            //     |     |
            //     |     |
            //     |     |
            //     2 --- 3
            //
            // See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
            // for more information on the coordinate system
            Pose2d modulePose = new Pose2d(
                ((i & 2) == 0 ? 1 : -1) * SWERVE_DRIVE.WHEELBASE / 2.0,
                ((i & 1) == 0 ? 1 : -1) * SWERVE_DRIVE.TRACKWIDTH / 2.0,
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
}
