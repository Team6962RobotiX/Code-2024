package frc.robot.subsystems.drive.alt;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.subsystems.drive.alt.field.FieldElement;
import frc.robot.subsystems.drive.alt.pose.Gyroscope;
import frc.robot.subsystems.drive.alt.pose.PoseEstimator;

public abstract class SwerveDrive extends SubsystemBase implements FieldElement {
    private SwerveModuleState[] targetStates;

    private DriveManager driveManager;
    private PoseEstimator poseEstimator;
    private Gyroscope gyroscope;

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

        gyroscope = new Gyroscope();

        driveManager = new DriveManager(config, kinematics);
        poseEstimator = new PoseEstimator(kinematics, gyroscope, driveManager::getModulePositions);
    }

    public void drive(ChassisSpeeds speeds) {
        targetStates = kinematics.toSwerveModuleStates(speeds);
    }

    public void drive(SwerveModuleState[] states) {
        targetStates = states;
    }

    @Override
    public void periodic() {
        driveManager.drive(targetStates);
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

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModuleState[] getModuleStates() {
        return driveManager.getModuleStates();
    }

    public SwerveModulePosition[] getModulePositions() {
        return driveManager.getModulePositions();
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
