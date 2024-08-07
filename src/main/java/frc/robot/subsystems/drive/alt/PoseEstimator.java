package frc.robot.subsystems.drive.alt;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.util.software.CustomSwerveDrivePoseEstimator;

/**
 * A {@code PoseEstimator} is responsible for managing the swerve drive's pose
 * estimation. All calculations are encapsulated inside of this class, so we can
 * change the algorithm without affecting the rest of the code.<br><br>
 * 
 * Currently, this works in two stages: <ol>
 *   <li>Before the robot is enabled, the PoseEstimator will use the vision
 *   measurements and gyroscope data to estimate the robot's initial pose.</li>
 *   <li>Once the robot is enabled, the PoseEstimator will create a
 *   {@link CustomSwerveDrivePoseEstimator} to estimate the robot's pose in
 *   real-time based on gyroscope, odometry, and vision data.</li>
 * </ol>
 */
public class PoseEstimator {
    private CustomSwerveDrivePoseEstimator poseEstimator;

    private Pose2d initialPose;
    private double initialInaccuracy = 0.0;
    private final double initialInaccuracyThreshold = 0.1; // TODO: Tune this value

    private Pose2d livePose;

    private Gyroscope gyroscope;
    private SwerveDriveKinematics kinematics;
    private Supplier<SwerveModulePosition[]> modulePositionsSupplier;
    
    public PoseEstimator(
        SwerveDriveKinematics kinematics, Gyroscope gyroscope,
        Supplier<SwerveModulePosition[]> modulePositionsSupplier
    ) {
        this.gyroscope = gyroscope;
        this.kinematics = kinematics;
        this.modulePositionsSupplier = modulePositionsSupplier;

        if (RobotBase.isSimulation()) {
            initialPose = new Pose2d();
        }
    }

    public void update() {
        if (RobotState.isDisabled()) {
            Translation3d velocity = gyroscope.getVelocity();

            if (velocity != null) {
                initialInaccuracy += velocity.getNorm();
                initialInaccuracy += gyroscope.getRate().in(RotationsPerSecond);
            }
        } else if (poseEstimator == null) {
            poseEstimator = new CustomSwerveDrivePoseEstimator(
                kinematics,
                gyroscope.getCurrentHeading().times(-1),
                modulePositionsSupplier.get(),
                initialPose
            );

            livePose = poseEstimator.getEstimatedPosition();
        } else {
            poseEstimator.update(
                gyroscope.getCurrentHeading().times(-1),
                modulePositionsSupplier.get()
            );

            livePose = poseEstimator.getEstimatedPosition();
        }
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        if (RobotState.isDisabled()) {
            // TODO: Factor in vision timestamp into initial pose estimate
            // TODO: Weight more accurate vision measurements more heavily
            double previousFactor = Math.max(1 - initialInaccuracy / initialInaccuracyThreshold, 0) / 2;

            initialPose = initialPose.interpolate(pose, previousFactor);
        } else if (poseEstimator != null) {
            poseEstimator.addVisionMeasurement(pose, timestamp);
        }
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator == null ? initialPose : livePose;
    }

    public Pose2d getEstimatedPose(double timestamp) {
        return poseEstimator == null ? initialPose : poseEstimator.getEstimatedPosition(timestamp);
    }
}
