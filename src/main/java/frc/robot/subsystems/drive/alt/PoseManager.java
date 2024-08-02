package frc.robot.subsystems.drive.alt;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.util.software.CustomSwerveDrivePoseEstimator;

/**
 * A {@code PoseManager} is responsible for managing the swerve drive's pose estimation.
 * All calculations are encapsulated inside of this class, so we can change the algorithm
 * without affecting the rest of the code.<br><br>
 * 
 * Currently, this works in two stages: <ol>
 *   <li>Before the robot is enabled, the PoseManager will use the vision measurements and
 *   gyroscope data to estimate the robot's initial pose.</li>
 *   <li>Once the robot is enabled, the PoseManager will create a {@link SwerveDrivePoseEstimator}
 *   to estimate the robot's pose in real-time based on gyroscope, odometry, and vision data.</li>
 * </ol>
 */
public class PoseManager {
    private CustomSwerveDrivePoseEstimator poseEstimator;

    private Pose2d initialPose;
    private double initialInaccuracy = 0.0;
    private final double initialInaccuracyThreshold = 0.1; // TODO: Tune this value

    private Pose2d livePose;

    private Supplier<AHRS> gyroscopeSupplier;
    private SwerveDriveKinematics kinematics;
    private Supplier<SwerveModulePosition[]> modulePositionsSupplier;
    
    public PoseManager(SwerveDriveKinematics kinematics, Supplier<AHRS> gyroscopeSupplier, Supplier<SwerveModulePosition[]> modulePositionsSupplier) {
        this.gyroscopeSupplier = gyroscopeSupplier;
        this.kinematics = kinematics;
        this.modulePositionsSupplier = modulePositionsSupplier;
    }

    public void update() {
        AHRS gyroscope = gyroscopeSupplier.get();

        if (RobotState.isDisabled()) {
            if (gyroscope != null) {
                initialInaccuracy += gyroscope.getVelocityX() + gyroscope.getVelocityY() + gyroscope.getVelocityZ();
                initialInaccuracy += Rotation2d.fromDegrees(gyroscope.getRate()).getRotations();
            }
        } else if (poseEstimator == null) {
            poseEstimator = new CustomSwerveDrivePoseEstimator(
                kinematics,
                Rotation2d.fromDegrees(-gyroscope.getAngle()),
                modulePositionsSupplier.get(),
                initialPose
            );
        } else {
            poseEstimator.update(
                Rotation2d.fromDegrees(-gyroscope.getAngle()),
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
