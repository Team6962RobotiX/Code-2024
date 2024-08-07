package frc.robot.subsystems.drive.alt;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * {@code Gyroscope} is a wrapper for {@code AHRS} that adds the abilities to:
 * <ul>
 *   <li>Reset the heading to a specific angle, for when the robot discovers
 *   the field orientation.</li>
 *   <li>Use odometry data instead of gyroscope data for the heading during
 *   simulation or when the gyroscope is not connected.</li>
 * </ul>
 */
public class Gyroscope extends SubsystemBase {
    private AHRS gyroscope;
    private Rotation2d heading = Rotation2d.fromDegrees(0);
    private Rotation2d offset = Rotation2d.fromDegrees(0);

    /**
     * Creates a new Gyroscope.
     */
    public Gyroscope() {
        if (RobotBase.isReal()) {
            try {
                gyroscope = new AHRS(SPI.Port.kMXP);
            } catch (RuntimeException ex) {
                DriverStation.reportError(
                    "Error instantiating navX-MXP:  " + ex.getMessage(),
                    true
                );
            }

            new Thread(() -> {
                try {
                    Thread.sleep(1000);

                    resetHeading();
                } catch (Exception e) {}
            }).start();
        }
    }

    /**
     * Gets the absolute heading of the robot, without the offset.
     */
    private Rotation2d getAbsoluteHeading() {
        return heading;
    }

    /**
     * Converts a offset heading to an absolute heading.
     */
    private Rotation2d toAbsolute(Rotation2d offsetHeading) {
        return offsetHeading.minus(offset);
    }

    /**
     * Specifies a specific offset heading to be considered as 0 degrees.
     */
    public void resetHeading(Rotation2d offsetHeading) {
        offset = toAbsolute(offsetHeading).times(-1);
    }

    /**
     * Sets the current heading to be considered 0 degrees.
     */
    public void resetHeading() {
        resetHeading(getCurrentHeading());
    }

    /**
     * Returns the current heading of the robot.
     */
    public Rotation2d getCurrentHeading() {
        return getAbsoluteHeading().plus(offset);
    }

    /**
     * Applies an odometry update to the gyroscope heading. This has effect if
     * the gyroscope is not connected or if the robot is in simulation.
     */
    public void applyOdometryUpdate(Rotation2d headingChange) {
        if (gyroscope == null || !gyroscope.isConnected()) {
            heading = heading.plus(headingChange);
        }
    }

    /**
     * Returns whether the Gyroscope object is using odometry data instead of
     * actual gyroscope data.
     */
    public boolean isUsingOdometry() {
        return gyroscope == null || !gyroscope.isConnected();
    }

    /**
     * Returns the current velocity of the gyroscope in the robot-relative
     * x, y, and z axes. When the gyroscope is not connected or the robot
     * is in simulation, this method returns null.
     * 
     * TODO: Add support for odometry-based velocity.
     */
    public Translation3d getVelocity() {
        if (gyroscope != null && gyroscope.isConnected()) {
            return new Translation3d(
                gyroscope.getVelocityX(),
                gyroscope.getVelocityY(),
                gyroscope.getVelocityZ()
            );
        } else {
            return null;
        }
    }

    /**
     * Returns the current rate of rotation of the gyroscope. When the gyroscope
     * is not connected or the robot is in simulation, this method returns null.
     * 
     * TODO: Add support for odometry-based rate.
     */
    public Measure<Velocity<Angle>> getRate() {
        if (gyroscope != null && gyroscope.isConnected()) {
            return DegreesPerSecond.of(gyroscope.getRate());
        } else {
            return null;
        }
    }

    @Override
    public void periodic() {
        if (gyroscope != null && gyroscope.isConnected()) {
            heading = gyroscope.getRotation2d();
        }
    }
}