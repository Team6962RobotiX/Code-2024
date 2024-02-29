package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.software.Logging.Logger;

public class CollisionDetector extends SubsystemBase {
  public Translation2d lastAcceleration = new Translation2d();
  public NetworkTableEntry collisionEntry = NetworkTableInstance.getDefault().getEntry("collisionCount");
  public Translation2d jerk = new Translation2d();

  public CollisionDetector() {
    collisionEntry.setPersistent();
    Logger.autoLog(this, "Jerk", () -> jerk.getNorm());
  }

  @Override
  public void periodic() {
    AHRS gyro = SwerveDrive.getGyro();
    Translation2d acceleration = new Translation2d(gyro.getWorldLinearAccelX(), gyro.getWorldLinearAccelY());
    lastAcceleration = new Translation2d(acceleration.getX(), acceleration.getY());
    jerk = acceleration.minus(lastAcceleration).div(Robot.getLoopTime());
    if (jerk.getNorm() > 25.0) {
      System.out.println("Collision detected");
      collisionEntry.setNumber(collisionEntry.getNumber(0).intValue() + 1);
    }
  }
}
