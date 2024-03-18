package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.software.Logging.Logger;

public class CollisionDetector extends SubsystemBase {
  public Translation2d lastAcceleration = new Translation2d();
  public NetworkTableEntry collisionEntry;
  public Translation2d jerk = new Translation2d();
  public boolean collisionDetected = false;
  public Debouncer collisionDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  public double collisionThreshold = 200.0;

  public CollisionDetector() {
    collisionEntry = NetworkTableInstance.getDefault().getEntry("collisionCount");
    collisionEntry.setPersistent();

    Logger.autoLog(this, "Jerk", () -> jerk.getNorm());
  }

  @Override
  public void periodic() {
    if (collisionEntry == null) return;

    Logger.log("Jerk", jerk.getNorm());
    Logger.log("Collisions", collisionEntry.getInteger(0));


    AHRS gyro = SwerveDrive.getGyro();
    Translation2d acceleration = new Translation2d(gyro.getWorldLinearAccelX(), gyro.getWorldLinearAccelY());
    jerk = acceleration.minus(lastAcceleration).div(Robot.getLoopTime());
    lastAcceleration = new Translation2d(acceleration.getX(), acceleration.getY());
    boolean newCollisionDetected = collisionDebouncer.calculate(jerk.getNorm() > collisionThreshold && jerk.getNorm() < 2000);
     if (!collisionDetected && newCollisionDetected) {
      System.out.println("Collision detected");
      collisionEntry.setNumber(collisionEntry.getNumber(0).intValue() + 1);
     }
    collisionDetected = newCollisionDetected;
  }
}
