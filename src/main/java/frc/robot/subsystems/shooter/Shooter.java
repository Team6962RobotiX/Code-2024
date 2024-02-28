// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.SHOOTER_PIVOT;
import frc.robot.Robot;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Field;
import frc.robot.Constants.Preferences;
import frc.robot.subsystems.drive.SwerveDrive;

public class Shooter extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private ShooterWheels shooterWheels;
  private ShooterPivot shooterPivot;
  private FeedWheels feedWheels;

  private double headingVelocity;
  private Rotation2d targetHeading = new Rotation2d();

  public static enum State {
    IN,
    AIM,
    SHOOT,
  }

  public Shooter(SwerveDrive swerveDrive) {
    this.shooterWheels = new ShooterWheels();
    this.shooterPivot = new ShooterPivot();
    this.swerveDrive = swerveDrive;
    this.feedWheels = new FeedWheels();
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
  }

  public Command setState(State state) {
    switch(state) {
      case IN:
        return Commands.sequence(
          feedWheels.setState(FeedWheels.State.IN)
        );
      case AIM:
        return Commands.parallel(
          aim(Field.SPEAKER),
          shooterWheels.setTargetVelocity(Preferences.SHOOTER_WHEELS.TARGET_SPEED)
        );
      case SHOOT:
        return setState(State.AIM).until(() -> getShotChance() == 1.0)
          .andThen(feedWheels.setState(FeedWheels.State.IN).withTimeout(1.0));
    }
    return null;
  }

  public ShooterWheels getWheels() {
    return shooterWheels;
  }

  public FeedWheels getFeedWheels() {
    return feedWheels;
  }

  public ShooterPivot getShooterPivot() {
    return shooterPivot;
  }

  public Command aim(Translation3d point) {
    // Calculate point to aim towards, accounting for current velocity
    return new RunCommand(() -> {
      Translation3d velocityCompensatedPoint = ShooterMath.calcVelocityCompensatedPoint(
        point,
        swerveDrive.getPose(),
        swerveDrive.getFieldVelocity(),
        shooterWheels.getVelocity(),
        shooterPivot.getPosition()
      );

      swerveDrive.facePointBackwards(velocityCompensatedPoint.toTranslation2d());
      
      shooterPivot.setTargetAngle(ShooterMath.calcPivotAngle(
        velocityCompensatedPoint,
        swerveDrive.getPose(),
        shooterWheels.getVelocity()
      ));
    });
  }

  public boolean doneMoving() {
    return shooterPivot.doneMoving();
  }

  public double getShotChance() {
    return ShooterMath.calcShotChance(
      Field.SPEAKER,
      swerveDrive.getPose(),
      swerveDrive.getFieldVelocity(),
      shooterPivot.getPosition(),
      shooterWheels.getVelocity()
    );
  }

  @Override
  public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
  }

  public void orientToPointDelayCompensated(Translation3d pointToAimTo) {
    Rotation2d newTargetHeading = pointToAimTo.toTranslation2d().minus(swerveDrive.getPose().getTranslation()).getAngle();
    headingVelocity = newTargetHeading.minus(targetHeading).getRadians() / 0.02;
    targetHeading = newTargetHeading;
    swerveDrive.setTargetHeading(targetHeading.plus(Rotation2d.fromRadians(headingVelocity * SHOOTER_PIVOT.ROTATION_DELAY)).plus(Rotation2d.fromDegrees(180.0)));
  }

  public ShooterPivot getPivot() {
    return shooterPivot;
  }
}
