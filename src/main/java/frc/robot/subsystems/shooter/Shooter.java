// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.SHOOTER;
import frc.robot.Constants.SHOOTER.PIVOT;
import frc.robot.Field;
import frc.robot.Presets;
import frc.robot.subsystems.amp.AmpWheels;
import frc.robot.subsystems.drive.SwerveDrive;

public class Shooter extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private ShooterWheels shooterWheels;
  private ShooterPivot shooterPivot;
  private FeedWheels feedWheels;

  private Mechanism2d mechanism = new Mechanism2d(0, 0);
  private MechanismRoot2d pivotMechanism = mechanism.getRoot("pivot", PIVOT.POSITION.getX(), PIVOT.POSITION.getZ());
  private MechanismLigament2d shooterMechanism = pivotMechanism.append(new MechanismLigament2d("shooter", PIVOT.CoM_DISTANCE, 0.0));

  private double headingVelocity;
  private Rotation2d targetHeading = new Rotation2d();

  public static enum State {
    IN,
    AIM,
    SHOOT,
    OFF
  }

  public Shooter(SwerveDrive swerveDrive) {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;

    this.shooterWheels = new ShooterWheels();
    this.shooterPivot = new ShooterPivot();
    this.swerveDrive = swerveDrive;
    this.feedWheels = new FeedWheels();

    SmartDashboard.putData("ShooterMechanism", mechanism);
  }


  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    shooterMechanism.setAngle(shooterPivot.getPosition());
  }

  public Command setState(State state) {
    switch(state) {
      case IN:
        return Commands.sequence( 
          // shooterPivot.setTargetAngle(Presets.SHOOTER.PIVOT.INTAKE_ANGLE),
          // shooterWheels.setTargetVelocity(0.0),
          // Commands.waitUntil(() -> shooterPivot.doneMoving()),
          feedWheels.setState(FeedWheels.State.IN)
        );
      case AIM:
        return Commands.sequence( 
          aim(Field.SPEAKER),
          shooterWheels.setTargetVelocity(0.0),
          feedWheels.setState(FeedWheels.State.OFF)
        );
      case SHOOT:
        return Commands.sequence( 
          // setState(State.AIM),
          shooterWheels.setTargetVelocity(Presets.SHOOTER.WHEELS.TARGET_SPEED),
          feedWheels.setState(FeedWheels.State.IN).onlyIf(() -> Math.abs(getShooterWheels().getVelocity()) > Presets.SHOOTER.WHEELS.TARGET_SPEED * 0.85),
          Commands.waitSeconds(1.0)
        );
      case OFF:
        return Commands.sequence( 
          shooterPivot.setTargetAngle(shooterPivot.getPosition()),
          shooterWheels.setTargetVelocity(0.0),
          feedWheels.setState(FeedWheels.State.OFF)
        );
    }
    return null;
  }

  public ShooterWheels getShooterWheels() {
    return shooterWheels;
  }

  public FeedWheels getFeedWheels() {
    return feedWheels;
  }

  public Command aim(Translation3d point) {
    // Calculate point to aim towards, accounting for current velocity
    Translation3d velocityCompensatedPoint = ShooterMath.calcVelocityCompensatedPoint(
      point,
      swerveDrive.getPose(),
      swerveDrive.getFieldVelocity(),
      shooterWheels.getVelocity()
    );

    shooterPivot.setTargetAngle(ShooterMath.calcPivotAngle(
      velocityCompensatedPoint,
      swerveDrive.getPose(),
      shooterWheels.getVelocity()
    ));

    return runOnce(() -> orientToPointDelayCompensated(velocityCompensatedPoint));
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

  public boolean hasJustReleaseddNote() {
    return feedWheels.hasJustReleaseddNote();
  }

  public boolean hasJustReceivedNote() {
    return feedWheels.hasJustReceivedNote();
  }

  @Override
  public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
  }

  public void orientToPointDelayCompensated(Translation3d pointToAimTo) {
    Rotation2d newTargetHeading = pointToAimTo.toTranslation2d().minus(swerveDrive.getPose().getTranslation()).getAngle();
    headingVelocity = newTargetHeading.minus(targetHeading).getRadians() / 0.02;
    targetHeading = newTargetHeading;
    // swerveDrive.setTargetHeading(targetHeading.plus(Rotation2d.fromRadians(headingVelocity * PIVOT.ROTATION_DELAY)));
  }
}
