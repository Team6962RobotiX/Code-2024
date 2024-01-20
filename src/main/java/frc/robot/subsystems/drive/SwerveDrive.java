// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.util.Logging.Logger;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule[] modules = new SwerveModule[4];
  private AHRS gyro;
  private SwerveDriveKinematics kinematics = getKinematics();
  private SwerveDrivePoseEstimator poseEstimator;
  private Field2d field = new Field2d();
  public Rotation2d heading = Rotation2d.fromDegrees(0);

  private ProfiledPIDController rotateController = new ProfiledPIDController(
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kP,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kI,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kD,
    new Constraints(
      SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY,
      SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_ACCELERATION
    )
  );

  double speedCap = 1.0;

  public SwerveDrive() {
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      if (RobotBase.isSimulation()) {
        modules[i] = new SwerveModuleSim(i);
      } else {
        modules[i] = new SwerveModule(i);
      }
    }
    
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePositions(), SWERVE_DRIVE.STARTING_POSE);
    rotateController.enableContinuousInput(-Math.PI, Math.PI);

    try {
      gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), false);
    }
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        setHeading(new Rotation2d());
      } catch (Exception e) {}
    }).start();
    
    SmartDashboard.putData("Field", field);
    Logger.autoLog("SwerveDrive/pose", () -> this.getPose());

    // AutoBuilder.configureHolonomic(
    //   this::getPose, // Robot pose supplier
    //   this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //   this::getTargetChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //   this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //   new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //     new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD),
    //     new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kP,    SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kI,    SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kD),
    //     SWERVE_DRIVE.AUTONOMOUS.MAX_LINEAR_VELOCITY,
    //     SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS,
    //     new ReplanningConfig() // Default path replanning config. See the API for the options here
    //   ),
    //   this // Reference to this subsystem to set requirements
    // );
  }

  @Override
  public void periodic() {
    // Update heading
    if (gyro.isConnected() && !RobotBase.isSimulation()) {
      heading = gyro.getRotation2d();
    } else {
      heading = heading.plus(new Rotation2d(getMeasuredChassisSpeeds().omegaRadiansPerSecond * 0.02));
    }
    heading = Rotation2d.fromRadians(MathUtil.angleModulus(heading.getRadians()));
    
    // Update pose
    poseEstimator.update(getHeading(), getModulePositions());

    // Update field
    FieldObject2d modulesObject = field.getObject("Swerve Modules");
    Pose2d[] modulePoses = new Pose2d[4];
    for (SwerveModule module : modules) {
      modulePoses[module.id] = module.getPose(getPose());
    }
    modulesObject.setPoses(modulePoses);
    field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(getCurrent()));
  }

  public void drive(ChassisSpeeds speeds) {
    driveModules(kinematics.toSwerveModuleStates(speeds));
  }

  public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
    /*double norm = Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2));
    if (norm > speedCap) {
      xVelocity *= (speedCap/norm);
      yVelocity *= (speedCap/norm);
    }*/
    //System.out.println(String.format("%e, %e, %e", xVelocity, yVelocity, angularVelocity));
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, getHeading()));
  }

  public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
    drive(ChassisSpeeds.fromRobotRelativeSpeeds(xVelocity, yVelocity, angularVelocity, getHeading()));
  }

  /**
   * Tells the modules what speed and direction to run at
   * @param moduleStates The target module states
   */
  private void driveModules(SwerveModuleState[] moduleStates) {
    ChassisSpeeds velocity = kinematics.toChassisSpeeds(moduleStates);
    
    if (toLinear(Math.abs(velocity.omegaRadiansPerSecond)) < SWERVE_DRIVE.VELOCITY_DEADBAND && toLinear(Math.abs(getMeasuredChassisSpeeds().omegaRadiansPerSecond)) < SWERVE_DRIVE.VELOCITY_DEADBAND) {
      velocity.omegaRadiansPerSecond += rotateController.calculate(getHeading().getRadians());
    } else {
      rotateController.setGoal(getHeading().getRadians());
      rotateController.calculate(getHeading().getRadians());
    }

    velocity = ChassisSpeeds.discretize(velocity, SWERVE_DRIVE.DISCRETIZED_TIME_STEP);
    
    moduleStates = kinematics.toSwerveModuleStates(velocity);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveModule.calcWheelVelocity(SWERVE_DRIVE.MOTOR_POWER_HARD_CAP));
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      modules[i].setTargetState(moduleStates[i]);
    }
  }

  public void setTargetHeading(Rotation2d heading) {
    rotateController.setGoal(heading.getRadians());
  }

  /**
   * This creates an "X" pattern with the wheels which makes the robot very hard to move
   */
  public void parkModules() {
    modules[0].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[1].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[2].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[3].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
  }

  /**
   * Resets the odometer position to a given position
   * @param pose Desired position
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
  }

  public void addVisionMeasurement(Pose2d visionMeasurement) {
    poseEstimator.addVisionMeasurement(visionMeasurement, Timer.getFPGATimestamp());
  }

  /**
   * Stops all motors on all modules
   */
  public void stopModules() {
    for (SwerveModule module : modules)
      module.stop();
  }

  /**
   * @return Target chassis x, y, and rotational velocity
   */
  public ChassisSpeeds getTargetChassisSpeeds() {
    return kinematics.toChassisSpeeds(getTargetModuleStates());
  }

  /**
   * @return Measured chassis x, y, and rotational velocity
   */
  public ChassisSpeeds getMeasuredChassisSpeeds() {
    return kinematics.toChassisSpeeds(getMeasuredModuleStates());
  }

  public ChassisSpeeds getDrivenChassisSpeeds() {
    return kinematics.toChassisSpeeds(getDrivenModuleStates());
  }

  public void setSpeedCap(double cap) {
    speedCap = cap;
  }

  public double getSpeedCap() {
    return speedCap;
  } 

  /**
   * @return Measured module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      positions[i] = modules[i].getModulePosition();
    }
    return positions;
  }

  /**
   * @return Target module states (speed and direction)
   */
  public SwerveModuleState[] getTargetModuleStates() {
    SwerveModuleState[] targetStates = new SwerveModuleState[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      targetStates[i] = modules[i].getTargetState();
    }
    return targetStates;
  }

  /**
   * @return Measured module states (speed and direction)
   */
  public SwerveModuleState[] getMeasuredModuleStates() {
    SwerveModuleState[] measuredStates = new SwerveModuleState[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      measuredStates[i] = modules[i].getMeasuredState();
    }
    return measuredStates;
  }

  /**
   * @return Measured module states (speed and direction)
   */
  public SwerveModuleState[] getDrivenModuleStates() {
    SwerveModuleState[] measuredStates = new SwerveModuleState[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      measuredStates[i] = modules[i].getDrivenState();
    }
    return measuredStates;
  }

  /**
   * @return Total current through all modules
   */
  public double getCurrent() {
    double totalCurrent = 0.0;
    for (SwerveModule module : modules)
      totalCurrent += module.getTotalCurrent();
    return totalCurrent;
  }

  /**
   * @return NavX AHRS IMU Gyro
   */
  public AHRS getGyro() {
    return gyro;
  }

  /**
   * @return Resets gyro heading
   */
  public void setHeading(Rotation2d newHeading) {
    heading = newHeading;
    gyro.reset();
    gyro.setAngleAdjustment(newHeading.getDegrees());
  }

  /**
   * @return Returns gyro heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return heading;
  }

  /**
   * @return Get pose on the field from odometer as a Pose2d
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @return Field2d object for SmartDashboard widget.
   */
  public Field2d getField() {
    return field;
  }

  /**
   * @param wheelSpeed drive velocity in m/s
   * @return rotational velocity in rad/s
   */
  public static double toAngular(double wheelSpeed) {
    return wheelSpeed / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS;
  }

  /**
   * @param angularVelocity rotational velocity in rad/s
   * @return drive velocity in m/s
   */
  public static double toLinear(double angularVelocity) {
    return angularVelocity * SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS;
  }


  public static SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(
      new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
      new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0), 
      new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
      new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0));
  }

  /*public Command followPathCommand(List<Pose2d> poses) {
    PathConstraints constraints = new PathConstraints(
      SWERVE_DRIVE.AUTONOMOUS.MAX_LINEAR_VELOCITY,
      SWERVE_DRIVE.AUTONOMOUS.MAX_LINEAR_ACCELERATION,
      SWERVE_DRIVE.AUTONOMOUS.MAX_ANGULAR_VELOCITY,
      SWERVE_DRIVE.AUTONOMOUS.MAX_ANGULAR_ACCELERATION
    );
    // List<Pose2d> points = new ArrayList<Pose2d>();
    // for (int i = 0; i < poses.size(); i++) {
      // Translation2d currentPosition = poses.get(i).getTranslation();
      // Translation2d prevPosition = currentPosition;
      // Translation2d nextPosition = currentPosition;
      // if (i > 0) {
      //   prevPosition = poses.get(i - 1).getTranslation();
      // }
      // if (i < poses.size() - 1) {
      //   nextPosition = poses.get(i + 1).getTranslation();
      // }
      // Rotation2d angle = nextPosition.minus(currentPosition).minus(prevPosition.minus(currentPosition)).getAngle();
      // points.add(
      //   new Pose2d(currentPosition, new Rotation2d())
      // );
    // }
    List<PathPoint> points = new ArrayList<PathPoint>();
    for (Pose2d pose: poses) {
      // points.add(new PathPoint(pose.getTranslation(), new Rotation2d()));
    }
    FieldObject2d pathObject = field.getObject("PathPoints");
    pathObject.setPoses(poses);
    
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);

    return AutoBuilder.followPathWithEvents(
      new PathPlannerPath(
        bezierPoints,
        constraints,
        new GoalEndState(0.0, new Rotation2d())
      )
      // PathPlannerPath.fromPathPoints(points, constraints, new GoalEndState(0.0, new Rotation2d()))
    );

  //   FieldObject2d pathObject = field.getObject("PathPoints");
  //   List<Pose2d> pathPoses = new ArrayList<Pose2d>();
  //   for (PathPoint point : points) {
      
  //     pathPoses.add(new Pose2d(point.position, point.holonomicRotation));
  //   }
  //   pathObject.setPoses(pathPoses);

  //   return AutoBuilder.pathfindThenFollowPath(
  //     PathPlannerPath.fromPathPoints(
  //       points, 
  //       new PathConstraints(
  //         SWERVE_DRIVE.AUTONOMOUS_VELOCITY,
  //         SWERVE_DRIVE.AUTONOMOUS_ACCELERATION,
  //         wheelVelocityToRotationalVelocity(SWERVE_DRIVE.AUTONOMOUS_VELOCITY),
  //         wheelVelocityToRotationalVelocity(SWERVE_DRIVE.AUTONOMOUS_ACCELERATION)
  //       ),
  //       new GoalEndState(0.0, new Rotation2d())
  //     ),
  //   new PathConstraints(
  //     SWERVE_DRIVE.AUTONOMOUS_VELOCITY,
  //     SWERVE_DRIVE.AUTONOMOUS_ACCELERATION,
  //     wheelVelocityToRotationalVelocity(SWERVE_DRIVE.AUTONOMOUS_VELOCITY),
  //     wheelVelocityToRotationalVelocity(SWERVE_DRIVE.AUTONOMOUS_ACCELERATION)
  //   )
  // );


    // FieldObject2d pathObject = field.getObject("PathPoints");
    // List<Pose2d> pathPoses = new ArrayList<Pose2d>();
    // SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    // for (Pose2d pose : poses) {
    //   commandGroup.addCommands(
    //     AutoBuilder.pathfindToPose(
    //       pose,
    //       new PathConstraints(
    //         SWERVE_DRIVE.AUTONOMOUS_VELOCITY,
    //         SWERVE_DRIVE.AUTONOMOUS_ACCELERATION,
    //         wheelVelocityToRotationalVelocity(SWERVE_DRIVE.AUTONOMOUS_VELOCITY),
    //         wheelVelocityToRotationalVelocity(SWERVE_DRIVE.AUTONOMOUS_ACCELERATION)
    //       )
    //     )
    //   );
    //   pathPoses.add(pose);
    // }
    // pathObject.setPoses(pathPoses);
    
    // return commandGroup;

  }

  // Assuming this is a method in your drive subsystem
  public Command followPathCommand(PathPlannerPath path) {
    // return AutoBuilder.pathfindThenFollowPath(path, 
    //   new PathConstraints(
    //     SWERVE_DRIVE.AUTONOMOUS_VELOCITY,
    //     SWERVE_DRIVE.AUTONOMOUS_ACCELERATION,
    //     calcAngularVelocity(SWERVE_DRIVE.AUTONOMOUS_VELOCITY),
    //     calcAngularVelocity(SWERVE_DRIVE.AUTONOMOUS_ACCELERATION)
    //   ), 0.0);
    return AutoBuilder.followPathWithEvents(path);
  }

  public Command followAutoPathCommand(String autoName) {
    return new PathPlannerAuto(autoName);
  }*/
}