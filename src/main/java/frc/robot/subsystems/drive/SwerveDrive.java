// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;
import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.LIMELIGHT;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.subsystems.vision.AprilTagPose;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;

/**
 * This class represents the subsystem for the swerve drive. It contains four
 * swerve module objects and a gyroscope object.
 */
public class SwerveDrive extends SubsystemBase {
  public SwerveModule[] modules = new SwerveModule[SWERVE_DRIVE.MODULE_COUNT];
  private AHRS gyro;

  private SwerveDriveKinematics kinematics = getKinematics();
  private SwerveDrivePoseEstimator poseEstimator;
  private Field2d field = new Field2d();
  private Rotation2d gyroHeading = Rotation2d.fromDegrees(0.0);
  private Rotation2d gyroOffset = SWERVE_DRIVE.STARTING_POSE.getRotation();

  private ChassisSpeeds drivenChassisSpeeds = new ChassisSpeeds();

  private PIDController alignmentController = new PIDController(
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kP,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kI,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kD
  );

  private boolean isAligning = false;
  private boolean parked = false;

  SWERVE_DRIVE.MODULE_CONFIG[] equippedModules;

  public SwerveDrive() {
    if (!ENABLED_SYSTEMS.ENABLE_DRIVE) return;

    // Create the serve module objects
    if (SWERVE_DRIVE.IS_PROTOTYPE_CHASSIS) {
      equippedModules = SWERVE_DRIVE.EQUIPPED_MODULES_PROTOTYPE;
    } else {
      equippedModules = SWERVE_DRIVE.EQUIPPED_MODULES_COMPETITION;
    }

    int corner = 0;
    for (SWERVE_DRIVE.MODULE_CONFIG config : equippedModules) {
      String name = SWERVE_DRIVE.MODULE_NAMES[corner];
      if (RobotBase.isSimulation()) {
        modules[corner] = new SwerveModuleSim(config, corner, name);
      } else {
        modules[corner] = new SwerveModule(config, corner, name);
      }
      corner++;
    }

    // Set up pose estimator and rotation controller
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      SWERVE_DRIVE.STARTING_POSE.getRotation(),
      getModulePositions(),
      SWERVE_DRIVE.STARTING_POSE,
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2)),
      VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(30))
    );

    alignmentController.enableContinuousInput(-Math.PI, Math.PI);
    alignmentController.setTolerance(SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.TOLERANCE);
    setTargetHeading(SWERVE_DRIVE.STARTING_POSE.getRotation());

    // If possible, connect to the gyroscope
    try {
      gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), false);
    }

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyroOffset = gyroOffset.minus(gyro.getRotation2d());
      } catch (Exception e) {}
    }).start();
    
    SmartDashboard.putData("Field", field);
    Logger.autoLog("SwerveDrive/pose", () -> this.getPose());
    Logger.autoLog("SwerveDrive/measuredHeading", () -> this.getHeading().getDegrees());
    Logger.autoLog("SwerveDrive/targetHeading", () -> Units.radiansToDegrees(alignmentController.getSetpoint()));
    Logger.autoLog("SwerveDrive/targetStates", this::getTargetModuleStates);
    Logger.autoLog("SwerveDrive/measuredStates", this::getMeasuredModuleStates);

    StatusChecks.addCheck("Gyro Connection", gyro::isConnected);

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getMeasuredChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD), // Translation PID constants
        new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kD), // Rotation PID constants
        SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY, // Max module speed, in m/s
        SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      this::shouldFlipPaths,
      this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_DRIVE) return;
    
    // Update current heading based on gyroscope or wheel speeds
    if (gyro.isConnected() && !RobotBase.isSimulation()) {
      gyroHeading = gyro.getRotation2d();
    } else {
      gyroHeading = gyroHeading.plus(new Rotation2d(getMeasuredChassisSpeeds().omegaRadiansPerSecond * 0.02));
    }

    // gyroHeading = Rotation2d.fromRadians(MathUtil.angleModulus(gyroHeading.getRadians()));
    System.out.println(getHeading());
    // Update pose based on measured heading and swerve module positions
    poseEstimator.update(gyroHeading.plus(gyroOffset), getModulePositions());
    AprilTagPose.injectVisionData(LIMELIGHT.APRILTAG_CAMERA_NAMES, this);    
    // Update field
    FieldObject2d modulesObject = field.getObject("Swerve Modules");

    // Update swerve module poses
    Pose2d[] modulePoses = new Pose2d[SWERVE_DRIVE.MODULE_COUNT];
    Pose2d robotPose = getPose();
    
    int i = 0;
    for (SwerveModule module : modules) {
      modulePoses[i] = module.getPose(robotPose);
      i++;
    }

    modulesObject.setPoses(modulePoses);

    // Update robot pose
    field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    // Simulate the correct voltage going into the roboRIO
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(getCurrent()));
  }

  /**
   * Drives the robot at a given field-relative velocity
   * @param xVelocity [meters / second] Positive x is away from your alliance wall
   * @param yVelocity [meters / second] Positive y is to your left when standing behind the alliance wall
   * @param angularVelocity [radians / second] Rotational velocity, positive spins counterclockwise
   */
  public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
    driveFieldRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
  }

  /**
   * 
   * Drives the robot at a given field-relative ChassisSpeeds
   * @param fieldRelativeSpeeds
   */
  private void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveAttainableSpeeds(fieldRelativeSpeeds);
  }

  /**
   * Drives the robot at a given robot-relative velocity
   * @param xVelocity [meters / second] Positive x is towards the robot's front
   * @param yVelocity [meters / second] Positive y is towards the robot's left
   * @param angularVelocity [radians / second] Rotational velocity, positive spins counterclockwise
   */
  public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
    driveRobotRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
  }

  /**
   * Drives the robot at a given robot-relative ChassisSpeeds
   * @param robotRelativeSpeeds
   */
  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    driveFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getAllianceAwareHeading()));
  }

  private void driveAttainableSpeeds(ChassisSpeeds fieldRelativeSpeeds) {    
    double targetAngularSpeed = toLinear((fieldRelativeSpeeds.omegaRadiansPerSecond));
    double alignmentAngularVelocity = alignmentController.calculate(getHeading().getRadians());

    if (isAligning) fieldRelativeSpeeds.omegaRadiansPerSecond += alignmentAngularVelocity;

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getAllianceAwareHeading()));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY);
    fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(moduleStates), getAllianceAwareHeading());

    fieldRelativeSpeeds = ChassisSpeeds.discretize(fieldRelativeSpeeds, SWERVE_DRIVE.DISCRETIZED_TIME_STEP);

    // Limit translational acceleration
    Translation2d targetLinearVelocity = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    Translation2d currentLinearVelocity = new Translation2d(drivenChassisSpeeds.vxMetersPerSecond, drivenChassisSpeeds.vyMetersPerSecond);
    Translation2d linearAcceleration = (targetLinearVelocity).minus(currentLinearVelocity).div(0.02);
    double linearForce = linearAcceleration.getNorm() * SWERVE_DRIVE.ROBOT_MASS;

     // Limit rotational acceleration
    double targetAngularVelocity = fieldRelativeSpeeds.omegaRadiansPerSecond;
    double currentAngularVelocity = drivenChassisSpeeds.omegaRadiansPerSecond;
    double angularAcceleration = (targetAngularVelocity - currentAngularVelocity) / 0.02;
    double angularForce = Math.abs((SWERVE_DRIVE.PHYSICS.ROTATIONAL_INERTIA * angularAcceleration) / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS);

    double frictionForce = 9.80 * SWERVE_DRIVE.ROBOT_MASS * SWERVE_DRIVE.FRICTION_COEFFICIENT;

    if (linearForce + angularForce > frictionForce) {
      double factor = (linearForce + angularForce) / frictionForce;
      linearAcceleration = linearAcceleration.div(factor);
      angularAcceleration /= factor;
    }

    Translation2d attainableLinearVelocity = currentLinearVelocity.plus(linearAcceleration.times(0.02));
    double attainableAngularVelocity = currentAngularVelocity + (angularAcceleration * 0.02);

    drivenChassisSpeeds = new ChassisSpeeds(attainableLinearVelocity.getX(), attainableLinearVelocity.getY(), attainableAngularVelocity);

    SwerveModuleState[] drivenModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(drivenChassisSpeeds, getAllianceAwareHeading()));

    boolean moving = false;
    for (SwerveModuleState moduleState : kinematics.toSwerveModuleStates(fieldRelativeSpeeds)) if (Math.abs(moduleState.speedMetersPerSecond) > SWERVE_DRIVE.VELOCITY_DEADBAND) moving = true;
    for (SwerveModuleState moduleState : drivenModuleStates) if (Math.abs(moduleState.speedMetersPerSecond) > SWERVE_DRIVE.VELOCITY_DEADBAND) moving = true;
    if (!parked) {
      for (SwerveModuleState moduleState : getMeasuredModuleStates()) if (Math.abs(moduleState.speedMetersPerSecond) > SWERVE_DRIVE.VELOCITY_DEADBAND) moving = true;
    }
    parked = false;
    if (!moving && !isAligning) {
      parkModules();
      return;
    }

    if (alignmentAngularVelocity == 0.0) isAligning = false;

    driveModules(drivenModuleStates);
  }
  
  private void driveModules(SwerveModuleState[] moduleStates) {
    // Drive the swerve modules at the calculated speeds
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      modules[i].setTargetState(moduleStates[i]);
    }
  }

  /**
   * 
   * @param heading 
   */
  public void setTargetHeading(Rotation2d heading) {
    alignmentController.setSetpoint(heading.getRadians());
    isAligning = true;
  }

  /**
   * 
   * @param point The point on the field we want to face
   */
  public void facePoint(Translation2d point) {
    Translation2d currentPosition = getPose().getTranslation();
    setTargetHeading(point.minus(currentPosition).getAngle());
  }

  public Rotation2d getTargetHeading() {
    return Rotation2d.fromRadians(alignmentController.getSetpoint());
  }
  
  /**
   * This creates an "X" pattern with the wheels which makes the robot very hard to move
   */
  private void parkModules() {
    modules[0].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[1].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[2].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[3].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    parked = true;
  }

  /**
   * Resets the odometer position to a given position
   * @param pose Position to reset the odometer to
   */
  private void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /**
   * 
   * @param visionMeasurement The robot position on the field from the apriltags
   */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestamp);
  }

  /**
   * Stops all motors on all modules
   */
  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  public Translation2d getFieldVelocity() {
    ChassisSpeeds fieldRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getMeasuredChassisSpeeds(), getHeading());
    return new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond);
  }

  /**
   * @return Target chassis x, y, and rotational velocity (robot-relative)
   */
  private ChassisSpeeds getTargetChassisSpeeds() {
    return kinematics.toChassisSpeeds(getTargetModuleStates());
  }

  /**
   * @return Measured chassis x velocity, y velocity, and rotational velocity (robot-relative)
   */
  private ChassisSpeeds getMeasuredChassisSpeeds() {
    return kinematics.toChassisSpeeds(getMeasuredModuleStates());
  }

  /**
   * @return Driven chassis x speed, y speed, and rotational speed (robot-relative)
   */
  private ChassisSpeeds getDrivenChassisSpeeds() {
    return drivenChassisSpeeds;
  }

  /**
   * @return Measured module positions
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      positions[i] = modules[i].getModulePosition();
    }
    return positions;
  }

  /**
   * @return Target module states (speed and direction)
   */
  private SwerveModuleState[] getTargetModuleStates() {
    SwerveModuleState[] targetStates = new SwerveModuleState[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      targetStates[i] = modules[i].getTargetState();
    }
    return targetStates;
  }

  /**
   * @return Measured module states (speed and direction)
   */
  private SwerveModuleState[] getMeasuredModuleStates() {
    SwerveModuleState[] measuredStates = new SwerveModuleState[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      measuredStates[i] = modules[i].getMeasuredState();
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
   * @return This swerve drive's NavX AHRS IMU Gyro
   */
  public AHRS getGyro() {
    return gyro;
  }

  /**
   * Resets gyro heading
   */
  public void resetGyroHeading(Rotation2d newHeading) {
    gyroOffset = newHeading.minus(gyroHeading);
    alignmentController.reset();
    alignmentController.setSetpoint(newHeading.getRadians());
  }

  /**
   * @return Gyro heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * @return Heading as a Rotation2d based on alliance
   */
  public Rotation2d getAllianceAwareHeading() {
    return getHeading().plus(Rotation2d.fromDegrees(Constants.IS_BLUE_TEAM ? 0.0 : 180.0));
  }

  /**
   * @return Pose on the field from odometer data as a Pose2d
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
    // Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
    // return new Pose2d(estimatedPose.getTranslation(), estimatedPose.getRotation().rotateBy(gyroOffset));
  }

  /**
   * @return Field2d object for SmartDashboard widget.
   */
  public Field2d getField() {
    return field;
  }

  /**
   * Converts the speed of a wheel moving to the angular velocity of the robot as if it's
   * rotating in place
   * @param wheelSpeed Drive velocity in m/s
   * @return Equivalent rotational velocity in rad/s
   * @see #toLinear(double)
   */
  public static double toAngular(double wheelSpeed) {
    return wheelSpeed / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS;
  }

  /**
   * Converts the angular velocity of the robot to the speed of a wheel moving as if the
   * robot is rotating in place
   * @param angularVelocity Rotational velocity in rad/s
   * @return Equivalent drive velocity in m/s
   * @see #toAngular(double)
   */
  public static double toLinear(double angularVelocity) {
    return angularVelocity * SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS;
  }

  /**
   * Create a kinematics object for the swerve drive based on the SWERVE_DRIVE constants
   * @return A SwerveDriveKinematics object that models the swerve drive
  */
  public static SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(
      new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
      new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0), 
      new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
      new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0));
  }

  public static void printChoreoConfig() {
    System.out.println(
      String.format(
        """


////////////////////////////////
///// CHOREO ROBOT CONFIG //////
////////////////////////////////

---------- DIMENSIONS ----------
  Mass: %.3f kg
  MOI: %.3f kg * m^2
  Bumper Width: %.3f m
  Bumper Length: %.3f m
  Wheelbase: %.3f m
  Trackwidth: %.3f m

--------- DRIVE MOTOR ----------
  Wheel Radius: %.3f m
  Gearing: %.3f : 1
  Motor Max Speed: %.0f RPM
  Motor Max Torque: %.3f N * m


        """,
        SWERVE_DRIVE.ROBOT_MASS,
        SWERVE_DRIVE.PHYSICS.ROTATIONAL_INERTIA,
        SWERVE_DRIVE.BUMPER_WIDTH,
        SWERVE_DRIVE.BUMPER_LENGTH,
        SWERVE_DRIVE.WHEELBASE,
        SWERVE_DRIVE.TRACKWIDTH,
        SWERVE_DRIVE.WHEEL_RADIUS,
        SWERVE_DRIVE.DRIVE_MOTOR_GEARING,
        SWERVE_DRIVE.PHYSICS.MAX_MOTOR_SPEED,
        SWERVE_DRIVE.PHYSICS.MAX_MOTOR_TORQUE
      )
    );
  }

  public Command followChoreoTrajectory(String pathName, boolean first) {

    ChoreoTrajectory trajectory = Choreo.getTrajectory(pathName);

    field.getObject("traj").setPoses(
      trajectory.getInitialPose(), trajectory.getFinalPose()
    );
    field.getObject("trajPoses").setPoses(
      trajectory.getPoses()
    );

    Command swerveCommand = Choreo.choreoSwerveCommand(
        // Choreo trajectory to follow
        trajectory,

        // A supplier that returns the current field-relative pose of the robot based on the wheel
        // and vision odometry
        this::getPose,

        // PIDControllers for correcting errors in field-relative translation (input: X or Y error in
        // meters, output: m/s).
        new PIDController(
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI, 
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD
        ),
        new PIDController(
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI, 
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD
        ),

        // PIDController to correct for rotation error (input: heading error in radians, output: rad/s)
        new PIDController(
          SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kI,
          SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kD
        ),

        // A consumer which drives the robot in robot-relative coordinates
        this::driveRobotRelative,
        
        // A supplier which returns whether or not to mirror the path based on alliance (this assumes
        // the path is created for the blue alliance)
        () -> false,

        // The subsystem(s) to require, typically your drive subsystem only
        this
    );
    
    if (first) {
      return Commands.sequence(
        Commands.runOnce(() -> System.out.println("===== STARTING AUTO =====")),
        Commands.runOnce(() -> this.resetPose(trajectory.getInitialPose())),
        swerveCommand
      );
    } else {
      return Commands.sequence(
        Commands.runOnce(() -> System.out.println("===== STARTING AUTO =====")),
        swerveCommand
      );
    }
  }

  /**
   * Go to a position on the field
   * @param goalPosition Field-relative position on the field to go to
   * @param orientation Field-relative orientation to rotate to
   * @return A command to run
   */
  public Command goTo(Pose2d pose, XboxController xboxController) {
    Command pathfindingCommand = goToSimple(pose, xboxController);

    if (pose.getTranslation().getDistance(getPose().getTranslation()) > 1.0) {

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      pathfindingCommand = AutoBuilder.pathfindToPose(
        pose,
        SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
      );
    }

    return Commands.sequence(
      pathfindingCommand,
      goToSimple(pose, xboxController)
    ).onlyWhile(() -> Constants.isIdle(xboxController));
  }

  private Command goToSimple(Pose2d pose, XboxController xboxController) {
    Rotation2d angle = pose.getTranslation().minus(getPose().getTranslation()).getAngle();

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(getPose().getTranslation(), angle),
      new Pose2d(pose.getTranslation(), angle)
    );

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      new GoalEndState(
        0.0,
        pose.getRotation(),
        true
      )
    );

    return Commands.sequence(
      AutoBuilder.followPath(path),
      Commands.runOnce(() -> setTargetHeading(pose.getRotation()))
    ).onlyWhile(() -> Constants.isIdle(xboxController));
  }

  public Command goToNearestPose(List<Pose2d> poses, XboxController xboxController) {
   return goTo(getPose().nearest(poses), xboxController).onlyWhile(() -> Constants.isIdle(xboxController));
  }
  
  public boolean shouldFlipPaths() {
    Optional<Alliance> optional = DriverStation.getAlliance();

    if (!optional.isPresent()) {
      System.out.println("Cannot determine alliance. Defaulting to blue");

      return false;
    }

    return optional.get().equals(Alliance.Red);
  }

  /**
   * Go to a position on the field
   * @param goalPosition Field-relative position on the field to go to
   * @param orientation Field-relative orientation to rotate to
   * @return A command to run
   */
  public Command goTo(Translation2d goalPosition, Rotation2d orientation) {
    Rotation2d angle = goalPosition.minus(getPose().getTranslation()).getAngle();

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(getPose().getTranslation(), angle),
      new Pose2d(goalPosition, angle)
    );

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      new PathConstraints(
        SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY, 
        SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION,
        SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY, 
        SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_ACCELERATION
      ),
      new GoalEndState(
        0.0,
        orientation
      )
    );

    return Commands.sequence(
      AutoBuilder.followPath(path),
      Commands.runOnce(() -> setTargetHeading(orientation))
    );
  }
}