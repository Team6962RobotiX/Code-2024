// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.Sendable;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.PHYSICS;
import frc.robot.util.StatusChecks;
import frc.robot.util.Logging.Logger;

/**
 * This class represents the subsystem for the swerve drive. It contains four
 * swerve module objects and a gyroscope object.
 */
public class SwerveDrive extends SubsystemBase {
  private SwerveModule[] modules = new SwerveModule[SWERVE_DRIVE.MODULE_COUNT];
  private AHRS gyro;

  private SwerveDriveKinematics kinematics = getKinematics();
  private SwerveDrivePoseEstimator poseEstimator;
  private Field2d field = new Field2d();
  private Rotation2d heading = Rotation2d.fromDegrees(0);

  private ChassisSpeeds drivenChassisSpeeds = new ChassisSpeeds();

  private ProfiledPIDController rotateController = new ProfiledPIDController(
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kP,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kI,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kD,
    new Constraints(
      SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY,
      SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_ACCELERATION
    )
  );

  public SwerveDrive() {
    // Create the serve module objects
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      if (RobotBase.isSimulation()) {
        modules[i] = new SwerveModuleSim(i);
      } else {
        modules[i] = new SwerveModule(i);
      }
    }
    
    // Set up pose estimator and rotation controller
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePositions(), SWERVE_DRIVE.STARTING_POSE);
    rotateController.enableContinuousInput(-Math.PI, Math.PI);

    // If possible, connect to the gyroscope
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
    Logger.autoLog("SwerveDrive/targetStates", this::getTargetModuleStates);
    Logger.autoLog("SwerveDrive/measuredStates", this::getMeasuredModuleStates);


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

    StatusChecks.addCheck("Gyro Connection", gyro::isConnected);
  }

  @Override
  public void periodic() {
    // Update current heading based on gyroscope or wheel speeds
    if (gyro.isConnected() && !RobotBase.isSimulation()) {
      heading = gyro.getRotation2d();
    } else {
      heading = heading.plus(new Rotation2d(getMeasuredChassisSpeeds().omegaRadiansPerSecond * 0.02));
    }

    heading = Rotation2d.fromRadians(MathUtil.angleModulus(heading.getRadians()));
    
    // Update pose based on measured heading and swerve module positions
    poseEstimator.update(getHeading(), getModulePositions());

    // Update field
    FieldObject2d modulesObject = field.getObject("Swerve Modules");

    // Update swerve module poses
    Pose2d[] modulePoses = new Pose2d[SWERVE_DRIVE.MODULE_COUNT];
    Pose2d robotPose = getPose();
    
    for (SwerveModule module : modules) {
      modulePoses[module.id] = module.getPose(robotPose);
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

  public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
    driveFieldRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveAttainableSpeeds(fieldRelativeSpeeds);
  }
  

  /**
   * Drives the robot at a given robot-relative speed and direction
   * @param xVelocity The x speed to drive at
   * @param yVelocity The y speed to drive at
   * @param angularVelocity The angular speed to drive at
   */
  public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
    driveRobotRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    driveFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getHeading()));
  }


  private void driveAttainableSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
    // Find the speeds in meters per second that the robot is trying to turn at and the speeds the
    // robot is currently turning at
    double targetWheelSpeeds = toLinear(Math.abs(fieldRelativeSpeeds.omegaRadiansPerSecond));
    double measuredWheelSpeeds = toLinear(Math.abs(getMeasuredChassisSpeeds().omegaRadiansPerSecond));

    // Get the current heading of the robot in radians
    double measuredHeading = getHeading().getRadians();
    
    // Continue to turn if the robot is not passing or going to be passing the velocity deadband
    if (targetWheelSpeeds < SWERVE_DRIVE.VELOCITY_DEADBAND && measuredWheelSpeeds < SWERVE_DRIVE.VELOCITY_DEADBAND) {
      fieldRelativeSpeeds.omegaRadiansPerSecond += rotateController.calculate(measuredHeading);
    } else {
      // If the velocity deadband is reached, zero the angular velocity
      rotateController.calculate(measuredHeading, measuredHeading);
    }

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
    
    driveModules(kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(drivenChassisSpeeds, getHeading())));
  }

  private void driveModules(SwerveModuleState[] moduleStates) {
    // Prevents wheels from going over the maximum velocity (based on the motor power cap)
    double maxWheelVelocity = SwerveModule.calcWheelVelocity(SWERVE_DRIVE.MOTOR_POWER_HARD_CAP);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxWheelVelocity);

    // Drive the swerve modules at the calculated speeds
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
    rotateController.calculate(getHeading().getRadians());
  }

  /**
   * Resets the odometer position to a given position
   * @param pose Position to reset the odometer to
   * @implNote Currently does nothing
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
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  /**
   * @return Target chassis x, y, and rotational velocity
   */
  public ChassisSpeeds getTargetChassisSpeeds() {
    return kinematics.toChassisSpeeds(getTargetModuleStates());
  }

  /**
   * @return Measured chassis x velocity, y velocity, and rotational velocity
   */
  public ChassisSpeeds getMeasuredChassisSpeeds() {
    return kinematics.toChassisSpeeds(getMeasuredModuleStates());
  }

  /**
   * @return Driven chassis x speed, y speed, and rotational speed
   */
  public ChassisSpeeds getDrivenChassisSpeeds() {
    return drivenChassisSpeeds;
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
   * @return Resets gyro heading
   */
  public void setHeading(Rotation2d newHeading) {
    heading = newHeading;
    gyro.reset();
    gyro.setAngleAdjustment(newHeading.getDegrees());
    rotateController.reset(newHeading.getRadians(), 0.0);
    rotateController.calculate(newHeading.getRadians(), newHeading.getRadians());
  }

  /**
   * @return Gyro heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return heading;
  }

  /**
   * @return Pose on the field from odometer data as a Pose2d
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

------- MOTOR CALCULATOR -------
  NEO
  Current Limit: %s A

(You've done it right when these all check out)
------ THEORETICAL -------
  Floor Speed: %.3f m/s
  Floor Accel: %.3f m/s^2
  Ang Speed: %.3f rad/s
  Ang Accel: %.3f rad/s^2


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
        SWERVE_DRIVE.PHYSICS.MAX_MOTOR_TORQUE,
        SWERVE_DRIVE.PHYSICS.SLIPLESS_CURRENT_LIMIT,
        SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
        SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION,
        SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY,
        SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_ACCELERATION
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
}