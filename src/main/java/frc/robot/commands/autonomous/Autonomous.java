package frc.robot.commands.autonomous;

import java.io.Console;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Field;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.Shooter;

public class Autonomous extends Command {
  private RobotStateController controller;
  private SwerveDrive swerveDrive;
  private List<Integer> notesToGet = List.of();
  private List<Integer> notesThatExist = List.of();
  private double robotDiagonal = Math.sqrt(Math.pow(Constants.SWERVE_DRIVE.BUMPER_LENGTH, 2.0) + Math.pow(Constants.SWERVE_DRIVE.BUMPER_WIDTH, 2.0));
  private double noteAvoidRadius = 0.75 * (Field.NOTE_LENGTH / 2.0 + Math.max(Constants.SWERVE_DRIVE.BUMPER_LENGTH, Constants.SWERVE_DRIVE.BUMPER_WIDTH) / 2.0);
  private boolean firstNote = true;
  private boolean dontPathfind = false;

  public Autonomous(RobotStateController controller, SwerveDrive swerveDrive, List<Integer> notesToGet) {
    this.controller = controller;
    this.swerveDrive = swerveDrive;
    this.notesToGet = notesToGet;
    this.notesThatExist = new ArrayList<>(List.of(0, 1, 2, 3, 4, 5, 6, 7));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pickupAndShootAll().schedule();
  }

  public void addDynamicObstacles() {
    List<Pair<Translation2d, Translation2d>> dynamicObstacles = new ArrayList<>();    
    for (Integer note : notesThatExist) {      
      // Define bounding box for each note
      dynamicObstacles.add(
        Pair.of(
          Field.NOTE_POSITIONS[note].minus(new Translation2d(noteAvoidRadius, noteAvoidRadius)), // example bounding box corner
          Field.NOTE_POSITIONS[note].plus(new Translation2d(noteAvoidRadius, noteAvoidRadius)) // example bounding box corner
        )
      );
    }
    
    // Set dynamic obstacles
    Pathfinding.setDynamicObstacles(dynamicObstacles, swerveDrive.getPose().getTranslation());
  }

  public Integer getNextClosestNote() {
    List<Integer> closeNotes = notesToGet.stream().filter(n -> n <= 2).collect(Collectors.toList());
    List<Integer> wingNotes = notesToGet.stream().filter(n -> n > 2).collect(Collectors.toList());
    
    List<Integer> relevantNotes = closeNotes.isEmpty() ? wingNotes : closeNotes;
    Integer lowestNote = Collections.max(relevantNotes);
    Integer highestNote = Collections.min(relevantNotes);

    Translation2d closestPosition = swerveDrive.getPose().getTranslation().nearest(
      List.of(
        Field.NOTE_POSITIONS[lowestNote],
        Field.NOTE_POSITIONS[highestNote]
      )
    );

    if (closestPosition.equals(Field.NOTE_POSITIONS[lowestNote])) {
      return lowestNote;
    } else {
      return highestNote;
    }
  }

  public Translation2d getClosestShootingPoint() {
    if ((swerveDrive.getPose().getX() < Field.WING_X && Constants.IS_BLUE_TEAM) || (swerveDrive.getPose().getX() > Field.WING_X && !Constants.IS_BLUE_TEAM)) {
      return swerveDrive.getPose().getTranslation();
    }
    if (notesToGet.isEmpty()) {
      return swerveDrive.getPose().getTranslation().nearest(List.of(Field.SHOT_POSITIONS));
    }

    Translation2d closestPoint = Field.SHOT_POSITIONS[0];
    double bestWeight = Double.MAX_VALUE;
    for (Translation2d position : Field.SHOT_POSITIONS) {
      double weight = swerveDrive.getPose().getTranslation().getDistance(position) + 
        position.getDistance(Field.NOTE_POSITIONS[getNextClosestNote()]) + 
        position.getDistance(Field.SPEAKER.toTranslation2d()) * 1.2;
      if (weight < bestWeight) {
        bestWeight = weight;
        closestPoint = position;
      }
    }
    return closestPoint;
  }

  public Command pickupClosestNote() {
    Integer closestNote = getNextClosestNote();
    Translation2d notePosition = Field.NOTE_POSITIONS[closestNote];
    Rotation2d heading = notePosition.minus(swerveDrive.getPose().getTranslation()).getAngle();
    
    boolean pathfind = false;
    if (closestNote >= 3) {
      pathfind = true;
    }
    if (firstNote) {
      pathfind = true;
      firstNote = false;
    }
    if (dontPathfind) {
      pathfind = false;
      dontPathfind = false;
    }

    return Commands.sequence(
      Commands.runOnce(() -> {
        addDynamicObstacles();
      }),
      pickupNote(notePosition, pathfind),
      Commands.runOnce(() -> {
        notesThatExist.remove(closestNote);
        notesToGet.remove(closestNote);
        swerveDrive.setRotationTargetOverrideFromPoint(null);
        addDynamicObstacles();
      })
    );
  }

  public Command pickupNote(Translation2d notePosition, boolean pathfind) {
    

    Rotation2d heading = notePosition.minus(swerveDrive.getPose().getTranslation()).getAngle();
    Translation2d pathfindPosition = Field.SPEAKER.toTranslation2d().minus(notePosition);
    Rotation2d speakerNoteAngle = pathfindPosition.getAngle();
    pathfindPosition = pathfindPosition.div(pathfindPosition.getNorm()).times(Constants.SWERVE_DRIVE.BUMPER_LENGTH / 2.0 + Field.NOTE_LENGTH / 2.0);
    pathfindPosition = pathfindPosition.plus(notePosition);
    
    Command pathplannerCommand = Commands.runOnce(() -> {});
    if (pathfind) {
      pathplannerCommand = Commands.sequence(
        Commands.runOnce(() -> swerveDrive.setRotationTargetOverrideFromPoint(notePosition)),
        swerveDrive.goTo(new Pose2d(notePosition, swerveDrive.getHeading()))
      ).finallyDo(() -> swerveDrive.setRotationTargetOverrideFromPoint(null));
    } else {

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(swerveDrive.getPose().getTranslation(), Field.SPEAKER.toTranslation2d().minus(swerveDrive.getPose().getTranslation()).getAngle()),
        new Pose2d(pathfindPosition, notePosition.minus(Field.SPEAKER.toTranslation2d()).getAngle()),
        // new Pose2d(
        //   new Translation2d(
        //     Math.signum(notePosition.minus(Field.SPEAKER.toTranslation2d()).getX()) * -0.6 + notePosition.getX(),
        //     notePosition.minus(swerveDrive.getPose().getTranslation()).div(2.0).plus(swerveDrive.getPose().getTranslation()).getY()
        //   ),
        //   notePosition.minus(swerveDrive.getPose().getTranslation()).getAngle()
        // ),
        new Pose2d(notePosition, speakerNoteAngle.plus(Rotation2d.fromDegrees(180.0)))
      );

      PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
        new GoalEndState(
          0.0,
          speakerNoteAngle,
          true
        )
      );

      pathplannerCommand = Commands.sequence(
        Commands.runOnce(() -> swerveDrive.setRotationTargetOverrideFromPointBackwards(Field.SPEAKER.toTranslation2d())),
        AutoBuilder.followPath(path)
      ).finallyDo(() -> swerveDrive.setRotationTargetOverrideFromPoint(null));
    }

    return Commands.sequence(
      pathplannerCommand.alongWith(
        Commands.sequence(
          Commands.waitUntil(() -> swerveDrive.getPose().getTranslation().getDistance(notePosition) < robotDiagonal / 2.0 + Field.NOTE_LENGTH),
          controller.setState(RobotStateController.State.INTAKE).onlyIf(() -> RobotBase.isReal())
        )
      ).raceWith(
        Commands.waitUntil(() -> controller.hasNote())
      ),
      Commands.runOnce(() -> controller.setState(RobotStateController.State.CENTER_NOTE))
    );
    
    // return Commands.sequence(
    //   Commands.runOnce(() -> swerveDrive.setRotationTargetOverrideFromPoint(notePosition)),
    //   swerveDrive.goTo(new Pose2d(pathfindPosition, swerveDrive.getHeading())),
    //   Commands.runOnce(() -> addDynamicObstacles()),
    //   controller.setState(RobotStateController.State.INTAKE).onlyIf(() -> RobotBase.isReal())
    //     .alongWith(swerveDrive.goTo(new Pose2d(notePosition, swerveDrive.getHeading())))
    //     .until(() -> controller.hasNote()),
    //   Commands.runOnce(() -> controller.setState(RobotStateController.State.CENTER_NOTE)),
    //   Commands.runOnce(() -> swerveDrive.setRotationTargetOverrideFromPoint(null))
    // );
  }

  public Command moveAndShoot() {
    if (!controller.hasNote()) {
      return Commands.runOnce(() -> {
        dontPathfind = true;
      });
    }

    Translation2d shotPosition = getClosestShootingPoint();
    Rotation2d heading = Field.SPEAKER.toTranslation2d().minus(shotPosition).getAngle().plus(Rotation2d.fromDegrees(180.0));
    Command moveToCommand = swerveDrive.goTo(new Pose2d(shotPosition, heading));
    if (shotPosition.getDistance(swerveDrive.getPose().getTranslation()) < Units.inchesToMeters(6.0)) {
      moveToCommand = Commands.runOnce(() -> {});
    }
    return Commands.sequence(
      Commands.runOnce(() -> swerveDrive.setRotationTargetOverrideFromPointBackwards(Field.SPEAKER.toTranslation2d())),
      moveToCommand
        .alongWith(controller.setState(RobotStateController.State.SPIN_UP))
        .alongWith(controller.setState(RobotStateController.State.AIM_SPEAKER))
        .until(() -> controller.canShoot()),
      controller.setState(RobotStateController.State.SHOOT_SPEAKER)
        .until(() -> !controller.hasNote()),
      Commands.runOnce(() -> swerveDrive.setRotationTargetOverrideFromPointBackwards(null))
    );
  }

  public boolean hasPickedUpNote(Translation2d notePosition) {
    Translation2d futurePosition = swerveDrive.getPose().getTranslation();
    futurePosition = futurePosition.plus(swerveDrive.getFieldVelocity().times(swerveDrive.getFieldVelocity().getNorm()).div(2.0 * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION));
    SwerveDrive.getField().getObject("futurePosition").setPoses(List.of(new Pose2d(futurePosition, new Rotation2d())));
    return notePosition.getDistance(futurePosition) < Constants.SWERVE_DRIVE.BUMPER_LENGTH / 2.0;
  }

  public Command pickupAndShootAll() {
    if (notesToGet.isEmpty()) {
      cancel();
      return Commands.runOnce(() -> System.out.println("NO MORE NOTES TO PICKUP"));
    } else {
      return pickupClosestNote().andThen(
        () -> moveAndShoot().andThen(
          () -> pickupAndShootAll().schedule()
        ).schedule()
      );
    }
  }

  public boolean shouldSeeNote(int note) {
    Translation2d position = Field.NOTE_POSITIONS[note];
    Translation2d relativePosition = position.minus(swerveDrive.getPose().getTranslation()).rotateBy(swerveDrive.getPose().getRotation().unaryMinus());
    relativePosition = relativePosition.minus(Constants.LIMELIGHT.NOTE_CAMERA_POSITION.toTranslation2d());
    
    if (relativePosition.getX() < 0) {
      return false;
    }

    Rotation2d lateralAngleMin = Rotation2d.fromRadians(Math.atan((relativePosition.getY() - Field.NOTE_LENGTH / 2.0) / relativePosition.getX()));
    Rotation2d lateralAngleMax = Rotation2d.fromRadians(Math.atan((relativePosition.getY() + Field.NOTE_LENGTH / 2.0) / relativePosition.getX()));

    Rotation2d verticalAngle = Rotation2d.fromRadians(Math.atan((Constants.LIMELIGHT.NOTE_CAMERA_POSITION.getZ() - Field.NOTE_THICKNESS / 2.0) / relativePosition.getX()));
    verticalAngle = verticalAngle.plus(Constants.LIMELIGHT.NOTE_CAMERA_PITCH);

    if (verticalAngle.getDegrees() < -(Constants.LIMELIGHT.FOV_HEIGHT.getDegrees() / 2.0) || verticalAngle.getDegrees() > (Constants.LIMELIGHT.FOV_HEIGHT.getDegrees() / 2.0)) {
      return false;
    }

    if (lateralAngleMin.getDegrees() < -(Constants.LIMELIGHT.FOV_WIDTH.getDegrees() / 2.0) || lateralAngleMax.getDegrees() > (Constants.LIMELIGHT.FOV_WIDTH.getDegrees() / 2.0)) {
      return false;
    }

    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    FieldObject2d visibleNotes = SwerveDrive.getField().getObject("visibleNotes");
    List<Pose2d> poses = new ArrayList<>();
    for (Integer note : List.of(0, 1, 2, 3, 4, 5, 6, 7)) {
      if (shouldSeeNote(note)) {
        poses.add(new Pose2d(Field.NOTE_POSITIONS[note], new Rotation2d()));
      }
    }
    visibleNotes.setPoses(poses);

    Translation2d futurePosition = swerveDrive.getPose().getTranslation();
    futurePosition = futurePosition.plus(swerveDrive.getFieldVelocity().times(swerveDrive.getFieldVelocity().getNorm()).div(2.0 * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION));
    SwerveDrive.getField().getObject("futurePosition").setPoses(List.of(new Pose2d(futurePosition, swerveDrive.getPose().getRotation())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
