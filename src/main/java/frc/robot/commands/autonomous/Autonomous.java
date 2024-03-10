package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.Constants.Field;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Notes;

public class Autonomous extends Command {
  private RobotStateController controller;
  private SwerveDrive swerveDrive;
  private List<Integer> queuedNotes = List.of();
  private List<Integer> remainingNotes = List.of();

  private double noteAvoidRadius = (Field.NOTE_LENGTH / 2.0 + Constants.SWERVE_DRIVE.BUMPER_DIAGONAL / 2.0);
  private double notePickupDistance = Constants.SWERVE_DRIVE.BUMPER_LENGTH / 2.0 - Field.NOTE_LENGTH;
  private double noteAlignDistance = Constants.SWERVE_DRIVE.BUMPER_LENGTH / 2.0 + Field.NOTE_LENGTH * 2.0;
  private double speakerShotDistance = 3.5;
  private double adjacentNoteBand = 0.5;
  private double minPathfindingDistance = 2.5;
  private double visionNoteDistance = 2.5;
  private Command runningCommand;

  private boolean simulatedNote = true;
  private boolean isFirstNote = true;
  public static boolean avoidPillars = true;
  
  private enum State {
    SHOOT,
    PICKUP
  }

  public State state;

  public Autonomous(RobotStateController controller, SwerveDrive swerveDrive, List<Integer> queuedNotes) {
    this.controller = controller;
    this.swerveDrive = swerveDrive;
    this.queuedNotes = queuedNotes;
    this.remainingNotes = new ArrayList<>(List.of(0, 1, 2, 3, 4, 5, 6, 7));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = null;
    simulatedNote = true;
    controller.setState(RobotStateController.State.LEAVE_AMP).withTimeout(1.0).schedule();
  }

  public Integer getClosestNote() {
    List<Integer> closeNotes = queuedNotes.stream().filter(n -> n <= 2).collect(Collectors.toList());
    List<Integer> wingNotes = queuedNotes.stream().filter(n -> n > 2).collect(Collectors.toList());
    
    List<Integer> relevantNotes = closeNotes.isEmpty() ? wingNotes : closeNotes;
    Integer lowestNote = Collections.max(relevantNotes);
    Integer highestNote = Collections.min(relevantNotes);

    Translation2d closestPosition = swerveDrive.getPose().getTranslation().nearest(
      List.of(
        Field.NOTE_POSITIONS.get(lowestNote).get(),
        Field.NOTE_POSITIONS.get(highestNote).get()
      )
    );

    if (closestPosition.equals(Field.NOTE_POSITIONS.get(lowestNote).get())) {
      return lowestNote;
    } else {
      return highestNote;
    }
  }

  public Translation2d getClosestShootingPosition() {
    List<Translation2d> shotPositions = Field.SHOT_POSITIONS.stream().map(Supplier::get).collect(Collectors.toList());
    return swerveDrive.getPose().getTranslation().nearest(shotPositions);
  }

  public void addNoteObstacles() {
    List<Pair<Translation2d, Translation2d>> dynamicObstacles = new ArrayList<>();
    for (Integer note : remainingNotes) {
      dynamicObstacles.add(
        Pair.of(
          Field.NOTE_POSITIONS.get(note).get().minus(new Translation2d(noteAvoidRadius, noteAvoidRadius)),
          Field.NOTE_POSITIONS.get(note).get().plus(new Translation2d(noteAvoidRadius, noteAvoidRadius))
        )
      );
    }
    Pathfinding.setDynamicObstacles(dynamicObstacles, swerveDrive.getPose().getTranslation());
  }

  public void clearNoteObstacles() {
    Pathfinding.setDynamicObstacles(new ArrayList<>(), swerveDrive.getPose().getTranslation());
  }


  public Translation2d getVisionNotePosition(Translation2d queuedNotePosition) {
    List<Translation2d> measuredNotePositions = Notes.getNotePositions(LIMELIGHT.NOTE_CAMERA_NAME, LIMELIGHT.NOTE_CAMERA_PITCH, swerveDrive, swerveDrive.getFieldVelocity(), LIMELIGHT.NOTE_CAMERA_POSITION);

    if (RobotBase.isSimulation()) {
      measuredNotePositions = new ArrayList<>();
      for (Integer note : List.of(0, 1, 2, 3, 4, 5, 6, 7)) {
        if (shouldSeeNote(note)) {
          measuredNotePositions.add(Field.NOTE_POSITIONS.get(note).get().plus(new Translation2d(0.0, 0.0)));
        }
      }
    }

    FieldObject2d visibleNotes = SwerveDrive.getField().getObject("visibleNotes");
    List<Pose2d> poses = new ArrayList<>();

    if (measuredNotePositions.size() == 0) return null;

    List<Translation2d> theoreticalNotePositions = Field.NOTE_POSITIONS.stream().map(Supplier::get).collect(Collectors.toList());
    
    for (Translation2d measuredNotePosition : measuredNotePositions) {
      if ((measuredNotePosition.getX() > Field.LENGTH / 2.0 && Constants.IS_BLUE_TEAM.get()) || (measuredNotePosition.getX() < Field.LENGTH / 2.0 && !Constants.IS_BLUE_TEAM.get())) {
        Translation2d relativeNotePosition = measuredNotePosition.minus(swerveDrive.getPose().getTranslation());
        relativeNotePosition = relativeNotePosition.div(-relativeNotePosition.getX());
        relativeNotePosition = relativeNotePosition.times(swerveDrive.getPose().getTranslation().getX() - Field.LENGTH / 2);
        measuredNotePosition = relativeNotePosition.plus(swerveDrive.getPose().getTranslation());
      }
      poses.add(new Pose2d(measuredNotePosition, new Rotation2d()));
      visibleNotes.setPoses(poses);
      Translation2d theoreticalNoteCounterpart = measuredNotePosition.nearest(theoreticalNotePositions);
      if (theoreticalNoteCounterpart.getDistance(queuedNotePosition) < 0.05 && swerveDrive.getPose().getTranslation().getDistance(measuredNotePosition) < visionNoteDistance) {
        return measuredNotePosition;
      }
    }

    return null;
  }

  public Command pickup(Translation2d notePosition) {
    if (hasNote()) return Commands.runOnce(() -> {});

    List<Translation2d> fieldNotePositions = Field.NOTE_POSITIONS.stream().map(Supplier::get).collect(Collectors.toList());
    Integer noteIndex = fieldNotePositions.indexOf(notePosition.nearest(fieldNotePositions));

    avoidPillars = noteIndex != 2;

    Rotation2d heading = notePosition.minus(Field.SPEAKER.get().toTranslation2d()).getAngle();

    Translation2d alignmentPoint = new Translation2d(-noteAlignDistance, 0).rotateBy(heading).plus(notePosition);
    Translation2d pickupPoint = new Translation2d(-notePickupDistance, 0).rotateBy(heading).plus(notePosition);
    
    // SwerveDrive.getField().getObject("bezier").setPoses(List.of(new Pose2d(alignmentPoint, heading), new Pose2d(pickupPoint, heading)));
    
    boolean adjacent = Math.abs(swerveDrive.getPose().getX() - notePosition.getX()) < adjacentNoteBand;
    boolean pathfind = !adjacent && !tooClose(notePosition);
    
    if (pathfind) {
      System.out.println("PATHFIND");
      return Commands.sequence(
        Commands.runOnce(() -> {
          swerveDrive.setRotationTargetOverrideFromPointBackwards(Field.SPEAKER.get().toTranslation2d());
          addNoteObstacles();
        }),
        AutoBuilder.pathfindToPose(
          new Pose2d(alignmentPoint, heading),
          SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS
        )
        .until(() -> tooClose(notePosition))
        .finallyDo(() -> pickup(notePosition).schedule())
      );
    }
    
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(swerveDrive.getPose().getTranslation(), swerveDrive.getFieldVelocity().getAngle()),
      new Pose2d(alignmentPoint, heading),
      new Pose2d(pickupPoint, heading)
    );

    if (adjacent && swerveDrive.getFieldVelocity().getNorm() < 1.0) {
      System.out.println("ADJACENT");
      bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(swerveDrive.getPose().getTranslation(), Field.SPEAKER.get().toTranslation2d().minus(swerveDrive.getPose().getTranslation()).getAngle()),
        new Pose2d(alignmentPoint, heading),
        new Pose2d(pickupPoint, heading)
      );
    }

    System.out.println("STANDARD");

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      new GoalEndState(
        0.0,
        swerveDrive.getHeading(),
        true
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> {
        swerveDrive.setRotationTargetOverrideFromPointBackwards(Field.SPEAKER.get().toTranslation2d());
        remainingNotes.remove(noteIndex);
        queuedNotes.remove(noteIndex);
        addNoteObstacles();
      }),
      AutoBuilder.followPath(path)
        .raceWith(
          Commands.sequence(
            Commands.waitUntil(() -> swerveDrive.getFuturePose().getTranslation().getDistance(notePosition) < 2.0),
            controller.setState(RobotStateController.State.INTAKE).until(() -> hasNote())
          )
        ),
      Commands.runOnce(() -> {
        simulatedNote = true;
        if (!hasNote() && !queuedNotes.isEmpty()) {
          pickup(Field.NOTE_POSITIONS.get(getClosestNote()).get()).schedule();
        }
        clearNoteObstacles();
        swerveDrive.setRotationTargetOverrideFromPointBackwards(null);
      })
    ).until(() -> {
      Translation2d visionNotePosition = getVisionNotePosition(notePosition.nearest(fieldNotePositions));
      if (visionNotePosition != null && visionNotePosition.getDistance(notePosition) > Field.NOTE_LENGTH / 2.0) {
        System.out.println("VISION");
        pickup(visionNotePosition).schedule();
        return true;
      }
      return false;
    });
  }

  public Command shoot() {
    if (!hasNote()) return Commands.runOnce(() -> {});

    Translation2d shootingPosition = getClosestShootingPosition();
    
    Command moveCommand = Commands.runOnce(() -> {});
    if (Field.SPEAKER.get().toTranslation2d().getDistance(swerveDrive.getFuturePose().getTranslation()) > speakerShotDistance || controller.underStage()) {
      moveCommand = AutoBuilder.pathfindToPose(
        new Pose2d(shootingPosition, swerveDrive.getHeading()),
        SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS
      );
    }
    return Commands.sequence(
      Commands.runOnce(() -> {
        swerveDrive.setRotationTargetOverrideFromPointBackwards(Field.SPEAKER.get().toTranslation2d());
      }),
      Commands.parallel(
        moveCommand.until(() -> !controller.underStage() && swerveDrive.getFuturePose().getTranslation().getDistance(Field.SPEAKER.get().toTranslation2d()) < speakerShotDistance),
        controller.setState(RobotStateController.State.SPIN_UP),
        controller.setState(RobotStateController.State.AIM_SPEAKER)
      )
      .raceWith(
        Commands.sequence(
          Commands.waitSeconds(1.0).onlyIf(() -> isFirstNote),
          Commands.run(() -> {
            if (controller.canShoot()) simulatedNote = false;
          }).until(() -> simulatedNote == false).onlyIf(() -> RobotBase.isSimulation()),
          controller.setState(RobotStateController.State.SHOOT_SPEAKER).until(() -> !hasNote())
        )
      )
    ).finallyDo(() -> {
      isFirstNote = false;
      swerveDrive.setRotationTargetOverrideFromPointBackwards(null);
    });
  }

  public boolean shouldSeeNote(int note) {
    Translation2d position = Field.NOTE_POSITIONS.get(note).get();
    Translation2d relativePosition = position.minus(swerveDrive.getPose().getTranslation()).rotateBy(swerveDrive.getPose().getRotation().unaryMinus());
    relativePosition = relativePosition.minus(Constants.LIMELIGHT.NOTE_CAMERA_POSITION.toTranslation2d());
    
    if (relativePosition.getX() < 0) return false;

    Rotation2d lateralAngleMin = Rotation2d.fromRadians(Math.atan((relativePosition.getY() - Field.NOTE_LENGTH / 2.0) / relativePosition.getX()));
    Rotation2d lateralAngleMax = Rotation2d.fromRadians(Math.atan((relativePosition.getY() + Field.NOTE_LENGTH / 2.0) / relativePosition.getX()));

    Rotation2d verticalAngle = Rotation2d.fromRadians(Math.atan((Constants.LIMELIGHT.NOTE_CAMERA_POSITION.getZ() - Field.NOTE_THICKNESS / 2.0) / relativePosition.getX()));
    verticalAngle = verticalAngle.plus(Constants.LIMELIGHT.NOTE_CAMERA_PITCH);

    if (verticalAngle.getDegrees() < -(Constants.LIMELIGHT.FOV_HEIGHT.getDegrees() / 2.0) || verticalAngle.getDegrees() > (Constants.LIMELIGHT.FOV_HEIGHT.getDegrees() / 2.0)) return false;
    if (lateralAngleMin.getDegrees() < -(Constants.LIMELIGHT.FOV_WIDTH.getDegrees() / 2.0) || lateralAngleMax.getDegrees() > (Constants.LIMELIGHT.FOV_WIDTH.getDegrees() / 2.0)) return false;
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotState.isAutonomous()) {
      runningCommand.cancel();
      this.cancel();
      return;
    }

    if (state != State.SHOOT && hasNote()) {
      if (runningCommand != null) runningCommand.cancel();
      runningCommand = shoot();
      runningCommand.schedule();
      state = State.SHOOT;
      System.out.println("SHOOTING");
      return;
    }

    if (state != State.PICKUP && !hasNote() && !queuedNotes.isEmpty()) {
      state = State.PICKUP;
      if (runningCommand != null) runningCommand.cancel();
      runningCommand = pickup(Field.NOTE_POSITIONS.get(getClosestNote()).get());
      runningCommand.schedule();
      System.out.println("PICKUP");
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (runningCommand != null) runningCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean hasNote() {
    if (RobotBase.isSimulation()) {
      return simulatedNote;
    } else {
      return controller.hasNote();
    }
  }

  public boolean tooClose(Translation2d notePosition) {
    return swerveDrive.getFuturePose().getTranslation().getDistance(new Translation2d(-notePickupDistance, 0).rotateBy(notePosition.minus(Field.SPEAKER.get().toTranslation2d()).getAngle()).plus(notePosition)) < minPathfindingDistance;
  }
}
