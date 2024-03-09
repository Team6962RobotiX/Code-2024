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
  private List<Integer> notesToGet = List.of();
  private List<Integer> notesThatExist = List.of();
  private double robotDiagonal = Math.sqrt(Math.pow(Constants.SWERVE_DRIVE.BUMPER_LENGTH, 2.0) + Math.pow(Constants.SWERVE_DRIVE.BUMPER_WIDTH, 2.0));
  private double noteAvoidRadius = (Field.NOTE_LENGTH / 2.0 + Math.max(Constants.SWERVE_DRIVE.BUMPER_LENGTH, Constants.SWERVE_DRIVE.BUMPER_WIDTH) / 2.0);
  private Command command = Commands.runOnce(() -> {});
  private Translation2d visionNotePosition;
  private boolean simHasNote = true;
  private boolean firstNote = true;

  private enum State {
    SHOOT,
    PICKUP
  }

  public State state;

  public Autonomous(RobotStateController controller, SwerveDrive swerveDrive, List<Integer> notesToGet) {
    this.controller = controller;
    this.swerveDrive = swerveDrive;
    this.notesToGet = notesToGet;
    this.notesThatExist = new ArrayList<>(List.of(0, 1, 2, 3, 4, 5, 6, 7));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setState(RobotStateController.State.LEAVE_AMP).schedule();
  }

  public void addDynamicObstacles() {
    List<Pair<Translation2d, Translation2d>> dynamicObstacles = new ArrayList<>();    
    for (Integer note : notesThatExist) {      
      // Define bounding box for each note
      dynamicObstacles.add(
        Pair.of(
          Field.NOTE_POSITIONS.get(note).get().minus(new Translation2d(noteAvoidRadius, noteAvoidRadius)), // example bounding box corner
          Field.NOTE_POSITIONS.get(note).get().plus(new Translation2d(noteAvoidRadius, noteAvoidRadius)) // example bounding box corner
        )
      );
    }
    
    // Set dynamic obstacles
    Pathfinding.setDynamicObstacles(dynamicObstacles, swerveDrive.getPose().getTranslation());
  }

  public void clearDynamicObstacles() {
    Pathfinding.setDynamicObstacles(List.of(), swerveDrive.getPose().getTranslation());
  }

  public Integer getNextClosestNote() {
    if (notesToGet.isEmpty()) return 0;

    List<Integer> closeNotes = notesToGet.stream().filter(n -> n <= 2).collect(Collectors.toList());
    List<Integer> wingNotes = notesToGet.stream().filter(n -> n > 2).collect(Collectors.toList());
    
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

  public Translation2d getClosestShootingPoint() {
    if ((swerveDrive.getPose().getX() < Field.WING_X.get() && Constants.IS_BLUE_TEAM.get()) || (swerveDrive.getPose().getX() > Field.WING_X.get() && !Constants.IS_BLUE_TEAM.get())) {
      return swerveDrive.getPose().getTranslation();
    }

    List<Translation2d> shotPositions = Field.SHOT_POSITIONS.stream().map(Supplier::get).collect(Collectors.toList());

    if (notesToGet.isEmpty()) {
      return swerveDrive.getPose().getTranslation().nearest(shotPositions);
    }
    

    Translation2d closestPoint = Field.SHOT_POSITIONS.get(0).get();
    double bestWeight = Double.MAX_VALUE;
    for (Translation2d position : shotPositions) {
      double weight = swerveDrive.getPose().getTranslation().getDistance(position) + 
        position.getDistance(Field.NOTE_POSITIONS.get(getNextClosestNote()).get());
      if (weight < bestWeight) {
        bestWeight = weight;
        closestPoint = position;
      }
    }
    return closestPoint;
  }

  public Translation2d getRealNotePosition(Translation2d theoreticalPosition) {
    List<Translation2d> measuredNotePositions = Notes.getNotePositions(LIMELIGHT.NOTE_CAMERA_NAME, LIMELIGHT.NOTE_CAMERA_PITCH, swerveDrive, swerveDrive.getFieldVelocity(), LIMELIGHT.NOTE_CAMERA_POSITION);

    if (RobotBase.isSimulation()) {
      measuredNotePositions = new ArrayList<>();
      for (Integer note : List.of(0, 1, 2, 3, 4, 5, 6, 7)) {
        if (shouldSeeNote(note)) {
          measuredNotePositions.add(Field.NOTE_POSITIONS.get(note).get().plus(new Translation2d(0.0, 0.0)));
        }
      }
    }

    // FieldObject2d visibleNotes = SwerveDrive.getField().getObject("visibleNotes");
    // List<Pose2d> poses = new ArrayList<>();

    if (measuredNotePositions.size() == 0) return null;

    List<Translation2d> theoreticalNotePositions = Field.NOTE_POSITIONS.stream().map(Supplier::get).collect(Collectors.toList());
    
    for (Translation2d measuredNotePosition : measuredNotePositions) {
      if ((measuredNotePosition.getX() > Field.LENGTH / 2.0 && Constants.IS_BLUE_TEAM.get()) || (measuredNotePosition.getX() < Field.LENGTH / 2.0 && !Constants.IS_BLUE_TEAM.get())) {
        Translation2d relativeNotePosition = measuredNotePosition.minus(swerveDrive.getPose().getTranslation());
        relativeNotePosition = relativeNotePosition.div(-relativeNotePosition.getX());
        relativeNotePosition = relativeNotePosition.times(swerveDrive.getPose().getTranslation().getX() - Field.LENGTH / 2);
        measuredNotePosition = relativeNotePosition.plus(swerveDrive.getPose().getTranslation());
      }
      // poses.add(new Pose2d(measuredNotePosition, new Rotation2d()));
      Translation2d theoreticalNoteCounterpart = measuredNotePosition.nearest(theoreticalNotePositions);
      if (theoreticalNoteCounterpart.getDistance(theoreticalPosition) < 0.05 && swerveDrive.getPose().getTranslation().getDistance(measuredNotePosition) < 3.0) {
        return measuredNotePosition;
      }
    }

    // visibleNotes.setPoses(poses);

    return null;
  }

  public Command pickupNote(Translation2d notePosition) {
    if (notePosition == null) return Commands.runOnce(() -> {});

    Rotation2d heading = notePosition.minus(swerveDrive.getPose().getTranslation()).getAngle();
    Translation2d middleSpline = Field.SPEAKER.get().toTranslation2d().minus(notePosition);
    Rotation2d speakerNoteAngle = middleSpline.getAngle();
    middleSpline = middleSpline.div(middleSpline.getNorm()).times(Constants.SWERVE_DRIVE.BUMPER_LENGTH / 2.0 + Field.NOTE_LENGTH * 1.0);
    middleSpline = middleSpline.plus(notePosition);

    Translation2d endSpline = Field.SPEAKER.get().toTranslation2d().minus(notePosition);
    endSpline = endSpline.div(endSpline.getNorm()).times(Constants.SWERVE_DRIVE.BUMPER_LENGTH / 2.0 - Field.NOTE_LENGTH / 2.0);
    endSpline = endSpline.plus(notePosition);
    
    Command pathplannerCommand = Commands.runOnce(() -> {});

    boolean pathfind = false;
    if (Math.abs(swerveDrive.getPose().getX() - notePosition.getX()) > Field.NOTE_LENGTH * 2.0) {
      if (swerveDrive.getPose().getTranslation().getDistance(notePosition) <= noteAvoidRadius + 0.3) {
        pathfind = false;
      }
      pathfind = true;
    }

    // pathfind = false;

    if (pathfind) {
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(middleSpline, speakerNoteAngle.plus(Rotation2d.fromDegrees(180.0))),
        new Pose2d(endSpline, speakerNoteAngle.plus(Rotation2d.fromDegrees(180.0)))
      );

      PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
        new GoalEndState(
          0.0,
          swerveDrive.getHeading(),
          true
        )
      );
      
      pathplannerCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
      );

      if (middleSpline.getDistance(swerveDrive.getPose().getTranslation()) < 1.0) {
        pathplannerCommand = AutoBuilder.followPath(path);
      }
    } else {
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(swerveDrive.getPose().getTranslation(), Field.SPEAKER.get().toTranslation2d().minus(swerveDrive.getPose().getTranslation()).getAngle()),
        new Pose2d(middleSpline, speakerNoteAngle.plus(Rotation2d.fromDegrees(180.0))),
        new Pose2d(endSpline, speakerNoteAngle.plus(Rotation2d.fromDegrees(180.0)))
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

      pathplannerCommand = AutoBuilder.followPath(path);
      // Commands.sequence(
      //   // Commands.runOnce(() -> System.out.println("START PATHFINDING")),
      //   Commands.runOnce(() -> swerveDrive.setRotationTargetOverrideFromPointBackwards(Field.SPEAKER.get().toTranslation2d())),
      //   AutoBuilder.followPath(path)
      //   // Commands.runOnce(() -> System.out.println("DONE PATHFINDING"))
      // ).finallyDo(() -> swerveDrive.setRotationTargetOverrideFromPoint(null));
      
      // pathplannerCommand = swerveDrive.goToSimple(new Pose2d(notePosition, new Rotation2d()));
    }

    if (getRealNotePosition(Field.NOTE_POSITIONS.get(getNextClosestNote()).get()) != null) {
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(swerveDrive.getPose().getTranslation(), swerveDrive.getFieldVelocity().getAngle()),
        new Pose2d(endSpline, speakerNoteAngle.plus(Rotation2d.fromDegrees(180.0)))
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

      pathplannerCommand = AutoBuilder.followPath(path);
    }

    Integer closestNote = getNextClosestNote();

    return Commands.sequence(
      Commands.runOnce(() -> {
        // System.out.println("START");
        addDynamicObstacles();
        swerveDrive.setRotationTargetOverrideFromPointBackwards(Field.SPEAKER.get().toTranslation2d());
        // System.out.println("START 229");
      }),
      pathplannerCommand
        .raceWith(Commands.sequence(
          Commands.waitUntil(() -> swerveDrive.getPose().getTranslation().getDistance(notePosition) < robotDiagonal + Field.NOTE_LENGTH),
          controller.setState(RobotStateController.State.INTAKE)
        ))
        .raceWith(Commands.waitUntil(() -> hasNote())),
      Commands.runOnce(() -> {        
        if (swerveDrive.getPose().getTranslation().getDistance(notePosition) < Field.NOTE_LENGTH + Constants.SWERVE_DRIVE.BUMPER_LENGTH / 2.0) {
          // simHasNote = true;
        }
        notesThatExist.remove(closestNote);
        notesToGet.remove(closestNote);
        visionNotePosition = null;
        // addDynamicObstacles();
        // System.out.println("STOP");
      })
    ).finallyDo(
      () -> swerveDrive.setRotationTargetOverrideFromPoint(null)
    );
    
    // return Commands.sequence(
    //   Commands.runOnce(() -> swerveDrive.setRotationTargetOverrideFromPoint(notePosition)),
    //   swerveDrive.goTo(new Pose2d(pathfindPosition, swerveDrive.getHeading())),
    //   Commands.runOnce(() -> addDynamicObstacles()),
    //   controller.setState(RobotStateController.State.INTAKE).onlyIf(() -> RobotBase.isReal())
    //     .alongWith(swerveDrive.goTo(new Pose2d(notePosition, swerveDrive.getHeading())))
    //     .until(() -> hasNote()),
    //   Commands.runOnce(() -> controller.setState(RobotStateController.State.CENTER_NOTE)),
    //   Commands.runOnce(() -> swerveDrive.setRotationTargetOverrideFromPoint(null))
    // );
  }

  public Command moveAndShoot() {
    if (!hasNote()) {
      return Commands.runOnce(() -> {});
    }

    Translation2d shotPosition = getClosestShootingPoint();
    Rotation2d heading = Field.SPEAKER.get().toTranslation2d().minus(shotPosition).getAngle().plus(Rotation2d.fromDegrees(180.0));
    Command moveToCommand = swerveDrive.goTo(new Pose2d(shotPosition, heading));
    if (shotPosition.getDistance(swerveDrive.getPose().getTranslation()) < 3.0) {
      moveToCommand = Commands.runOnce(() -> {});
    }
    return Commands.sequence(
      Commands.runOnce(() -> {
        clearDynamicObstacles();
        swerveDrive.setRotationTargetOverrideFromPointBackwards(Field.SPEAKER.get().toTranslation2d());
      }),
      Commands.parallel(
        moveToCommand
          .until(() -> !controller.underStage() && swerveDrive.getFuturePose().getTranslation().getDistance(Field.SPEAKER.get().toTranslation2d()) < 4)
          .onlyIf(() -> !firstNote),
        Commands.parallel(
          controller.setState(RobotStateController.State.SPIN_UP),
          controller.setState(RobotStateController.State.AIM_SPEAKER)
        ).until(() -> controller.canShoot() && swerveDrive.getFieldVelocity().getNorm() < 0.5)//,
        // controller.setState(RobotStateController.State.CENTER_NOTE).onlyIf(() -> RobotBase.isReal() && swerveDrive.getPose().getTranslation().getDistance(Field.SPEAKER.get().toTranslation2d()) > 4.5 && hasNote())
      ),
      Commands.waitSeconds(0.5).onlyIf(() -> firstNote),
      controller.setState(RobotStateController.State.SHOOT_SPEAKER)
        .alongWith(Commands.runOnce(() -> {simHasNote = false; System.out.println("SHOOTING IN SPEAKER");}))
        .until(() -> !hasNote()),
      Commands.runOnce(() -> {
        swerveDrive.setRotationTargetOverrideFromPointBackwards(null);
        firstNote = false;
      })
    );
  }

  // public boolean hasPickedUpNote(Translation2d notePosition) {
  //   Translation2d futurePosition = swerveDrive.getPose().getTranslation();
  //   futurePosition = futurePosition.plus(swerveDrive.getFieldVelocity().times(swerveDrive.getFieldVelocity().getNorm()).div(2.0 * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION));
  //   SwerveDrive.getField().getObject("futurePosition").setPoses(List.of(new Pose2d(futurePosition, new Rotation2d())));
  //   return notePosition.getDistance(futurePosition) < Constants.SWERVE_DRIVE.BUMPER_LENGTH / 2.0;
  // }

  public boolean shouldSeeNote(int note) {
    Translation2d position = Field.NOTE_POSITIONS.get(note).get();
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
    // getRealNotePosition(Field.SPEAKER.get().toTranslation2d());
    // System.out.println(controller.canShoot());
    // FieldObject2d visibleNotes = SwerveDrive.getField().getObject("visibleNotes");
    // List<Pose2d> poses = new ArrayList<>();
    // for (Integer note : List.of(0, 1, 2, 3, 4, 5, 6, 7)) {
    //   if (shouldSeeNote(note)) {
    //     poses.add(new Pose2d(Field.NOTE_POSITIONS.get(note).get().plus(new Translation2d(0.0, 0.0)), new Rotation2d()));
    //   }
    // }
    // visibleNotes.setPoses(poses);

    // Translation2d futurePosition = swerveDrive.getPose().getTranslation();
    // futurePosition = futurePosition.plus(swerveDrive.getFieldVelocity().times(swerveDrive.getFieldVelocity().getNorm()).div(2.0 * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION));
    // SwerveDrive.getField().getObject("futurePosition").setPoses(List.of(new Pose2d(futurePosition, swerveDrive.getPose().getRotation())));
    
    if (!command.isScheduled()) {
      if (state == State.PICKUP || (state == null && hasNote())) {
        System.out.println("SHOOT");
        command = moveAndShoot();
        command.schedule();
        state = State.SHOOT;
        return;
      }
      if ((state == State.SHOOT || (state == null && !hasNote())) && !notesToGet.isEmpty()) {
        System.out.println("PICKUP");
        state = State.PICKUP;
        command = pickupNote(Field.NOTE_POSITIONS.get(getNextClosestNote()).get());
        command.schedule();
        return;
      }
    }

    if (state == State.PICKUP) {
      Translation2d measuredVisionNotePosition = getRealNotePosition(Field.NOTE_POSITIONS.get(getNextClosestNote()).get());
      if (measuredVisionNotePosition != null) {
        if (visionNotePosition == null || measuredVisionNotePosition.getDistance(visionNotePosition) > Field.NOTE_LENGTH / 2.0) {
          visionNotePosition = measuredVisionNotePosition;
          command.cancel();
          command = pickupNote(visionNotePosition);
          command.schedule();
          return;
        }
      }
    }

    if (notesToGet.isEmpty() && state == State.SHOOT && !command.isScheduled()) {
      command.cancel();
      this.cancel();
      return;
    }
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

  public boolean hasNote() {
    // return false;
    if (RobotBase.isReal()) {
      return controller.hasNote();
    } else {
      return simHasNote;
    }
  }
}
