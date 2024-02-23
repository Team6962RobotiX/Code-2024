package frc.robot.commands.autonomous;

import java.io.Console;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Field;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.shooter.Shooter;

public class AutonCommand extends Command {
  private RobotStateController controller;
  private SwerveDrive swerveDrive;
  private List<Integer> notesToGet = List.of();
  private List<Integer> notesThatExist = List.of();

  public AutonCommand(RobotStateController controller, SwerveDrive swerveDrive, List<Integer> notesToGet) {
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
    double radius = Field.NOTE_LENGTH / 2.0 + Math.max(Constants.SWERVE_DRIVE.BUMPER_LENGTH, Constants.SWERVE_DRIVE.BUMPER_WIDTH) / 2.0;
    
    for (Integer note : notesThatExist) {      
      // Define bounding box for each note
      dynamicObstacles.add(
        Pair.of(
          Field.NOTE_POSITIONS[note].minus(new Translation2d(radius, radius)), // example bounding box corner
          Field.NOTE_POSITIONS[note].plus(new Translation2d(radius, radius)) // example bounding box corner
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
    return swerveDrive.getPose().getTranslation().nearest(List.of(Field.SHOT_POSITIONS));
  }

  public Command pickupClosestNote() {
    System.out.println("pickupClosestNote");
    Integer closestNote = getNextClosestNote();
    Translation2d notePosition = Field.NOTE_POSITIONS[closestNote];
    Rotation2d heading = notePosition.minus(swerveDrive.getPose().getTranslation()).getAngle();

    return Commands.runOnce(() -> System.out.println("PICKING UP NOTE"))
      .andThen(() -> notesThatExist.remove(closestNote))
      .andThen(() -> addDynamicObstacles())
      .andThen(() -> swerveDrive.setRotationTargetOverrideFromPoint(notePosition))
      .andThen(swerveDrive.goTo(new Pose2d(notePosition, heading)).until(() -> hasPickedUpNote(closestNote)))
      .andThen(() -> System.out.println("INTAKE"))
      .andThen(controller.setState(RobotStateController.State.INTAKE))
      .andThen(() -> System.out.println("REMOVING NOTE"))
      .andThen(() -> notesToGet.remove(closestNote))
      .andThen(() -> swerveDrive.setRotationTargetOverrideFromPoint(null));
  }

  public Command moveAndShoot() {
    System.out.println("moveAndShoot");
    if ((swerveDrive.getPose().getX() < Field.WING_X && Constants.IS_BLUE_TEAM) || (swerveDrive.getPose().getX() > Field.WING_X && !Constants.IS_BLUE_TEAM)) {
      return controller.setState(RobotStateController.State.PREPARE_SPEAKER)
        .andThen(controller.setState(RobotStateController.State.SHOOT_SPEAKER))
        .andThen(() -> System.out.println("SHOOTING"));
    }
    Translation2d shotPosition = getClosestShootingPoint();
    Rotation2d heading = Field.SPEAKER.toTranslation2d().minus(shotPosition).getAngle();
    return Commands.runOnce(() -> System.out.println("MOVING TO SHOOTING POSITION"))
      .andThen(() -> swerveDrive.setRotationTargetOverrideFromPoint(Field.SPEAKER.toTranslation2d()))
      .andThen(swerveDrive.goTo(new Pose2d(shotPosition, heading)).alongWith(controller.setState(RobotStateController.State.PREPARE_SPEAKER)))
      .andThen(() -> System.out.println("SHOOTING"))
      .andThen(controller.setState(RobotStateController.State.SHOOT_SPEAKER))
      .andThen(() -> swerveDrive.setRotationTargetOverrideFromPoint(null));
  }

  public boolean hasPickedUpNote(int note) {
    return Field.NOTE_POSITIONS[note].getDistance(swerveDrive.getPose().getTranslation()) < Constants.SWERVE_DRIVE.BUMPER_LENGTH;
  }

  public Command pickupAndShootAll() {
    System.out.println("pickupAndShootAll");
    if (notesToGet.isEmpty()) {
      cancel();
      return Commands.runOnce(() -> System.out.println("NO MORE NOTES TO PICKUP"));
    } else {
      return pickupClosestNote().andThen(() -> moveAndShoot().andThen(() -> pickupAndShootAll().schedule()).schedule());
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
