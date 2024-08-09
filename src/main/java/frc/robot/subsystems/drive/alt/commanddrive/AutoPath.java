package frc.robot.subsystems.drive.alt.commanddrive;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.drive.alt.field.FieldElement;

public class AutoPath implements FieldElement {
    private Pose2d targetPose;
    private List<Pose2d> path;

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    public void setActivePath(List<Pose2d> path) {
        this.path = path;
    }

    @Override
    public void updateFieldElement(Field2d field) {
        if (targetPose != null) {
            field.getObject("Target Pose").setPose(targetPose);
        }

        if (path != null) {
            field.getObject("Active Path").setPoses(path);
        }
    }
}