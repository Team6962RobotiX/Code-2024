package frc.robot.subsystems.drive.alt.field;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Field extends SubsystemBase {
    private Field2d field = new Field2d();
    private List<FieldElement> elements = new ArrayList<>();

    public Field() {
        SmartDashboard.putData("Field", field);
    }

    public void addElement(FieldElement element) {
        elements.add(element);
    }

    public void removeElement(FieldElement element) {
        elements.remove(element);
    }

    @Override
    public void periodic() {
        for (FieldElement element : elements) {
            element.updateFieldElement(field);
        }
    }
}
