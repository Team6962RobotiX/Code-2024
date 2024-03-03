package frc.robot.util.software.Dashboard;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Field;
import frc.robot.util.software.Logging.StatusChecks;

public final class AutonChooser {
  public static ShuffleboardTab tab = Shuffleboard.getTab("Autonomous"); 
  public static List<SimpleWidget> notes = new ArrayList<>();

  public static List<Integer> getNotes() {

    if (tab == null) {
      init();
    }

    List<Integer> notesToGet = new ArrayList<>();
    for (int i = 0; i < Field.NOTE_POSITIONS.length; i++) {
      if (notes.get(i).getEntry().getBoolean(false)) {
        notesToGet.add(i);
      }
    }
    return notesToGet;
  }

  public static void init() {
    for (int i = 0; i < Field.NOTE_POSITIONS.length; i++) {
      int row = i;
      int column = 0;
      if (i > 2) {
        row = i - 3;
        column = 1;
      }
      notes.add(tab.add("Note " + i, false).withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(column, row));
    }
  }
}
