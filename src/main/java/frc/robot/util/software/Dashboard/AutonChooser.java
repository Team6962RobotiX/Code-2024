package frc.robot.util.software.Dashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.robot.Constants.Field;

public final class AutonChooser {
  public static ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
  public static GenericEntry[] notes = 
    IntStream.range(0, Field.NOTE_POSITIONS.length)
    .mapToObj(i -> tab.add("Note " + i, false).withWidget(BuiltInWidgets.kToggleButton).getEntry())
    .toArray(GenericEntry[]::new);

  public static List<Integer> getNotes() {
    List<Integer> notesToGet = new ArrayList<>();
    for (int i = 0; i < Field.NOTE_POSITIONS.length; i++) {
      if (AutonChooser.notes[i].getBoolean(false)) {
        notesToGet.add(i);
      }
    }
    return notesToGet;
  }
}
