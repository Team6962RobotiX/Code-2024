package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {
    public static enum NoteDestination {
        WAIT,
        SHOOTER,
        AMP
    }

    public Transfer() {
        super();
    }

    public void setNoteDestination(NoteDestination destination) {
        if (destination == NoteDestination.WAIT) {
            // Hold the note in the elevator
        } else if (destination == NoteDestination.SHOOTER) {
            // Move the note to the shooter
        } else if (destination == NoteDestination.AMP) {
            // Move the note to the amp
        }
    }

    public boolean isHoldingNote() {
        // Return whether the elevator is holding a note
        return false;
    }
}
