package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class XBoxIntake extends Command {
    Intake intake;
    XboxController controller;

    public XBoxIntake(Intake intake, XboxController controller) {
        super();

        this.intake = intake;
        this.controller = controller;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (controller.getBButtonPressed()) {
            intake.setState(Intake.IntakeState.FORWARD);
        } else {
            intake.setState(Intake.IntakeState.OFF);
        }
    }
}
