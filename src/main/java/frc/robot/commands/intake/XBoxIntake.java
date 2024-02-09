package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeWheels;

public class XBoxIntake extends Command {
    IntakeWheels intake;
    XboxController controller;

    public XBoxIntake(IntakeWheels intake, XboxController controller) {
        super();

        this.intake = intake;
        this.controller = controller;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (controller.getBButtonPressed()) {
            intake.setState(IntakeWheels.IntakeState.FORWARD);
        } else {
            intake.setState(IntakeWheels.IntakeState.OFF);
        }
    }
}
