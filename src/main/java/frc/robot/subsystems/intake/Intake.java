package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ENABLED_SYSTEMS;
import frc.robot.subsystems.transfer.Transfer;

public class Intake extends SubsystemBase {
    private CANSparkMax motor;
    private Transfer elevator;
    private IntakeState state = IntakeState.OFF;

    public static enum IntakeState {
        FORWARD,
        REVERSE,
        OFF
    }

    private static double INTAKE_SPEED = 0.5;

    public Intake(Transfer elevator) {
        if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
        
        this.elevator = elevator;

        if (RobotBase.isReal()) {
            motor = new CANSparkMax(0, MotorType.kBrushless);
        }
    }


    public void setState(IntakeState newState) {
        state = newState;
    }

    @Override
    public void periodic() {
        if (state != IntakeState.OFF && !elevator.isHoldingNote()) runIntakeMotor(state);
        else runIntakeMotor(IntakeState.OFF);
    }

    private IntakeState previousIntakeState = IntakeState.OFF;
    private void runIntakeMotor(IntakeState state) {
        if (RobotBase.isReal()) {
            if (state == IntakeState.FORWARD) motor.set(INTAKE_SPEED);
            else if (state == IntakeState.REVERSE) motor.set(-INTAKE_SPEED);
            else motor.set(0);
        } else {
            if (previousIntakeState != state) {
                if (state == IntakeState.FORWARD) {
                    System.out.println("Intake motor is now running forward");
                } else if (state == IntakeState.REVERSE) {
                    System.out.println("Intake motor is now running in reverse");
                } else {
                    System.out.println("Intake motor is now stopped");
                }
            }

            previousIntakeState = state;
        }
    }
}
