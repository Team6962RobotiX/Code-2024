package frc.robot.subsystems.drive.alt.module;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveModule extends Subsystem {
    /**
     * Sets the target module state (wheel speed and orientation).
     * @param state The target module state.
     */
    public void drive(SwerveModuleState state);

    /**
     * Stops the swerve module by setting the target state.
     */
    public void stop();

    /**
     * Gets the target module state (wheel speed and orientation) that was set through the drive() or stop()
     * methods.
     * @return The target module state.
     */
    public SwerveModuleState getTargetState();

    /**
     * Gets the measured module state (wheel speed and orientation), based on sensor data.
     * @return
     */
    public SwerveModuleState getMeasuredState();

    /**
     * Gets the measured module position (angle and distance), based on sensor data.
     * @return
     */
    public SwerveModulePosition getMeasuredPosition();

    public Measure<Voltage> getTotalCurrent();

    /**
     * Run a system identification routine to determine the steer motor's properties.
     * @return The command that will run the system identification routine.
     */
    public Command calibrateSteerMotor();

    /**
     * Run a system identification routine to determine the drive motor's properties.
     * @return The command that will run the system identification routine.
     */
    public Command calibrateDriveMotor();

    /**
     * Gets the path for logging telemetry data for a module.
     * @param corner The corner of the robot that the module is on.
     * @return The path for logging telemetry data (e.g. {@code module-front-left/}).
     */
    public static String getLogPath(int corner) {
        String name;

        if (corner == 0) name = "front-left";
        else if (corner == 1) name = "front-right";
        else if (corner == 2) name = "back-left";
        else if (corner == 3) name = "back-right";
        else throw new IllegalArgumentException("Invalid corner: " + corner);

        return "module-" + name + "/";
    }
}
