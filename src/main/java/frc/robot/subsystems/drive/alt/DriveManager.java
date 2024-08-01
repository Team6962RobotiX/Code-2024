package frc.robot.subsystems.drive.alt;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.alt.module.RealModule;

public class DriveManager {
    private RealModule[] modules;
    private SwerveDriveKinematics kinematics;

    public DriveManager(SwerveConfig.Module[] equippedModules, SwerveDriveKinematics kinematics) {
        if (equippedModules.length != 4) throw new IllegalArgumentException("Swerve drive must have exactly 4 modules");

        modules = new RealModule[equippedModules.length];

        for (int i = 0; i < equippedModules.length; i++) {
            modules[i] = new RealModule(equippedModules[i], i);
        }

        this.kinematics = kinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < positions.length; i++) {
            positions[i] = modules[i].getMeasuredPosition();
        }

        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < states.length; i++) {
            states[i] = modules[i].getMeasuredState();
        }

        return states;
    }

    public void rawDrive(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules[i].drive(states[i]);
        }
    }

    public void rawDrive(ChassisSpeeds speeds) {
        rawDrive(kinematics.toSwerveModuleStates(speeds));
    }

    public void drive(SwerveModuleState[] states) {
        rawDrive(states); // TODO: Add fancier logic
    }

    public void drive(ChassisSpeeds speeds) {
        drive(kinematics.toSwerveModuleStates(speeds)); // TODO: Add fancier logic, use PID control
    }

    public void stop() {
        for (RealModule module : modules) {
            module.stop();
        }
    }
}