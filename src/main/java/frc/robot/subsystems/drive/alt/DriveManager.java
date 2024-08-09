package frc.robot.subsystems.drive.alt;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.alt.module.RealModule;
import frc.robot.subsystems.drive.alt.module.SimulatedModule;
import frc.robot.subsystems.drive.alt.module.SwerveModule;
import frc.robot.subsystems.drive.alt.pose.Gyroscope;

public class DriveManager implements Gyroscope.Observer {
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private PIDController headingPID;

    public DriveManager(SwerveConfig swerveConfig, SwerveDriveKinematics kinematics) {
        if (swerveConfig.equippedModules().length != 4) throw new IllegalArgumentException("Swerve drive must have exactly 4 modules");

        modules = new SwerveModule[swerveConfig.equippedModules().length];

        for (int i = 0; i < swerveConfig.equippedModules().length; i++) {
            modules[i] = RobotBase.isSimulation() ? new SimulatedModule(swerveConfig, i) : new RealModule(swerveConfig.equippedModules()[i], i);
        }

        headingPID = new PIDController(
            swerveConfig.headingPID().kP(),
            swerveConfig.headingPID().kI(),
            swerveConfig.headingPID().kD()
        );

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
        headingPID.setSetpoint(speeds.omegaRadiansPerSecond);
        // TODO: IEI)#JIEWJIOJDONDJINEJHCDNHJNCHJDNCHJNSHJ PROBLEM
    }

    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    @Override
    public void onGyroscopeReset(Rotation2d offsetChange) {
        
    }
}