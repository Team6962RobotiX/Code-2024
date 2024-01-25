// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import frc.robot.util.LimelightHelpers;
import frc.robot.util.Logging.Logger;
import frc.robot.Constants;


public class Camera extends SubsystemBase {
  private LimelightHelpers.LimelightResults limelightData;
  private String name;
  private ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");
  private Pose3d botPose; 
  private Pose3d fieldSpace;   
  private Pose3d helpME;
  
  
  private AprilTagFieldLayout tagLayout;
  private VisionSystemSim visionSim = new VisionSystemSim("main");
  private SimCameraProperties cameraProp = new SimCameraProperties();
  private PhotonCamera camera;
  private PhotonCameraSim cameraSim;
  private Supplier<Pose2d> poseSupplier;
  private NetworkTableInstance inst;


  // LimelightHelper Fiducial methods are not static so you need to make an instance of it
  private LimelightHelpers.LimelightTarget_Fiducial LimelightHelperFidcuial = new LimelightHelpers.LimelightTarget_Fiducial();
  //private double lastKnownAprilTagZ = 0.0;

  private void initialize(String name) {
    this.name = name;
    this.camera = new PhotonCamera(name);
    cameraProp.setCalibration(Constants.PHOTON_LIB.CAM_RESOLUTION_WIDTH, Constants.PHOTON_LIB.CAM_RESOLUTION_HEIGHT, Rotation2d.fromDegrees(Constants.PHOTON_LIB.CAM_DIAG_FOV));

    dashboard.addCamera(name, name, "http://" + name + ".local:5800");

    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    }
    catch(Exception ex) {
      DriverStation.reportError(ex.getMessage(), true);
    }
  }
  // real camera
  public Camera(String name) {
    initialize(name);
  }

  //simulated camera
  public Camera(String name, Supplier<Pose2d> poseSupplier) {
    initialize(name);
    this.poseSupplier = poseSupplier;
    this.cameraSim = new PhotonCameraSim(camera, cameraProp);
    visionSim.addAprilTags(tagLayout);
    
    Translation3d robotToCameraTrl = new Translation3d(0, 0, Constants.PHOTON_LIB.CAM_HEIGHT_OFF_GROUND);
    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-Constants.PHOTON_LIB.CAM_PITCH), 0);
    Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

    visionSim.addCamera(cameraSim, robotToCamera);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(false);
  }


  @Override
  public void periodic() {

    //limelightData = LimelightHelpers.getLatestResults(name);
    //botPose = LimelightHelpers.getBotPose3d(name);
    helpME = LimelightHelpers.getCameraPose3d_TargetSpace(name);
    System.out.println("botpose: " + helpME);  
    //fieldSpace = LimelightHelperFidcuial.getRobotPose_FieldSpace();
  
    //Pose3d targetSpace = LimelightHelpers.getCameraPose3d_TargetSpace("light"); // change later
    //Pose3d targetSpaceBottom = LimelightHelpers.getCameraPose3d_TargetSpace("bottom");
    inst = NetworkTableInstance.getDefault();
    NetworkTable inet = inst.getTable("limelight");
    NetworkTableEntry tx = inet.getEntry("tx");
    double x = tx.getDouble(0.0);
    System.out.println("table: " + x);

    //double z = Math.abs(targetSpace.getZ());
    //double z_bottom = Math.abs(targetSpaceBottom.getZ());

    // if (z_top != 0 && z_bottom != 0) {
    //   lastKnownAprilTagZ = (z_top+z_bottom)/2;
    // }else if (z_top != 0){
    //   lastKnownAprilTagZ = z_top;
    // }else if (z_bottom != 0){
    //   lastKnownAprilTagZ = z_bottom;
    // }

    //lastKnownAprilTagZ = z;
  }

  /*public double getLastKnownAprilTagZ() {
    return lastKnownAprilTagZ;
  }
  */

  public LimelightHelpers.Results getTargetingResults() {
    return limelightData.targetingResults;
  }

  public String getName() {
    return name;
  }

  @Override
  public void simulationPeriodic() {
    visionSim.update(poseSupplier.get());
  }
}