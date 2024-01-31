// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Set;
import java.util.function.Supplier;

import javax.security.auth.login.LoginException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.util.LimelightHelpers;
import frc.robot.util.Logging.Logger;
import frc.robot.Constants;
import frc.robot.Constants.PHOTON_LIB;
import java.util.List;
import java.util.Optional;


public class Camera extends SubsystemBase {
  private LimelightHelpers.LimelightResults limelightData;
  private String name;
  private ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");
  // private Pose3d helpME;
  
  
  private AprilTagFieldLayout tagLayout;
  private VisionSystemSim visionSim = new VisionSystemSim("main");
  private SimCameraProperties cameraProp = new SimCameraProperties();
  private PhotonCamera camera;
  private PhotonPipelineResult latestResult;
  private PhotonCameraSim cameraSim;
  private Supplier<Pose2d> poseSupplier;
  private NetworkTableInstance inst;
  PhotonPoseEstimator photonPoseEstimator;


  // LimelightHelper Fiducial methods are not static so you need to make an instance of it
  private LimelightHelpers.LimelightTarget_Fiducial LimelightHelperFidcuial = new LimelightHelpers.LimelightTarget_Fiducial();
  //private double lastKnownAprilTagZ = 0.0;

  // Test vision target

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

    botPose = Constants.SWERVE_DRIVE.STARTING_POSE_3D;
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
    photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCamera);


    visionSim.addCamera(cameraSim, robotToCamera);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(false);
  }


  @Override
  public void periodic() {
<<<<<<< Updated upstream
    // helpME = LimelightHelpers.getCameraPose3d_TargetSpace(name);
    // System.out.println("botpose: " + helpME);  
    inst = NetworkTableInstance.getDefault();
    NetworkTable inet = inst.getTable("limelight");
    NetworkTableEntry tx = inet.getEntry("tx");
    double x = tx.getDouble(0.0);
    System.out.println("table: " + x);

    latestResult = camera.getLatestResult();
  }

=======

    


    setEstimatedGlobalPose();



    //Forward Camera

    // Construct PhotonPoseEstimator

    //limelightData = LimelightHelpers.getLatestResults(name);
    //botPose = LimelightHelpers.getBotPose3d(name);
    // helpME = LimelightHelpers.getCameraPose3d_TargetSpace(name);
    // System.out.println("botpose: " + helpME);  
    // //fieldSpace = LimelightHelperFidcuial.getRobotPose_FieldSpace();
  
    // //Pose3d targetSpace = LimelightHelpers.getCameraPose3d_TargetSpace("light"); // change later
    // //Pose3d targetSpaceBottom = LimelightHelpers.getCameraPose3d_TargetSpace("bottom");
    // inst = NetworkTableInstance.getDefault();
    // NetworkTable inet = inst.getTable("limelight");
    // NetworkTableEntry tx = inet.getEntry("tx");
    // double x = tx.getDouble(0.0);
    // System.out.println("table: " + x);

    // //double z = Math.abs(targetSpace.getZ());
    // //double z_bottom = Math.abs(targetSpaceBottom.getZ());
    // latestResult = camera.getLatestResult();

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

  public int getFiducialId(){
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    
    List<PhotonTrackedTarget> targets = result.getTargets();

    PhotonTrackedTarget target = result.getBestTarget();

    if (hasTargets) {
      return target.getFiducialId();
    }
    else {
      return 0;
    }
  }

  public void setEstimatedGlobalPose() {
    photonPoseEstimator.setReferencePose(botPose);
    Optional<EstimatedRobotPose> raw = photonPoseEstimator.update();
    if (raw.isPresent()){
      EstimatedRobotPose poseList = raw.get();
      Pose3d pose = poseList.estimatedPose;
      botPose = pose;
      //System.out.println(pose);

    }
  }

  public Pose3d getPose(){
    return botPose;
  }


>>>>>>> Stashed changes
  public LimelightHelpers.Results getTargetingResults() {
    return limelightData.targetingResults;
  }

  public String getName() {
    return name;
  }

  @Override
  public void simulationPeriodic() {
    visionSim.update(poseSupplier.get());
    Logger.log("vision/getBestTargetDist", getBestTargetDist());
  }

  public PhotonTrackedTarget getBestTarget() {
    if (latestResult.hasTargets()) {
      return latestResult.getBestTarget();
    }
    return null;
  }

  public double getBestTargetDist() {
    if (latestResult.hasTargets()) {
      return PhotonUtils.calculateDistanceToTargetMeters(
        PHOTON_LIB.CAM_HEIGHT_OFF_GROUND,
        tagLayout.getTagPose(getBestTarget().getFiducialId()).get().getZ(),
        PHOTON_LIB.CAM_PITCH,
        Units.degreesToRadians(getBestTarget().getPitch())
      );
    }
    return -1.0;
  }
}