// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import javax.security.auth.login.LoginException;

import javax.security.auth.login.LoginException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import org.photonvision.EstimatedRobotPose;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
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
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.Constants.PHOTON_LIB;


public class Camera extends SubsystemBase {
  private LimelightHelpers.LimelightResults limelightData;
  private String name;
  private ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");
  private Pose3d helpME;
  
  
  private AprilTagFieldLayout tagLayout;
  private VisionSystemSim visionSim = new VisionSystemSim("main");
  private SimCameraProperties cameraProp = new SimCameraProperties();
  private PhotonCamera camera;
  private PhotonPipelineResult latestResult;
  private PhotonCameraSim cameraSim;
  private Supplier<Pose2d> poseSupplier;
  private SwerveDrive sDrive;
  private NetworkTableInstance inst;
  private NetworkTable inet;
  private PhotonPoseEstimator photonPoseEstimator;
  private Pose2d previousPose;


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
  }
  // real camera
  public Camera(String name) {
    initialize(name);
  }

  //simulated camera
  public Camera(String name, Supplier<Pose2d> poseSupplier, SwerveDrive sDrive) {
    initialize(name);
    this.poseSupplier = poseSupplier;
    this.sDrive = sDrive;
    this.cameraSim = new PhotonCameraSim(camera, cameraProp);
    visionSim.addAprilTags(tagLayout);
    
    Translation3d robotToCameraTrl = new Translation3d(0, 0, Constants.PHOTON_LIB.CAM_HEIGHT_OFF_GROUND);
    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-Constants.PHOTON_LIB.CAM_PITCH), 0);
    Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

    visionSim.addCamera(cameraSim, robotToCamera);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(false);

    photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCamera);

  }


  @Override
  public void periodic() {
    helpME = LimelightHelpers.getCameraPose3d_TargetSpace(name);
    System.out.println("botpose: " + helpME);  
    inst = NetworkTableInstance.getDefault();
    NetworkTable inet = inst.getTable("limelight");
    NetworkTableEntry ty = inet.getEntry("ty");
    NetworkTableEntry tx = inet.getEntry("tx");
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    System.out.println("Ty: " + y);
    System.out.println("Tx: " + x);
    System.out.println("hi");

    if (y != 0.0){
      //2.51 meters
      System.out.println("Distance: " + 0.79*Math.tan((90-y)*Math.PI/180));
    }

   
  }

  public LimelightHelpers.Results getTargetingResults() {
    return limelightData.targetingResults;
  }

  public double getNoteDist(){
    double y = getTY();
    if (y != 0.0) {
      return Constants.PHOTON_LIB.CHAIR_HEIGHT_OFF_GROUND*Math.tan(Math.PI/2+y*Math.PI/180);
    }
    else {
      return 0;
    }

  }

  public double getTY(){
    NetworkTableEntry ty = inet.getEntry("ty");
    return ty.getDouble(0.0);
  }

  public double getTX(){
    NetworkTableEntry tx = inet.getEntry("tx");
    return tx.getDouble(0.0);

  }

  public String getName() {
    return name;
  }

  @Override
  public void simulationPeriodic() {
    visionSim.update(poseSupplier.get());

    previousPose = poseSupplier.get();

    //Logger.log("vision/getBestTargetDist", getBestTargetDist());

    try {

      latestResult = camera.getLatestResult();
      System.out.println(camera.getLatestResult());

      System.out.println(latestResult);

      boolean hasTargets = latestResult.hasTargets();

      if (hasTargets) {
        Optional<EstimatedRobotPose> poseRaw = getEstimatedGlobalPose(previousPose);

        if (poseRaw.isPresent()) {
          EstimatedRobotPose position = poseRaw.get();
          Pose3d pose = position.estimatedPose;

          Pose2d pose2d = pose.toPose2d();
          System.out.println(pose2d);
          
          sDrive.addVisionMeasurement(pose2d);
        }
      }
    }
      catch(Exception e){
        System.out.println("null");

      }

      System.out.println(poseSupplier.get());
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

  public int getFiducialId() {
    if (latestResult.hasTargets()){
      PhotonTrackedTarget target = latestResult.getBestTarget();

      int targetID = target.getFiducialId();

      return targetID;
    }
    else {
      return 0;
    }
  }
  

   public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
   }
  
}


