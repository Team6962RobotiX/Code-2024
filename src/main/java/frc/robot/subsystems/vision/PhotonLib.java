// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import org.opencv.core.Mat;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.kauailabs.navx.frc.Quaternion;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;
import frc.robot.Constants;


public class PhotonLib extends SubsystemBase {

  VisionSystemSim visionSim = new VisionSystemSim("main");
  AprilTagFieldLayout tagLayout;
  SimCameraProperties cameraProp = new SimCameraProperties();
  PhotonCamera camera;
  PhotonCameraSim cameraSim;
  Supplier<Pose2d> pose;

  public Pose3d translate(double[] pose){

      double [][] arr = new double[4][4];

      for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
          arr[i][j] = pose[i*4+j];
        }
      }


      double roll = Math.atan(arr[1][0]/arr[0][0]);
      double pitch = Math.atan(-arr[2][0]/Math.sqrt(Math.pow(arr[2][1],2)+Math.pow(arr[2][2],2)));
      double yaw = Math.atan(arr[2][1]/arr[2][2]);
      

      Rotation3d x = new Rotation3d(roll, pitch, yaw);

      return new Pose3d(arr[0][3],arr[1][3],arr[2][3],x);


    }
  /** Creates a new ExampleSubsystem. */
  public PhotonLib(Supplier<Pose2d> poseSupplier) {
    pose = poseSupplier;
    Path aprilTagPath = Filesystem.getDeployDirectory().toPath().resolve("corrected_apriltags_coordinates.json");

    double[] nums = {-0.5,
      -0.866025,
      0,
      6.808597,
      0.866025,
      -0.5,
      0,
      -3.859403,
      0,
      0,
      1,
      1.355852,
      0,
      0,
      0,
      1};
   
    //System.out.print(translate(nums));
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    }
    catch(Exception e) {
      System.out.println("");
      System.out.print(e);
    }

    System.out.println(tagLayout);
   
    visionSim.addAprilTags(tagLayout);

    cameraProp.setCalibration(Constants.PHOTON_LIB.CAM_RESOLUTION_WIDTH, Constants.PHOTON_LIB.CAM_RESOLUTION_HEIGHT, Rotation2d.fromDegrees(Constants.PHOTON_LIB.CAM_DIAG_FOV));

    camera = new PhotonCamera("cameraName");

    cameraSim = new PhotonCameraSim(camera, cameraProp);
    
    Translation3d robotToCameraTrl = new Translation3d(0, 0, Constants.PHOTON_LIB.CAM_HEIGHT_OFF_GROUND);
    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-Constants.PHOTON_LIB.CAM_PITCH), 0);
    Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

    // Add this camera to the vision system simulation with the given robot-to-camera transform.
    visionSim.addCamera(cameraSim, robotToCamera);

    // Enable the raw and processed streams. These are enabled by default.
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    cameraSim.enableDrawWireframe(false);

  }


  @Override
  public void periodic() {
    visionSim.update(pose.get());
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
