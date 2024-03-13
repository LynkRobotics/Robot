// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
  private static VisionSubsystem instance;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final Field2d field = new Field2d();
  private double lastEstTimestamp = 0.0;
  private boolean haveTarget = false;
  private Pose2d lastPose  = new Pose2d();
  private boolean updateDashboard = true;

  private final Transform3d kRobotToCam =
    new Transform3d(
        new Translation3d(Units.inchesToMeters(6.0), 0.0, Units.inchesToMeters(13.5)),
        new Rotation3d(0, Units.degreesToRadians(-31.7), 0));

  public VisionSubsystem() {
    assert(instance == null);
    instance = this;

    camera = new PhotonCamera(Constants.Vision.cameraName);

    photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    SmartDashboard.putData("vision/Field", field);
    SmartDashboard.putBoolean("vision/Update dashboard", updateDashboard);
  }

  public static VisionSubsystem getInstance() {
    return instance;
  }

  public boolean haveTarget() {
    return haveTarget;
  }

  private Translation2d speakerOffset() {
    return PoseSubsystem.getInstance().speakerLocation().minus(lastPose.getTranslation());
  }

  private Rotation2d angleToSpeaker() {
    return speakerOffset().getAngle();
  }

  public Rotation2d angleError() {
    if (!haveTarget) {
      return new Rotation2d(0.0);
    }

    Rotation2d speakerAngle = angleToSpeaker();
    Rotation2d robotAngle = lastPose.getRotation();

    return speakerAngle.minus(robotAngle);
  }

  public double distanceToSpeaker() {
    double distance = lastPose.getTranslation().getDistance(PoseSubsystem.getInstance().speakerLocation()); // distance from center of robot to speaker
    distance -= Constants.Vision.centerToReferenceOffset; // distance from center of robot to reference point
    distance *= Constants.Vision.distanceFudgeFactor;
    return distance;
  }

  public boolean updatePoseEstimate(PoseEstimator<SwerveDriveWheelPositions> poseEstimator) {
    Optional<EstimatedRobotPose> optVisionEst = photonEstimator.update();
    EstimatedRobotPose visionEst;
    double latestTimestamp;
    boolean newResult;
    
    if (!optVisionEst.isPresent()) {
      return false;
    }
    visionEst = optVisionEst.get();
    latestTimestamp = visionEst.timestampSeconds;
    newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (updateDashboard) {
      SmartDashboard.putBoolean("vision/New result", newResult);
    }
    if (!newResult) {
      return false;
    }
    lastEstTimestamp = latestTimestamp;
    lastPose = visionEst.estimatedPose.toPose2d();
    field.setRobotPose(lastPose);
    poseEstimator.addVisionMeasurement(lastPose, lastEstTimestamp);
    return true;
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    haveTarget = result.hasTargets();
    updateDashboard = SmartDashboard.getBoolean("vision/Update dashboard", false);

    if (updateDashboard) {
      SmartDashboard.putString("vision/Result", result.toString());
      SmartDashboard.putBoolean("vision/Have target(s)", haveTarget);
      SmartDashboard.putNumber("vision/distance", Units.metersToInches(distanceToSpeaker()));
      SmartDashboard.putString("vision/Last pose", lastPose.toString());
      SmartDashboard.putString("vision/speakerOffset", speakerOffset().toString());
      SmartDashboard.putNumber("vision/speakerOffset angle", angleToSpeaker().getDegrees());
      SmartDashboard.putNumber("vision/Angle error", angleError().getDegrees());
    }
  }
}