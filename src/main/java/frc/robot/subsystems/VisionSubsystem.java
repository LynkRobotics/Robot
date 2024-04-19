// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class VisionSubsystem extends SubsystemBase {
  private static VisionSubsystem instance;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final Field2d field = new Field2d();
  private double lastEstTimestamp = 0.0;
  private boolean haveTarget = false;
  private boolean haveSpeakerTarget = false;
  private boolean haveAmpTarget = false;
  private boolean haveSourceTarget = false;
  private Pose2d lastPose  = new Pose2d();
  private boolean overrideRotation = false;
  private boolean overrideAmpRotation = false;
  private boolean overrideSourceRotation = false;

  public VisionSubsystem() {
    assert(instance == null);
    instance = this;

    camera = new PhotonCamera(Constants.Vision.cameraName);

    photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.Vision.robotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    overrideRotation = false;
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    SmartDashboard.putData("vision/Field", field);
    SmartDashboard.putBoolean("vision/Update dashboard", true);
  }

  public void enableRotationTargetOverride() { overrideRotation = true; }
  public void disableRotationTargetOverride() { overrideRotation = false; }

  public void enableRotationAmpOverride() { overrideAmpRotation = true; }
  public void disableRotationAmpOverride() { overrideAmpRotation = false; }

  public void enableRotationSourceOverride() { overrideSourceRotation = true; }
  public void disableRotationSourceOverride() { overrideSourceRotation = false; }

  public Optional<Rotation2d> getRotationTargetOverride() {
      if (!overrideRotation || !haveSpeakerTarget()) {
          return Optional.empty();
      }
      Rotation2d adjustment = new Rotation2d(); //Units.degreesToRadians(180.0)); // TODO What about when on Red Alliance?
      Rotation2d rotation = angleToSpeaker().plus(adjustment); // Adjust angle to PathPlanner coordinates
      // System.out.println("Overriding rotation target to be " + rotation.getDegrees());
      return Optional.of(rotation);
  }


  public Optional<Rotation2d> getRotationAmpOverride() {
      if (!overrideAmpRotation || !haveAmpTarget()) {
          return Optional.empty();
      }
      Rotation2d adjustment = new Rotation2d(); //Units.degreesToRadians(180.0)); // TODO What about when on Red Alliance?
      Rotation2d rotation = angleToSpeaker().plus(adjustment); // Adjust angle to PathPlanner coordinates
      // System.out.println("Overriding amp rotation target to be " + rotation.getDegrees());
      return Optional.of(rotation);
  }

  public Optional<Rotation2d> getRotationSpeakerOverride() {
      if (!overrideSourceRotation || !haveSourceTarget()) {
          return Optional.empty();
      }
      Rotation2d adjustment = new Rotation2d(); //Units.degreesToRadians(180.0)); // TODO What about when on Red Alliance?
      Rotation2d rotation = angleToSpeaker().plus(adjustment); // Adjust angle to PathPlanner coordinates
      // System.out.println("Overriding amp rotation target to be " + rotation.getDegrees());
      return Optional.of(rotation);
  }

  public static VisionSubsystem getInstance() {
    return instance;
  }

  public boolean haveTarget() {
    return haveTarget;
  }

  public boolean haveSpeakerTarget() {
    return haveSpeakerTarget;
  }

  public boolean haveAmpTarget() {
    return haveAmpTarget;
  }

  public boolean haveSourceTarget() {
    return haveSourceTarget;
  }

  private Translation2d speakerOffset() {
    return lastPose.getTranslation().minus(speakerLocation());
  }

  public Rotation2d angleToSpeaker() {
    return speakerOffset().getAngle();
  }

  public Rotation2d angleError() {
    if (!haveSpeakerTarget) {
      return new Rotation2d(0.0);
    }

    // TODO Verify operation for Red alliance
    Rotation2d speakerAngle = angleToSpeaker();
    Rotation2d robotAngle = lastPose.getRotation();

    return speakerAngle.minus(robotAngle);
  }

  private Translation2d speakerLocation() {
    return (Robot.isRed() ? Constants.Vision.redSpeakerLocation : Constants.Vision.blueSpeakerLocation);
  }

  // Distance from center of robot to speaker
  public double distanceToSpeakerFromCenter() {
    return lastPose.getTranslation().getDistance(speakerLocation());
  }

  // Distance from edge of robot to speaker 
  public double distanceToSpeakerRaw() {
    double distance = distanceToSpeakerFromCenter();
    distance += Constants.Vision.centerToReferenceOffset; // distance from center of robot to reference point
    return distance;
  }

  public double distanceToSpeaker() {
    boolean isRed = Robot.isRed();
    double distance = distanceToSpeakerFromCenter();

    // Fudge factor based on calibration between two points
    distance *= isRed ? Constants.Vision.calibrationFactorRed : Constants.Vision.calibrationFactorBlue;

    // Distance from center of robot to reference point
    distance += Constants.Vision.centerToReferenceOffset;

    // Fudge amount based on calibration after factor is applied
    distance += isRed ? Constants.Vision.calibrationOffsetRed : Constants.Vision.calibrationOffsetBlue;

    return distance;
  }

  private boolean isSpeakerId(int id) {
    if (Robot.isRed()) {
      return (id == 3 || id == 4);
    } else {
      return (id == 7 || id == 8);
    }
  }

  private boolean isAmpId(int id) {
    if (Robot.isRed()) {
      return (id == 5);
    } else {
      return (id == 6);
    }
  }

  private boolean isSourceId(int id) {
    if (Robot.isRed()) {
      return (id == 9 || id == 10);
    } else {
      return (id == 1 || id == 2);
    }
  }

  public Pose2d lastPose() {
    return lastPose;
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    EstimatedRobotPose visionEst = photonEstimator.update().orElse(null);
    double latestTimestamp = result.getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    boolean updateDashboard = SmartDashboard.getBoolean("vision/Update dashboard", false);

    if (visionEst != null) {
        Pose3d estPose3d = visionEst.estimatedPose;
        lastPose = estPose3d.toPose2d();
        field.setRobotPose(lastPose);
    }

    if (updateDashboard) {
      SmartDashboard.putString("vision/Result", result.toString());
      SmartDashboard.putBoolean("vision/New result", newResult);
      SmartDashboard.putBoolean("vision/Result hasTargets", result.hasTargets());
      SmartDashboard.putNumber("vision/distance", Units.metersToInches(distanceToSpeaker()));
      SmartDashboard.putNumber("vision/Raw distance", Units.metersToInches(distanceToSpeakerRaw()));
      SmartDashboard.putString("vision/Last pose", lastPose.toString());
      SmartDashboard.putString("vision/speakerOffset", speakerOffset().toString());
      SmartDashboard.putNumber("vision/speakerOffset angle", angleToSpeaker().getDegrees());
      SmartDashboard.putNumber("vision/Angle error", angleError().getDegrees());
      // System.out.println("Vision(" + newResult + "," + result.hasTargets() + "): Angle error between speaker @ " + angleToSpeaker().getDegrees() + " and robot @ " + lastPose.getRotation().getDegrees());
    }

    if (newResult) {
      lastEstTimestamp = latestTimestamp;
      haveTarget = result.hasTargets();
      haveSpeakerTarget = false;
      haveAmpTarget = false;
      haveSourceTarget = false;
      if (haveTarget) {
        result.getTargets().forEach((t) -> {
          haveSpeakerTarget = haveSpeakerTarget || isSpeakerId(t.getFiducialId());
          haveAmpTarget = haveAmpTarget || isAmpId(t.getFiducialId());
          haveSourceTarget = haveSourceTarget || isSourceId(t.getFiducialId());
        } );
      }
      if (updateDashboard) {
        SmartDashboard.putBoolean("vision/Have target(s)", haveTarget);
        SmartDashboard.putBoolean("vision/Have speaker target", haveSpeakerTarget);
        SmartDashboard.putBoolean("vision/Have amp target", haveAmpTarget);
        SmartDashboard.putBoolean("vision/Have source target", haveSourceTarget);
      }
    }
  }
}

/* 
 * Calibration procedure:
 *   1. Place the robot, with bumpers, against the subwoofer.  This puts the robot bumper outside edge 36.125 inches from the alliance wall
 *   2. Record the "Raw distance" as measured by vision to speaker in inches as data point (A)
 *   3. Move the robot back 6 feet (72 inches), putting the robot bumper outside edge 108.125 inches from the alliance wall
 *   4. Record the "Raw distance" as measured by vision to speaker in inches as data point (B)
 *   5. Subtract (A) from (B) and divide into 72 to get the "Calibration factor" (i.e., 72.0/(B-A))), and record this as Constants.Vision.calibrationFactorRed/Blue
 *   6. Set Constants.Vision.calibrationOffset to 0.0
 *   7. Build and deploy the updated code to the robot
 *   8. Move the robot bumpers against the subwoofer again
 *   9. Record the "distance" as measured by vision to speaker in inches as data point (C)
 *  10. Subtract (C) from 36.125, and record this as Constants.Vision.calibrationOffsetRed/Blue (i.e., (C) values above 36.125 should result in a negative offset)
 *  11. Build and deploy the updated code to the robot
 *  12. Verify that the "distance" as measured by vision to speaker in inches is approximately 36.125
 *  13. Move the robot back 6 feet (72 inches) again
 *  14. Verify that the "distance" as meansured by vision to speaker in inches is approximately 108.125, or other value measured against reference equipment
 *  15. Optionally compare other distances against values measured against reference equipment for additional verification
 *  16. Use values from vision when calibrating the Shooter subsystem
 * 
 * Repeat for both Red and Blue
 */