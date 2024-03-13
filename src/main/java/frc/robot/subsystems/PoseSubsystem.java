// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseSubsystem extends SubsystemBase {
    private static PoseSubsystem instance;
    private final Swerve s_Swerve;
    private final VisionSubsystem s_Vision;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;

    public PoseSubsystem(Swerve s_Swerve, VisionSubsystem s_Vision) {
        assert(instance == null);
        instance = this;
        
        this.s_Swerve = s_Swerve;
        this.s_Vision = s_Vision;

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());

        field = new Field2d();
        SmartDashboard.putData("Pose", field);

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            s_Swerve::getSpeeds, 
            s_Swerve::driveRobotRelativeAuto,
            // TODO Configure PIDs
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(8.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
                Constants.Swerve.maxSpeed, // Max module speed, in m/s
                Constants.Swerve.driveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            s_Swerve // Reference to Swerve subsystem to set requirements
        );

    }

    public static PoseSubsystem getInstance() {
        return instance;
    }
    
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        setHeading(new Rotation2d());
    }

    public Translation2d speakerLocation() {
        return (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? Constants.Vision.blueSpeakerLocation
                : Constants.Vision.redSpeakerLocation);
    }

    public double distanceToSpeaker() {
        double distance = getPose().getTranslation().getDistance(PoseSubsystem.getInstance().speakerLocation()); // distance from center of robot to speaker 
        distance -= Constants.Vision.centerToReferenceOffset; // distance from center of robot to reference point
        //distance *= Constants.Vision.distanceFudgeFactor;
        return distance;
    }
       
    public Rotation2d dumpShotError() {
        Rotation2d robotAngle = getPose().getRotation();
    
        return Constants.Swerve.dumpAngle.minus(robotAngle);    
    }

    private Translation2d speakerOffset() {
        return speakerLocation().minus(getPose().getTranslation());
    }

    private Rotation2d angleToSpeaker() {
        return speakerOffset().getAngle();
    }

    public Rotation2d angleError() {
        Rotation2d speakerAngle = angleToSpeaker();
        Rotation2d robotAngle = getPose().getRotation();

        return speakerAngle.minus(robotAngle);
    }
    
    @Override
    public void periodic() {
        poseEstimator.update(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions());
        if (DriverStation.isTeleop()) {
            s_Vision.updatePoseEstimate(poseEstimator);
        }
        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }
}