// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Pose;
import frc.robot.Robot;

public class PoseSubsystem extends SubsystemBase {
    private static PoseSubsystem instance;
    private final Swerve s_Swerve;
    private final VisionSubsystem s_Vision;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;
    private final Pigeon2 gyro;
    private static Rotation2d targetAngle = null;

    public PoseSubsystem(Swerve s_Swerve, VisionSubsystem s_Vision) {
        assert(instance == null);
        instance = this;
        
        this.s_Swerve = s_Swerve;
        this.s_Vision = s_Vision;

        gyro = new Pigeon2(Pose.pigeonID, Constants.Swerve.swerveCanBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);        

        Pose.rotationPID.enableContinuousInput(-180.0, 180.0);
        Pose.rotationPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
        Pose.rotationPID.reset();

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());

        field = new Field2d();
        SmartDashboard.putData("pose/Field", field);

        SmartDashboard.putBoolean("pose/Update from vision in Teleop", true);
        SmartDashboard.putBoolean("pose/Update from vision in Auto", false);

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            s_Swerve::getSpeeds, 
            s_Swerve::driveRobotRelativeAuto,
            // TODO Configure PIDs
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(8.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.5, 0.0, 0.0), // Rotation PID constants
                Constants.Swerve.maxSpeed, // Max module speed, in m/s
                Constants.Swerve.driveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            Robot::isRed,
            s_Swerve // Reference to Swerve subsystem to set requirements
        );

    }

    public static PoseSubsystem getInstance() {
        return instance;
    }
    
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void zeroGyro() {
        gyro.setYaw(0);
        DogLog.log("Swerve/Gyro/Status", "Zeroed Gyro Yaw");
    }

    public void hack() {
        gyro.setYaw(gyro.getYaw().getValue() + 180.0);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), pose);
        DogLog.log("Swerve/Status/Setting Pose", pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        setHeading(new Rotation2d());
        DogLog.log("Swerve/Gyro/Status", "Zeroed Gyro Heading");
    }

    public void resetHeading() {
        if (Robot.isRed()) {
            setHeading(new Rotation2d(Math.PI));
        } else {
            setHeading(new Rotation2d());
        }
    }

    public Translation2d speakerLocation() {
        return (Robot.isRed() ? Pose.redSpeakerLocation : Pose.blueSpeakerLocation);
    }

    public double distanceToSpeaker() {
        double distance = getPose().getTranslation().getDistance(PoseSubsystem.getInstance().speakerLocation()); // distance from center of robot to speaker 
        distance -= Constants.Vision.centerToReferenceOffset; // distance from center of robot to reference point
        //distance *= Constants.Vision.distanceFudgeFactor;
        return distance;
    }

    public Rotation2d dumpShotError() {
        Rotation2d robotAngle = getPose().getRotation();
        if (Robot.isRed()){
            return Pose.redDumpAngle.minus(robotAngle);
        } else {
            return Pose.blueDumpAngle.minus(robotAngle);
        }
    }

    public boolean dumpShotAligned() {
        if (Math.abs(dumpShotError().getDegrees()) < Pose.maxDumpError) {
            if (Math.abs(Pose.rotationPID.getVelocityError()) < Constants.Shooter.dumpShotVelocityErrorMax) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    public Rotation2d slideShotError() {
        Rotation2d robotAngle = getPose().getRotation();
        if (Robot.isRed()){
            return Pose.redSlideAngle.minus(robotAngle);
        } else {
            return Pose.blueSlideAngle.minus(robotAngle);
        }
    }

    public boolean slideShotAligned() {
        if (Math.abs(slideShotError().getDegrees()) < Pose.maxSlideError) {
            if (Math.abs(Pose.rotationPID.getVelocityError()) < Constants.Shooter.slideShotVelocityErrorMax) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    public static void angleErrorReset() {
        Pose.rotationPID.reset();
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

    public static void setTargetAngle(Rotation2d angle) {
        targetAngle = angle;
    }
    
    public static Rotation2d getTargetAngle() {
        return targetAngle;
    }
    
    public static double angleErrorToSpeed(Rotation2d angleError) {
        double angleErrorDeg = angleError.getDegrees();
        double correction = Pose.rotationPID.calculate(angleErrorDeg);
        double feedForward = Pose.rotationKS * Math.signum(angleErrorDeg);
        double output = MathUtil.clamp(correction + feedForward, -1.0, 1.0);

        DogLog.log("Pose/Angle Error", angleErrorDeg);
        DogLog.log("Pose/Angle PID correction", correction);
        DogLog.log("Pose/Angle feedforward", feedForward);
        DogLog.log("Pose/Angle output", output);
        
        return output;
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroYaw(), s_Swerve.getModulePositions());
        if ((DriverStation.isTeleop() && SmartDashboard.getBoolean("pose/Update from vision in Teleop", true))
            || (DriverStation.isAutonomous() && SmartDashboard.getBoolean("pose/Update from vision in Auto", false))) {
            s_Vision.updatePoseEstimate(poseEstimator);
        } else {
            s_Vision.updatePoseEstimate(null);
        }
        field.setRobotPose(getPose());

        SmartDashboard.putNumber("pose/Gyro", getHeading().getDegrees());
        SmartDashboard.putString("pose/Pose", getPose().toString());

        DogLog.log("Pose/Pose", getPose());
        DogLog.log("Pose/Gyro/Heading", getHeading().getDegrees());
        DogLog.log("Pose/Gyro/Raw Yaw", getGyroYaw());
    }
}