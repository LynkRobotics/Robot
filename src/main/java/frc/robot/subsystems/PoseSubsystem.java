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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableOption;
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
    private static Zone zone = Zone.SPEAKER;

    private static final TunableOption optUpdatePoseWithVisionAuto = new TunableOption("pose/Update with vision in Auto", false);

    public enum Zone {
        SPEAKER,
        MIDDLE,
        FAR
    }

    public enum Target {
        SPEAKER,
        SHUTTLE,
        FAR_SHUTTLE,
        FIXED_AMP,
        FIXED_SLIDE,
        FIXED_DUMP
    }

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

        Pose.maintainPID.enableContinuousInput(-180.0, 180.0);
        Pose.maintainPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
        Pose.maintainPID.reset();

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());

        field = new Field2d();
        SmartDashboard.putData("pose/Field", field);

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
    
    public static String prettyPose(Pose2d pose) {
        return String.format("(%01.2f, %01.2f @ %01.1f)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
    
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void zeroGyro() {
        gyro.setYaw(0);
        DogLog.log("Pose/Gyro/Status", "Zeroed Gyro Yaw");
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), pose);
        DogLog.log("Pose/Status/Setting Pose", pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        setHeading(new Rotation2d());
        DogLog.log("Pose/Gyro/Status", "Zeroed Gyro Heading");
    }

    public void resetHeading() {
        if (Robot.isRed()) {
            setHeading(new Rotation2d(Math.PI));
        } else {
            setHeading(new Rotation2d());
        }
    }

    public static Translation2d getLocation(Target target) {
        return (Robot.isRed() ? Pose.redLocations.get(target) : Pose.blueLocations.get(target));
    }

    public double getDistance(Target target) {
        double distance = getPose().getTranslation().getDistance(getLocation(target)); // distance from center of robot to target 
        distance -= Constants.Vision.centerToReferenceOffset; // distance from center of robot to reference point
        return distance;
    }

    private Translation2d targetOffset(Target target) {
        return getPose().getTranslation().minus(getLocation(target));
    }

    public Rotation2d angleToTarget(Target target) {
        if (target == Target.FIXED_AMP) {
            return Pose.ampAngle;
        } else if (target == Target.FIXED_DUMP) {
            return Robot.isRed() ? Pose.redDumpAngle : Pose.blueDumpAngle;
        } else if (target == Target.FIXED_SLIDE) {
            return Robot.isRed() ? Pose.redSlideAngle : Pose.blueSlideAngle;
        } else {
            return targetOffset(target).getAngle();
        }
    }

    public Rotation2d targetAngleError(Target target) {
        Rotation2d targetAngle = angleToTarget(target);
        Rotation2d robotAngle = getPose().getRotation();

        return targetAngle.minus(robotAngle);
    }

    public boolean targetAligned(Target target) {
        // TODO Consider putting this into the PID
        if (Math.abs(targetAngleError(target).getDegrees()) < Constants.Shooter.maxAngleError.get(target)) {
            if (Math.abs(Pose.rotationPID.getVelocityError()) < Constants.Shooter.maxVelocityError.get(target)) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    public static void angleErrorReset() {
        angleErrorReset(Pose.rotationPID);
    }

    public static void angleErrorReset(PIDController pid) {
        pid.reset();
    }

    public static void setTargetAngle(Rotation2d angle) {
        targetAngle = angle;
    }
    
    public static Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public static double angleErrorToSpeed(Rotation2d angleError) {
        return angleErrorToSpeed(angleError, Pose.rotationPID);
    }

    public static double angleErrorToSpeed(Rotation2d angleError, PIDController pid) {
        double angleErrorDeg = angleError.getDegrees();
        double correction = pid.calculate(angleErrorDeg);
        double feedForward = Pose.rotationKS * Math.signum(correction);
        double output = MathUtil.clamp(correction + feedForward, -1.0, 1.0);

        DogLog.log("Pose/Angle Error", angleErrorDeg);
        DogLog.log("Pose/Angle PID correction", correction);
        DogLog.log("Pose/Angle feedforward", feedForward);
        DogLog.log("Pose/Angle output", output);
        
        // Invert due to use as joystick controls
        return -output;
    }

    public static Zone getZone() {
        return zone;
    }

    public static Rotation2d reflect(Rotation2d angle, Rotation2d across) {
        return angle.times(2.0).minus(across);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroYaw(), s_Swerve.getModulePositions());
        if (!DriverStation.isAutonomousEnabled() || optUpdatePoseWithVisionAuto.get()) {
            s_Vision.updatePoseEstimate(poseEstimator);
        } else {
            s_Vision.updatePoseEstimate(null);
        }

        Pose2d pose = getPose();
        field.setRobotPose(pose);
        double distance = pose.getTranslation().getX();
        if (Robot.isRed()) {
            distance = Constants.Pose.fieldLength - distance;
        }
        if (distance > Constants.Pose.zoneSourceStart) {
            if (distance > Constants.Pose.zoneMiddleEnd || zone != Zone.MIDDLE) {
                zone = Zone.FAR;
            }
        } else if (distance < Constants.Pose.zoneSpeakerEnd) {
            if (distance < Constants.Pose.zoneMiddleStart || zone != Zone.MIDDLE) {
                zone = Zone.SPEAKER;
            }
        } else {
            zone = Zone.MIDDLE;
        }
        DogLog.log("Pose/Zone", zone);
        SmartDashboard.putString("pose/Zone", zone.toString());
        // SmartDashboard.putNumber("pose/Distance to shuttle", Units.metersToInches(getDistance(Target.SHUTTLE)));
        // SmartDashboard.putNumber("pose/Distance to far shuttle", Units.metersToInches(getDistance(Target.FAR_SHUTTLE)));

        SmartDashboard.putNumber("pose/Gyro", getHeading().getDegrees());
        SmartDashboard.putString("pose/Pose", pose.toString());

        DogLog.log("Pose/Pose", pose);
        DogLog.log("Pose/Gyro/Heading", getHeading().getDegrees());
        DogLog.log("Pose/Gyro/Raw Yaw", getGyroYaw());
    }
}