package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.swerveCanBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getSpeeds, 
                this::driveRobotRelativeAuto,
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
                this // Reference to this subsystem to set requirements
            );
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                );

        driveRobotRelative(desiredChassisSpeeds, isOpenLoop);
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelativeAuto(ChassisSpeeds desirChassisSpeeds) {
        driveRobotRelative(desirChassisSpeeds, false);
    }

    public void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds, boolean isOpenLoop) {
        ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02); 
        
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds); 
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void hack(){
        gyro.setYaw(gyro.getYaw().getValue() + 180.0);
    }

    public Rotation2d dumpShotError() {
        Rotation2d robotAngle = getPose().getRotation();
        if (DriverStation.getAlliance().get() == Alliance.Red){
            return Constants.Swerve.redDumpAngle.minus(robotAngle);
        } else {
            return Constants.Swerve.dumpAngle.minus(robotAngle);    
        }
    }

    public static double angleErrorToSpeed(Rotation2d angleError) {
        double angleErrorDeg = angleError.getDegrees();
        double magnitude = Math.abs(angleErrorDeg);
        double rotationVal = 0.0;

        if (magnitude > 10.0) {
            rotationVal = 0.10 * Math.signum(angleErrorDeg);
        } else if (magnitude > 7.0) {
            rotationVal = 0.06 * Math.signum(angleErrorDeg);
        } else if (magnitude > 3.0) {
            rotationVal = 0.04 * Math.signum(angleErrorDeg);
        } else if (magnitude > 0.3) {
            rotationVal = 0.03 * Math.signum(angleErrorDeg);
        }
        return rotationVal;
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Gyro", getHeading().getDegrees());
        // System.out.println("Swerve: Heading @ " + getHeading().getDegrees());
    }
}