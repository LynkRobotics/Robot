package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private boolean speedLimit = false;

    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private final Field2d field = new Field2d();

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.swerveCanBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        SmartDashboard.putData("swerve/Field", field);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    PoseSubsystem.getInstance().getHeading()
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

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void hack() {
        gyro.setYaw(gyro.getYaw().getValue() + 180.0);
    }

    public void enableSpeedLimit() {
        speedLimit = true;
    }

    public void disableSpeedLimit() {
        speedLimit = false;
    }

    public double getSpeedLimitRot() {
        return speedLimit ? Constants.Swerve.speedLimitRot : 1.0;
    }

    @Override
    public void periodic(){
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}