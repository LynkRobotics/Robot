package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.concurrent.locks.ReentrantLock;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private boolean speedLimit = false;

    public SwerveModule[] mSwerveMods;
    private final ReentrantLock swerveModLock = new ReentrantLock();

    public Swerve() {
        Timer.delay(5); //Delaying the initalization of the swerve module should prevent a race condition with the CANcoders initializing, and causing just general funkiness
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

        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        swerveModLock.unlock();
    }

    public void alignStraight() {
        SwerveModuleState aligned = new SwerveModuleState(0.0, new Rotation2d());

        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(aligned, false);
        }
        swerveModLock.unlock();
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];

        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        swerveModLock.unlock();

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        swerveModLock.unlock();

        return positions;
    }

    public void resetModulesToAbsolute() {
        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
        swerveModLock.unlock();
    }

    public void enableSpeedLimit() {
        speedLimit = true;
        DogLog.log("Swerve/Status", "Swerve Speed Limit Enabled");
    }

    public void disableSpeedLimit() {
        speedLimit = false;
        DogLog.log("Swerve/Status", "Swerve Speed Limit Disabled");
    }

    public double getSpeedLimitRot() {
        return speedLimit ? Constants.Swerve.speedLimitRot : 1.0;
    }

    public void setMotorsToCoast() {
        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods) {
            mod.setCoastMode();  
        }
        swerveModLock.unlock();
        DogLog.log("Swerve/Status", "Coasted Swerve Motors");
    }

    public void setDriveMotorsToCoast() {
        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods) {
            mod.setCoastMode();  
        }
        swerveModLock.unlock();
        DogLog.log("Swerve/Status", "Coasted Swerve Drive Motors");
    }

    public void setMotorsToBrake() {
        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods) {
            mod.setBrakeMode();  
        }
        swerveModLock.unlock();
        DogLog.log("Swerve/Status", "Braked Swerve Motors");
    }

    public void setDriveMotorsToBrake() {
        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods) {
            mod.setBrakeMode();  
        }
        swerveModLock.unlock();
        DogLog.log("Swerve/Status", "Braked Swerve Drive Motors");
    }

    public void stopSwerve() {
        drive(new Translation2d(0, 0), 0, false);
        DogLog.log("Swerve/Status", "Stopped Swerve");
    }

    @Override
    public void periodic() {

        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            DogLog.log("Swerve/Mod/" + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            DogLog.log("Swerve/Mod/" + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Mod/" + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
            DogLog.log("Swerve/Mod/" + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        swerveModLock.unlock();

        DogLog.log("Swerve/Module States", getModuleStates());        
    }
}