// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.subsystems.PoseSubsystem.Target;
import static frc.robot.Options.*;

public class ShooterSubsystem extends SubsystemBase {
  private static TalonFX top;
  private static TalonFX bottom;
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage topControl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage bottomControl = new VelocityVoltage(0).withEnableFOC(true);
  private double topCurrentTarget = 0.0;
  private double bottomCurrentTarget = 0.0;
  SendableChooser<ShotType> defaultShotChooser = new SendableChooser<>();
  private static final TunableOption optAlwaysReady = new TunableOption("shooter/Always ready", false);

  private class ShooterSpeed {
    double topMotorSpeed;
    double bottomMotorSpeed;

    public ShooterSpeed(double top, double bottom) {
      topMotorSpeed = top;
      bottomMotorSpeed = bottom;
    }
  }

  private class ShooterCalibration {
    double distance;
    ShooterSpeed speed;

    public ShooterCalibration(double distance, ShooterSpeed speed) {
      this.distance = distance;
      this.speed = speed;
    }
  }

  public enum ShotType {
    STOP,
    INTAKE,
    IDLE,
    AMP,
    SUBWOOFER,
    AMPSIDE,
    MIDLINE,
    PODIUM,
    FULL,
    SOURCESIDEOTF,
    AMPSIDEOTF,
    OTF,
    SHORTSLIDE,
    SLIDE,
    DUMP,
    EJECT,
    BLOOP,
    AUTOMATIC,
    SHUTTLE,
    FARSHUTTLE,
  };

  private ShotType nextShot = null;

  private final EnumMap<ShotType, ShooterSpeed> shooterSpeeds = new EnumMap<>(Map.ofEntries(
      Map.entry(ShotType.STOP, new ShooterSpeed(Constants.Shooter.stopSpeed, Constants.Shooter.stopSpeed)),
      Map.entry(ShotType.INTAKE, new ShooterSpeed(Constants.Shooter.intakeSpeed, Constants.Shooter.intakeSpeed)),
      Map.entry(ShotType.IDLE, new ShooterSpeed(Constants.Shooter.idleSpeed, Constants.Shooter.idleSpeed)),
      Map.entry(ShotType.AMP, new ShooterSpeed(350, 950)),
      Map.entry(ShotType.SUBWOOFER, new ShooterSpeed(1360, 2830)),
      Map.entry(ShotType.AMPSIDE, new ShooterSpeed(2850, 2050)),
      Map.entry(ShotType.MIDLINE, new ShooterSpeed(2800, 2300)),
      Map.entry(ShotType.PODIUM, new ShooterSpeed(3000, 1600)),
      Map.entry(ShotType.FULL, new ShooterSpeed(Constants.Shooter.topSpeed, Constants.Shooter.topSpeed)),
      Map.entry(ShotType.OTF, new ShooterSpeed(3000, 1600)),
      Map.entry(ShotType.SOURCESIDEOTF, new ShooterSpeed(2950, 1925)),
      Map.entry(ShotType.AMPSIDEOTF, new ShooterSpeed(2950, 1800)),
      Map.entry(ShotType.SLIDE, new ShooterSpeed(2500, 1000)),
      Map.entry(ShotType.SHORTSLIDE, new ShooterSpeed(2250, 900)),
      Map.entry(ShotType.DUMP, new ShooterSpeed(2650, 2650)),
      Map.entry(ShotType.EJECT, new ShooterSpeed(-800, -800)),
      Map.entry(ShotType.BLOOP, new ShooterSpeed(400, 400))
      // Auto types: AUTOSPEAKER, SHUTTLE, FARSHUTTLE
  ));

  private final ShooterCalibration[] shooterCalibration = {
    new ShooterCalibration(35.9, new ShooterSpeed(1200, 3200)),
    new ShooterCalibration(46.9, new ShooterSpeed(1500, 2500)),
    new ShooterCalibration(59.5, new ShooterSpeed(2200, 2200)),
    new ShooterCalibration(72.2, new ShooterSpeed(2700, 2200)),
    new ShooterCalibration(83.5, new ShooterSpeed(3000, 1900)),
    new ShooterCalibration(95.7, new ShooterSpeed(2900, 1700)),
    new ShooterCalibration(108.1, new ShooterSpeed(2800, 1550)),
    new ShooterCalibration(120.9, new ShooterSpeed(2800, 1425)),
    new ShooterCalibration(132.0, new ShooterSpeed(2700, 1400))
  };

  private final ShooterCalibration[] shuttleCalibration = {
    new ShooterCalibration(210, new ShooterSpeed(1450, 1450)),
    new ShooterCalibration(292, new ShooterSpeed(2100, 2200)),
    new ShooterCalibration(384, new ShooterSpeed(2700, 3000)),
    new ShooterCalibration(449, new ShooterSpeed(2900, 3200)), // Blair Robot Project FTW!
    new ShooterCalibration(506, new ShooterSpeed(3200, 3506)), // Thanks, YETI!
  };

  public ShooterSubsystem() {
    top = new TalonFX(Constants.Shooter.topShooterID, Constants.Shooter.shooterMotorCanBus);
    bottom = new TalonFX(Constants.Shooter.bottomShooterID, Constants.Shooter.shooterMotorCanBus);
    applyConfigs();

    SmartDashboard.putNumber("shooter/Top RPM adjustment", 0.0);
    SmartDashboard.putNumber("shooter/Bottom RPM adjustment", 0.0);

    defaultShotChooser.setDefaultOption("== AUTO SPEAKER ==", ShotType.AUTOMATIC);
    for (ShotType shot : ShotType.values()) {
      if (shot != ShotType.AUTOMATIC) {
        defaultShotChooser.addOption(shot.toString(), shot);
      }
    }
    SmartDashboard.putData("shooter/Default shot", defaultShotChooser);
  }

  private void applyConfigs() {
    /* Configure the Shooter Motors */
    var m_ShooterMotorsConfiguration = new TalonFXConfiguration();
    /* Set Shooter motors to Brake */
    m_ShooterMotorsConfiguration.MotorOutput.NeutralMode = Constants.Shooter.motorNeutralValue;
    /* Set the Shooters motor direction */
    m_ShooterMotorsConfiguration.MotorOutput.Inverted = Constants.Shooter.motorOutputInverted;
    /* Config the peak outputs */
    m_ShooterMotorsConfiguration.Voltage.PeakForwardVoltage = Constants.Shooter.peakForwardVoltage;
    m_ShooterMotorsConfiguration.Voltage.PeakReverseVoltage = Constants.Shooter.peakReverseVoltage;

    // PID & FF configuration
    m_ShooterMotorsConfiguration.Slot0.kP = Constants.Shooter.kP;
    m_ShooterMotorsConfiguration.Slot0.kI = Constants.Shooter.kI;
    m_ShooterMotorsConfiguration.Slot0.kD = Constants.Shooter.kD;
    m_ShooterMotorsConfiguration.Slot0.kS = Constants.Shooter.kS;
    m_ShooterMotorsConfiguration.Slot0.kV = 1.0 / toRPS(Constants.Shooter.RPMsPerVolt);
    m_ShooterMotorsConfiguration.Slot0.kA = 0.0;
    m_ShooterMotorsConfiguration.Slot0.kG = 0.0;

    /* Apply Shooters Motor Configs */
    top.getConfigurator().apply(m_ShooterMotorsConfiguration);
    bottom.getConfigurator().apply(m_ShooterMotorsConfiguration);
  }

  private double toRPM(double rps) {
    return rps * 60.0;
  }

  private double toRPS(double rpm) {
    return rpm / 60.0;
  }

  public void setNextShot(ShotType speed) {
    nextShot = speed;
  }

  private ShooterSpeed speedFromDistance(double meters, ShooterCalibration[] calibrationTable) {
    double distance = Units.metersToInches(meters);
    ShooterCalibration priorEntry = null;
    ShooterSpeed speed = null;

    for (ShooterCalibration calibration : calibrationTable) {
      if (distance <= calibration.distance) {
        if (priorEntry == null) {
          // Anything closer that minimum calibration distance gets the same speed as minimum distance
          speed = calibration.speed;
        } else {
          // Linear interpolation between calibration entries
          double fraction = (distance - priorEntry.distance) / (calibration.distance - priorEntry.distance);

          speed = new ShooterSpeed(
            fraction * calibration.speed.topMotorSpeed + (1 - fraction) * priorEntry.speed.topMotorSpeed,
            fraction * calibration.speed.bottomMotorSpeed + (1 - fraction) * priorEntry.speed.bottomMotorSpeed
          );
        }

        break;
      }
      priorEntry = calibration;
    }

    // NOTE: Might be null if the calibration distance has been exceeded
    return speed;
  }

  private ShotType defaultShot() {
    return defaultShotChooser.getSelected();
  }

  public boolean setCurrentSpeed(ShotType shot) {
    return setCurrentSpeed(shot, null);
  }

  public boolean setCurrentSpeed(ShotType shot, Target target) {
    ShooterSpeed shooterSpeed;
    double distance;

    // TODO Is this safeguard needed?
    if (shot == null) {
      DogLog.logFault("ShooterSubsystem::setCurrentSpeed: shot is null");
      shot = defaultShot();
    }

    if (shooterSpeeds.containsKey(shot)) {
      shooterSpeed = shooterSpeeds.get(shot);
    } else {
      if (target == null) {
        DogLog.logFault("Shot type " + shot + " without target; presuming speaker");
        target = Target.SPEAKER;
      }
      if (shot == ShotType.AUTOMATIC && optShootWithVision.get()) {
        distance = VisionSubsystem.getInstance().distanceToTarget(target);
      } else {
        distance = PoseSubsystem.getInstance().getDistance(target);
      }
      shooterSpeed = speedFromDistance(distance, target == Target.SPEAKER ? shooterCalibration : shuttleCalibration);
      if (shooterSpeed == null) {
        DogLog.logFault(String.format("ShooterSubsystem::setCurrentSpeed: distance lookup failure for %s shot and target %s at %01.1f inches", shot.toString(), target.toString(), Units.metersToInches(distance)));
        DogLog.log("Shooter/Status", String.format("ShooterSubsystem::setCurrentSpeed: distance lookup failure for %s shot  and target %s at %01.1f inches", shot.toString(), target.toString(), Units.metersToInches(distance)));
        return false;
      }
    }

    setCurrentSpeed(shooterSpeed);
    return true;
  }

  public void setCurrentSpeed(double topRPM, double bottomRPM) {
    setCurrentSpeed(new ShooterSpeed(topRPM, bottomRPM));
  }

  private void setCurrentSpeed(ShooterSpeed speed) {
    topCurrentTarget = speed.topMotorSpeed + SmartDashboard.getNumber("shooter/Top RPM adjustment", 0.0);
    bottomCurrentTarget = speed.bottomMotorSpeed + SmartDashboard.getNumber("shooter/Bottom RPM adjustment", 0.0);
    top.setControl(topControl.withVelocity(toRPS(topCurrentTarget)));
    bottom.setControl(bottomControl.withVelocity(toRPS(bottomCurrentTarget)));
  }

  public void setVoltage(double voltage) {
    top.setControl(voltageOut.withOutput(voltage));
    bottom.setControl(voltageOut.withOutput(voltage));
  }

  public void setRPM(double rpm) {
    setCurrentSpeed(new ShooterSpeed(rpm, rpm));
  }

  public void idle() {
    if (optAlwaysReady.get()) {
      setCurrentSpeed(nextShot(), currentTarget());
    } else {
      setCurrentSpeed(ShotType.IDLE);
    }
  }

  public void intake() {
    setCurrentSpeed(ShotType.INTAKE);
  }

  public void eject() {
    setCurrentSpeed(ShotType.EJECT);
  }

  public void stop() {
    setCurrentSpeed(ShotType.STOP);
  }

  private double getMaxRPMError(Target target, boolean precise) {
    double rpm = Constants.Shooter.maxVelocityError.get(target);
    if (precise) {
      rpm *= Constants.Shooter.longAccuracyFactor;
    }
    return rpm;
  }

  public boolean isReady(Target target, boolean precise) {
    double maxError = getMaxRPMError(target, precise);
    return (Math.abs(toRPM(top.getVelocity().getValueAsDouble()) - topCurrentTarget) < maxError &&
      Math.abs(toRPM(bottom.getVelocity().getValueAsDouble()) - bottomCurrentTarget) < maxError);
  }

  public ShotType nextShot() {
    return nextShot == null ? defaultShot() : nextShot;
  }

  public static boolean requiresAlignment(ShotType shot) {
    return (shot == ShotType.DUMP || shot == ShotType.SLIDE || shot == ShotType.AMP || shot == ShotType.SHUTTLE || shot == ShotType.FARSHUTTLE);
  }

  public static boolean requiresVision(ShotType shot, Target target) {
    return shot == ShotType.AUTOMATIC && target == Target.SPEAKER && optShootWithVision.get(); 
  }

  public boolean requiresVision() {
    return requiresVision(nextShot(), currentTarget());
  }

  public Target shotToTarget(ShotType shot) {
    if (shot == ShotType.DUMP) {
      return Target.FIXED_DUMP;
    } else if (shot == ShotType.SLIDE) {
      return Target.FIXED_SLIDE;
    } else if (shot == ShotType.AMP) {
      return Target.FIXED_AMP;
    } else if (shot == ShotType.SHUTTLE) {
      return Target.SHUTTLE;
    } else if (shot == ShotType.FARSHUTTLE) {
      return Target.FAR_SHUTTLE;
    } else {
      return Target.SPEAKER;
    }
  }

  public Target currentTarget() {
    return shotToTarget(nextShot());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double topVel = toRPM(top.getVelocity().getValueAsDouble());
    double bottomVel = toRPM(bottom.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("shooter/Top RPM", topVel);
    SmartDashboard.putNumber("shooter/Bottom RPM", bottomVel);
    SmartDashboard.putNumber("shooter/Top RPM tgt", topCurrentTarget);
    SmartDashboard.putNumber("shooter/Bottom RPM tgt", bottomCurrentTarget);
    SmartDashboard.putNumber("shooter/Top RPM err", topVel - topCurrentTarget);
    SmartDashboard.putNumber("shooter/Bottom RPM err", bottomVel - bottomCurrentTarget);
    SmartDashboard.putString("shooter/Next shot", nextShot().toString());

    DogLog.log("Shooter/Top RPM", topVel);
    DogLog.log("Shooter/Bottom RPM", bottomVel);
    DogLog.log("Shooter/Top RPM tgt", topCurrentTarget);
    DogLog.log("Shooter/Bottom RPM tgt", bottomCurrentTarget);
    DogLog.log("Shooter/Top RPM err", topVel - topCurrentTarget);
    DogLog.log("Shooter/Bottom RPM err", bottomVel - bottomCurrentTarget);
  }
}