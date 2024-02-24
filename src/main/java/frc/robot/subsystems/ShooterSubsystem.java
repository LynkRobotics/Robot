// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static TalonFX top;
  private static TalonFX bottom;
  //private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage topControl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage bottomControl = new VelocityVoltage(0).withEnableFOC(true);

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

  public enum Speed {
    STOP,
    IDLE,
    AMP,
    SUBWOOFER,
    MIDLINE,
    PODIUM,
    FULL
  };

  private Speed targetSpeed = Speed.IDLE;

  private final EnumMap<Speed, ShooterSpeed> shooterSpeeds = new EnumMap<>(Map.ofEntries(
      Map.entry(Speed.STOP, new ShooterSpeed(Constants.Shooter.stopSpeed, Constants.Shooter.stopSpeed)),
      Map.entry(Speed.IDLE, new ShooterSpeed(Constants.Shooter.idleSpeed, Constants.Shooter.idleSpeed)),
      Map.entry(Speed.AMP, new ShooterSpeed(1100, 1700)),
      Map.entry(Speed.SUBWOOFER, new ShooterSpeed(1400, 2500)), //TOP: 1700, 1500 BOTTOM: 3400(NO), 3700(NO), 3900(NO)
      Map.entry(Speed.MIDLINE, new ShooterSpeed(2800, 2300)),
      Map.entry(Speed.PODIUM, new ShooterSpeed(3000, 1600)), //3350, 1600 | 
      Map.entry(Speed.FULL, new ShooterSpeed(Constants.Shooter.topSpeed, Constants.Shooter.topSpeed))
  ));

  // TODO Calibrate shooter from various distances
  private final ShooterCalibration[] shooterCalibration = {
    new ShooterCalibration(3.0, new ShooterSpeed(3000, 3000)),
    new ShooterCalibration(4.0, new ShooterSpeed(3000, 3000)),
    new ShooterCalibration(5.0, new ShooterSpeed(3000, 3000)),
  };

  public ShooterSubsystem() {
    top = new TalonFX(Constants.Shooter.topShooterID, Constants.Shooter.shooterMotorCanBus);
    bottom = new TalonFX(Constants.Shooter.bottomShooterID, Constants.Shooter.shooterMotorCanBus);
    applyConfigs();
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

  public void setTargetSpeed(Speed speed) {
    targetSpeed = speed;
  }

  private ShooterSpeed speedFromDistance(double distance) {
    ShooterCalibration priorEntry = null;
    ShooterSpeed speed = null;

    for (ShooterCalibration calibration : shooterCalibration) {
      if (distance <= calibration.distance) {
        if (priorEntry == null) {
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

    if (speed == null) {
      // Distance is greater than the last calibration
      speed = priorEntry.speed;
    }

    return speed;
  }

  public void shoot() {
    setCurrentSpeed(targetSpeed);
  }

  private void setCurrentSpeed(Speed speed) {
    ShooterSpeed shooterSpeed;

    if (speed == null) {
      // TODO Get distance from vision subsystem
      shooterSpeed = speedFromDistance(4.0);
    } else {
      shooterSpeed = shooterSpeeds.get(speed);
    }
    setCurrentSpeed(shooterSpeed);
  }

  /*private void setVelocityTorque(TalonFX motor, double rpm) {
    motor.setControl(velocityTorqueCurrentFOC.withVelocity(toRPS(rpm)));
  }*/

  private void setCurrentSpeed(ShooterSpeed speed) {
    top.setControl(topControl.withVelocity(toRPS(speed.topMotorSpeed)));
    bottom.setControl(bottomControl.withVelocity(toRPS(speed.bottomMotorSpeed)));
  }

  public void setVoltage(double voltage) {
    top.setControl(voltageOut.withOutput(voltage));
    bottom.setControl(voltageOut.withOutput(voltage));
  }

  public void setRPM(double rpm) {
    setCurrentSpeed(new ShooterSpeed(rpm, rpm));
  }

  public void idle() {
    setCurrentSpeed(Speed.IDLE);
  }

  public void stop() {
    setCurrentSpeed(Speed.STOP);
  }

  public boolean isReady() {
    return (Math.abs(toRPM(top.getVelocity().getValueAsDouble()) - shooterSpeeds.get(targetSpeed).topMotorSpeed) < Constants.Shooter.maxError &&
      Math.abs(toRPM(bottom.getVelocity().getValueAsDouble()) - shooterSpeeds.get(targetSpeed).bottomMotorSpeed) < Constants.Shooter.maxError);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double topVel = toRPM(top.getVelocity().getValueAsDouble());
    double bottomVel = toRPM(bottom.getVelocity().getValueAsDouble());
    double topTarget = shooterSpeeds.get(targetSpeed).topMotorSpeed;
    double bottomTarget = shooterSpeeds.get(targetSpeed).bottomMotorSpeed;
    SmartDashboard.putNumber("Top RPM", topVel);
    SmartDashboard.putNumber("Bottom RPM", bottomVel);
    SmartDashboard.putNumber("Top RPM tgt", topTarget);
    SmartDashboard.putNumber("Bottom RPM tgt", bottomTarget);
    SmartDashboard.putNumber("Top RPM err", topVel - topTarget);
    SmartDashboard.putNumber("Bottom RPM err", bottomVel - bottomTarget);
    SmartDashboard.putBoolean("Shooter ready", isReady());
    SmartDashboard.putString("Next shot", targetSpeed == null ? "Vision" : targetSpeed.toString());
  }
}