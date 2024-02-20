// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static TunableNumber topNumber = new TunableNumber("top motor/top");
  private static TunableNumber bottomNumber = new TunableNumber("bottom motor/top");
  private static TalonFX top;
  private static TalonFX bottom;
  private static final Timer timer = new Timer();

  private class ShooterSpeed {
    double topMotorSpeed;
    double bottomMotorSpeed;

    public ShooterSpeed(double top, double bottom) {
      topMotorSpeed = top;
      bottomMotorSpeed = bottom;
    }
  }

  public enum Speed {
    STOP,
    IDLE,
    AMP,
    SUBWOOFER,
    MIDLINE,
    PODIUM,
  };

  private Speed targetSpeed = Speed.IDLE;

  private final EnumMap<Speed, ShooterSpeed> shooterSpeeds = new EnumMap<>(Map.ofEntries(
      Map.entry(Speed.STOP, new ShooterSpeed(Constants.Shooter.stopSpeed, Constants.Shooter.stopSpeed)),
      Map.entry(Speed.IDLE, new ShooterSpeed(Constants.Shooter.idleSpeed, Constants.Shooter.idleSpeed)),
      Map.entry(Speed.AMP, new ShooterSpeed(0.20, 0.30)),
      Map.entry(Speed.SUBWOOFER, new ShooterSpeed(0.30, 0.60)),
      Map.entry(Speed.MIDLINE, new ShooterSpeed(0.50, 0.40)),
      Map.entry(Speed.PODIUM, new ShooterSpeed(0.59, 0.29)) // TODO: Podium Shooter Speeds
  ));

  public ShooterSubsystem() {
    top = new TalonFX(Constants.Shooter.topShooterID, Constants.Shooter.shooterMotorCanBus);
    bottom = new TalonFX(Constants.Shooter.bottomShooterID, Constants.Shooter.shooterMotorCanBus);
    topNumber.setDefault(0);
    bottomNumber.setDefault(0);
  }

  public void applyConfigs() {
    /* Configure the Shooter Motors */
    var m_ShooterMotorsConfiguration = new TalonFXConfiguration();
    /* Set Shooter motors to Brake */
    m_ShooterMotorsConfiguration.MotorOutput.NeutralMode = Constants.Shooter.motorNeutralValue;
    /* Set the Shooters motor direction */
    m_ShooterMotorsConfiguration.MotorOutput.Inverted = Constants.Shooter.motorOutputInverted; // TODO: test this Monday
    // TODO Why is inversion not working?
    /* Config the peak outputs */
    m_ShooterMotorsConfiguration.Voltage.PeakForwardVoltage = Constants.Shooter.peakForwardVoltage;
    m_ShooterMotorsConfiguration.Voltage.PeakReverseVoltage = Constants.Shooter.peakReverseVoltage;
    /* Apply Shooters Motor Configs */
    top.getConfigurator().apply(m_ShooterMotorsConfiguration);
    bottom.getConfigurator().apply(m_ShooterMotorsConfiguration);
  }

  public void setTargetSpeed(Speed speed) {
    targetSpeed = speed;
  }

  public void shoot() {
    setCurrentSpeed(targetSpeed);
    timer.restart();
  }

  public void setCurrentSpeed(Speed speed) {
    setCurrentSpeed(shooterSpeeds.get(speed));
  }

  public void setCurrentSpeed(ShooterSpeed speed) {
    top.set(speed.topMotorSpeed);
    bottom.set(speed.bottomMotorSpeed);
  }

  public void idle() {
    setCurrentSpeed(Speed.IDLE);
  }

  public void stop() {
    setCurrentSpeed(Speed.STOP);
  }

  public boolean isReady() {
    // TODO Actually check current speeds instead
    return timer.hasElapsed(Constants.Index.waitToShootTime);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("topVal", topNumber.get());
    SmartDashboard.putNumber("bottomVal", bottomNumber.get());
  }
}