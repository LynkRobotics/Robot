// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static TunableNumber topNumber = new TunableNumber("top motor/top");
  private static TunableNumber bottomNumber = new TunableNumber("bottom motor/top");
  private static TalonFX top;
  private static TalonFX bottom;
  //private static final Timer timer = new Timer();
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
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
      Map.entry(Speed.SUBWOOFER, new ShooterSpeed(1700, 3400)),
      Map.entry(Speed.MIDLINE, new ShooterSpeed(2800, 2300)),
      Map.entry(Speed.PODIUM, new ShooterSpeed(3350, 1600)), // TODO: Podium Shooter Speeds
      Map.entry(Speed.FULL, new ShooterSpeed(Constants.Shooter.topSpeed, Constants.Shooter.topSpeed))
  ));

  public ShooterSubsystem() {
    top = new TalonFX(Constants.Shooter.topShooterID, Constants.Shooter.shooterMotorCanBus);
    bottom = new TalonFX(Constants.Shooter.bottomShooterID, Constants.Shooter.shooterMotorCanBus);
    applyConfigs();
    topNumber.setDefault(0);
    bottomNumber.setDefault(0);
  }

  public void applyConfigs() {
    /* Configure the Shooter Motors */
    var m_ShooterMotorsConfiguration = new TalonFXConfiguration();
    /* Set Shooter motors to Brake */
    m_ShooterMotorsConfiguration.MotorOutput.NeutralMode = Constants.Shooter.motorNeutralValue;
    /* Set the Shooters motor direction */
    m_ShooterMotorsConfiguration.MotorOutput.Inverted = Constants.Shooter.motorOutputInverted;
    /* Config the peak outputs */
    m_ShooterMotorsConfiguration.Voltage.PeakForwardVoltage = Constants.Shooter.peakForwardVoltage;
    m_ShooterMotorsConfiguration.Voltage.PeakReverseVoltage = Constants.Shooter.peakReverseVoltage;

    m_ShooterMotorsConfiguration.Slot0.kP = 0.25; // Constants.Shooter.kP; // 12 // Voltage per 1 RPS of error
    m_ShooterMotorsConfiguration.Slot0.kI = 0;
    m_ShooterMotorsConfiguration.Slot0.kD = 0;
    m_ShooterMotorsConfiguration.Slot0.kV = 1.0 / (490 / 60.0); // Constants.Shooter.kV; // 0.2 // Anticipated voltage per 1 RPS
    m_ShooterMotorsConfiguration.Slot0.kA = 0;
    m_ShooterMotorsConfiguration.Slot0.kS = 0.21; // Constants.Shooter.kS; // 14 // Voltage to overcome static friction
    m_ShooterMotorsConfiguration.Slot0.kG = 0;

    /* Apply Shooters Motor Configs */
    top.getConfigurator().apply(m_ShooterMotorsConfiguration);
    bottom.getConfigurator().apply(m_ShooterMotorsConfiguration);
  }

  public void setTargetSpeed(Speed speed) {
    targetSpeed = speed;
  }

  public void shoot() {
    setCurrentSpeed(targetSpeed);
  }

  public void setCurrentSpeed(Speed speed) {
    setCurrentSpeed(shooterSpeeds.get(speed));
  }

  public void setVelocity(TalonFX motor, double rpm) {    
    motor.setControl(velocityControl.withVelocity(rpm / 60.0));
  }

  public void setVelocityTorque(TalonFX motor, double rpm) {
    motor.setControl(velocityTorqueCurrentFOC.withVelocity(rpm / 60.0));
  }

  public void setCurrentSpeed(ShooterSpeed speed) {
    //setVelocity(top, speed.topMotorSpeed);
    //setVelocity(bottom, speed.bottomMotorSpeed);
    top.setControl(topControl.withVelocity(speed.topMotorSpeed / 60.0));
    bottom.setControl(bottomControl.withVelocity(speed.bottomMotorSpeed / 60.0));
  }

  public void setMotorVoltage(TalonFX motor, double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  public void setVoltage(double voltage) {
    setMotorVoltage(top, voltage);
    setMotorVoltage(bottom, voltage);
  }

  public void setRPM(double rpm) {
    setVelocity(top, rpm);
    setVelocity(bottom, rpm);
  }

  public void idle() {
    setCurrentSpeed(Speed.IDLE);
  }

  public void stop() {
    setCurrentSpeed(Speed.STOP);
  }

  public boolean isReady() {
    return (Math.abs(top.getVelocity().getValueAsDouble() * 60 - shooterSpeeds.get(targetSpeed).topMotorSpeed) < Constants.Shooter.maxError &&
      Math.abs(bottom.getVelocity().getValueAsDouble() * 60 - shooterSpeeds.get(targetSpeed).bottomMotorSpeed) < Constants.Shooter.maxError);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double topVel = top.getVelocity().getValueAsDouble() * 60;
    double bottomVel = bottom.getVelocity().getValueAsDouble() * 60;
    double topTarget = shooterSpeeds.get(targetSpeed).topMotorSpeed;
    double bottomTarget = shooterSpeeds.get(targetSpeed).bottomMotorSpeed;
    SmartDashboard.putNumber("Top RPM", topVel);
    SmartDashboard.putNumber("Bottom RPM", bottomVel);
    SmartDashboard.putNumber("Top RPM tgt", topTarget);
    SmartDashboard.putNumber("Bottom RPM tgt", bottomTarget);
    SmartDashboard.putNumber("Top RPM err", topVel - topTarget);
    SmartDashboard.putNumber("Bottom RPM err", bottomVel - bottomTarget);
    SmartDashboard.putBoolean("Shooter ready", isReady());
  }
}