// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.EnumMap;
import java.util.Map;

import javax.swing.text.Position;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private TunableNumber topNumber = new TunableNumber("top motor/top");
  private TunableNumber bottomNumber = new TunableNumber("bottom motor/top");
  /** Creates a new ShooterSubsystem. */
  public static TalonFX top;
  public static TalonFX bottom;

  private class ShooterSpeed {
    double topMotorSpeed;
    double bottomMotorSpeed;

    public ShooterSpeed(double top, double bottom) {
      topMotorSpeed = top;
      bottomMotorSpeed = bottom;
    }
  }

  public enum Speed {
    IDLE,
    AMP,
    SUBWOOFER,
    MIDLINE,
    PODIUM,
  };

  private Deque<Speed> targetSpeed = new ArrayDeque<Speed>();

  private final EnumMap<Speed, ShooterSpeed> shooterSpeeds = new EnumMap<>(Map.ofEntries(
      Map.entry(Speed.IDLE, new ShooterSpeed(0.15, 0.15)),
      Map.entry(Speed.AMP, new ShooterSpeed(0.20, 0.30)),
      Map.entry(Speed.SUBWOOFER, new ShooterSpeed(0.30, 0.60)),
      Map.entry(Speed.MIDLINE, new ShooterSpeed(0.50, 0.40)),
      Map.entry(Speed.PODIUM, new ShooterSpeed(0, 0)) // TODO: Podium Shooter Speeds
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
    /* Config the peak outputs */
    m_ShooterMotorsConfiguration.Voltage.PeakForwardVoltage = Constants.Shooter.peakForwardVoltage;
    m_ShooterMotorsConfiguration.Voltage.PeakReverseVoltage = Constants.Shooter.peakReverseVoltage;
    /* Apply Shooters Motor Configs */
    top.getConfigurator().apply(m_ShooterMotorsConfiguration);
    bottom.getConfigurator().apply(m_ShooterMotorsConfiguration);
  }

  public void setTargetSpeeds(Speed speed) {
    targetSpeed.clear();
    targetSpeed.add(speed);
  }

  public void shoot() {
    Speed nextSpeedType = targetSpeed.peek();
    ShooterSpeed nextSpeed;
    nextSpeed = shooterSpeeds.get(nextSpeedType);

    top.set(nextSpeed.topMotorSpeed); // 59
    bottom.set(nextSpeed.bottomMotorSpeed); // 29
  }

  public void idle() {
    top.set(Constants.Shooter.idleSpeed);
    bottom.set(Constants.Shooter.idleSpeed);
  }

  public void stop() {
    top.set(Constants.Shooter.stopSpeed);
    bottom.set(Constants.Shooter.stopSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("topVal", topNumber.get());
    SmartDashboard.putNumber("bottomVal", bottomNumber.get());
  }
}
