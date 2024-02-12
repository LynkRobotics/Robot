// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {
  private final TalonFX indexMotor;
  private final DutyCycleOut indexSpeedDutyCycleOut;

  private final DigitalInput indexSesnor;

  public IndexSubsystem() {
    indexMotor = new TalonFX(Constants.Index.indexMotorID, Constants.Index.indexMotorCanBus);
    indexSpeedDutyCycleOut = new DutyCycleOut(0);
    indexSesnor = new DigitalInput(Constants.Index.indexSensorID);
    applyConfigs();
  }

  public BooleanSupplier getIndexSensor(){
    return () -> !indexSesnor.get();
  }

  public void applyConfigs(){
    /* Configure the Index Motor */
    var m_indexConfiguration = new TalonFXConfiguration();
    /* Set Index motor to Brake */
    m_indexConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    /* Set the motor direction */
    m_indexConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: test this Monday
    /* Config the peak outputs */
    m_indexConfiguration.Voltage.PeakForwardVoltage = Constants.Index.peakForwardVoltage;
    m_indexConfiguration.Voltage.PeakReverseVoltage = Constants.Index.peakReverseVoltage;
    /* Apply Index Motor Configs */
    indexMotor.getConfigurator().apply(m_indexConfiguration);
  }
  
  public void index() {
    indexMotor.setControl(indexSpeedDutyCycleOut.withOutput(Constants.Index.indexSpeed));
  }

  public void feed(){
    Timer.delay(Constants.Index.waitToShootTime);
    indexMotor.setControl(indexSpeedDutyCycleOut.withOutput(Constants.Index.feedSpeed));
  }

  public void stop(){
    indexMotor.setControl(indexSpeedDutyCycleOut.withOutput(Constants.Index.stopSpeed));
  }

  public void eject(){
    indexMotor.setControl(indexSpeedDutyCycleOut.withOutput(Constants.Index.ejectSpeed));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("a", indexSesnor.get());
  }
}
