// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private CANdle m_candle = new CANdle(0, "rio");
  public LEDSubsystem() {
    m_candle.configBrightnessScalar(0.50);
    m_candle.configLEDType(LEDStripType.GRB);
    m_candle.configV5Enabled(true);
    m_candle.configLOSBehavior(true);
    m_candle.setLEDs(191, 87, 0, 255, 0, 68);
  }

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_candle.setLEDs(191, 87, 0, 255, 0, 68);

  }
}
