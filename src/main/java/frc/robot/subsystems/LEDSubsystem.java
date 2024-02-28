// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private IndexSubsystem m_Index;
  private CANdle m_candle = new CANdle(0, "rio");

  public static class Color {
    private final int R, G, B;
    public Color(int r, int g, int b) {
      R = r;
      G = g;
      B = b;
    }
  }

  public enum BaseState {
    DISABLED,
    EMPTY,
    NOTE
  }

  public enum ActiveState {
    CUBE,
    CONE
  }

  public static final class Colors {
    public static final Color off = new LED.Color(0, 0, 0);
    public static final Color red = new LED.Color(255, 0, 0);
    public static final Color green = new LED.Color(0, 255, 0);
    public static final Color blue = new LED.Color(0, 0, 255);
    public static final Color cyan = new LED.Color(0, 255, 255);
    public static final Color magenta = new LED.Color(255, 0, 255);
    public static final Color yellow = new LED.Color(255, 255, 0);
    public static final Color white = new LED.Color(255, 255, 255);
    public static final Color lynk = new LED.Color(162, 255, 0);
    public static final Color disabled = new LED.Color(200, 0, 0);
    public static final Color cube = new LED.Color(186, 0, 255);
    public static final Color cone = new LED.Color(255, 128, 0);
  }

  public LEDSubsystem() {
    m_Index = new IndexSubsystem();
    m_candle.configBrightnessScalar(0.50);
    m_candle.configLEDType(LEDStripType.GRB);
    m_candle.configV5Enabled(true);
    m_candle.configLOSBehavior(true);
    setColor(Colors.lynk);
  }
  
  public void setColor(Color color) {
    m_candle.setLEDs(color.R, color.G, color.B);
  }

  public void setRainbow(){
    m_candle.clearAnimation(0);
    m_candle.animate(new RainbowAnimation(0.50, 0.5, 68, false, 8));
  }

  public void detectIntake(){
    if (m_Index.getIndexSensor().getAsBoolean()) {
      setRainbow();
    } else {
      m_candle.setLEDs(255, 0, 0, 255, 0, 31);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_candle.setLEDs(255, 0, 0, 255, 0, 31);
  }
}
