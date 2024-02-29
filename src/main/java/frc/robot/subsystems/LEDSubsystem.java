// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private CANdle m_candle = new CANdle(0, "rio");
  private static BaseState baseState = null;
  private static TempState tempState = null;
  private static BaseState lastBaseState = null;
  private static TempState lastTempState = null;

  public static class Color {
    private final int R, G, B;
    public Color(int r,  int g, int b) {
      R = r;
      G = g;
      B = b;
    }
  }

  public enum BaseState {
    DISABLED,
    READY,
    EMPTY,
    NOTE,
    SHOOTABLE
  }

  public enum TempState {
    INTAKING,
    SHOOTING,
    ERROR
  }

  public static final class Colors {
    public static final Color off = new LEDSubsystem.Color(0, 0, 0);
    public static final Color red = new LEDSubsystem.Color(255, 0, 0);
    public static final Color green = new LEDSubsystem.Color(0, 255, 0);
    public static final Color blue = new LEDSubsystem.Color(0, 0, 255);
    public static final Color cyan = new LEDSubsystem.Color(0, 255, 255);
    public static final Color magenta = new LEDSubsystem.Color(255, 0, 255);
    public static final Color yellow = new LEDSubsystem.Color(255, 255, 0);
    public static final Color white = new LEDSubsystem.Color(255, 255, 255);
    public static final Color lynk = new LEDSubsystem.Color(255, 64, 0);
    public static final Color disabled = new LEDSubsystem.Color(200, 0, 0);
  }

  public LEDSubsystem() {
    m_candle.configBrightnessScalar(0.50);
    m_candle.configLEDType(LEDStripType.GRB);
    m_candle.configV5Enabled(true);
    m_candle.configLOSBehavior(false); // TODO: true -- why is this triggering?
    setBaseState(BaseState.READY);
  }

  public static void setBaseState(BaseState newState) {
    baseState = newState;
  }
  
  public static void setTempState(TempState newState) {
    tempState = newState;
  }
  
  public static void clearTempState() {
    tempState = null;
  }
  
  public void setColor(Color color) {
    System.out.printf("Setting color (%d, %d, %d)%n", color.R, color.G, color.B);
    m_candle.setLEDs(color.R, color.G, color.B);
  }

  public void setRainbow(){
    m_candle.clearAnimation(0);
    m_candle.animate(new RainbowAnimation(0.50, 0.5, 68, false, 8));
  }

  /*public void detectIntake(){
    if (m_Index.getIndexSensor().getAsBoolean()) {
      setRainbow();
    } else {
      m_candle.setLEDs(255, 0, 0, 255, 0, 31);
    }
  }*/

  @Override
  public void periodic() {
    SmartDashboard.putString("LED/Base state", baseState == null ? "NULL" : baseState.toString());
    SmartDashboard.putString("LED/Temp state", tempState == null ? "NULL" : tempState.toString());

    if (tempState != null && tempState == lastTempState) {
      // Temporary state is still active
      // TODO Support blinking
      System.out.println("LED::periodic: Temp State unchanged");
      return;
    }
    if (tempState != null) {
      System.out.println("LED::periodic: Temp State NEW");
      // TODO Set color based on temporary state
      if (tempState == TempState.ERROR) {
        setColor(Colors.red); // TODO Blink
      } else if (tempState == TempState.INTAKING) {
        setColor(Colors.yellow); // TODO Blink
      } else if (tempState == TempState.SHOOTING) {
        setColor(Colors.green); // TODO Blink
      }
      lastTempState = tempState;
      return;
    }
    if (baseState == null) {
      // Temporary startup condition
      System.out.println("LED::periodic: Base State NULL");
      return;
    }
    if (baseState != lastBaseState || lastTempState != null) {
      System.out.println("LED::periodic: Base State new (or dropped temp state)");
      if (baseState == BaseState.DISABLED) {
        setColor(Colors.disabled);
      } else if (baseState == BaseState.READY) {
        setColor(Colors.lynk);
      } else if (baseState == BaseState.EMPTY) {
        setColor(Colors.white);
      } else if (baseState == BaseState.NOTE) {
        setColor(Colors.yellow);
      } else if (baseState == BaseState.SHOOTABLE) {
        setColor(Colors.green);
      }
      lastBaseState = baseState;
    }
    lastTempState = tempState;
  }
}