// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ShooterConfiguration;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private TunableNumber topNumber = new TunableNumber("top motor/top");
  private TunableNumber bottomNumber = new TunableNumber("bottom motor/top");
  /** Creates a new ShooterSubsystem. */
  public static TalonFX top;
  public static TalonFX bottom;

  public ShooterSubsystem() {
    top = new TalonFX(Constants.Shooter.topShooterID, Constants.Shooter.shooterMotorCanBus);
    bottom = new TalonFX(Constants.Shooter.bottomShooterID, Constants.Shooter.shooterMotorCanBus);
    // topNumber.setDefault(0);
    // bottomNumber.setDefault(0);
  }
  public void shoot(){
    top.set(-0.20); 
    bottom.set(-0.32); 
  }

  public static ShooterConfiguration[] shootingTableNormal = {
    new ShooterConfiguration(0, 0.30, 0.60), //Subwoofer Shot
    new ShooterConfiguration(1, 0.50, 0.40), //Mid Line
    new ShooterConfiguration(2, 0.20, 0.30), //AMP
    new ShooterConfiguration(3, 0, 0),
    new ShooterConfiguration(4, 0, 0),
    new ShooterConfiguration(5, 0, 0),
    new ShooterConfiguration(6, 0, 0),
    new ShooterConfiguration(7, 0, 0),
    new ShooterConfiguration(8, 0, 0),
    new ShooterConfiguration(9, 0, 0)
  };

  public static ShooterConfiguration[] shootingTable45 = {
    new ShooterConfiguration(0, 0.20, 0.80), //Subwoofer Shot
    new ShooterConfiguration(1, 0.00, 0.00), //Mid Line
    new ShooterConfiguration(2, 0.20, 0.32), //AMP
    new ShooterConfiguration(3, 0.60, 0.35), //Long shot
    new ShooterConfiguration(4, 0, 0),
    new ShooterConfiguration(5, 0, 0),
    new ShooterConfiguration(6, 0, 0),
    new ShooterConfiguration(7, 0, 0),
    new ShooterConfiguration(8, 0, 0),
    new ShooterConfiguration(9, 0, 0)
  };

  public void idle(){
    top.set(-.15);
    bottom.set(-.15);
  }

  public void stop(){
    top.set(0);
    bottom.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("topVal", topNumber.get());
  }
}
