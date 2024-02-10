// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public static TalonFX top;
  public static TalonFX bottom;

  public ShooterSubsystem() {
    top = new TalonFX(Constants.Shooter.topShooterID, Constants.Shooter.shooterMotorCanBus);
    bottom = new TalonFX(Constants.Shooter.bottomShooterID, Constants.Shooter.shooterMotorCanBus);
  }
  public void shoot(){
    top.set(-.15);
    bottom.set(-.35); //was 37
    
  }

  public void stop(){
    top.set(0);
    bottom.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
