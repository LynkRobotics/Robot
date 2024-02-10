// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {
  /** Creates a new IndexSubsystem. */
  public static TalonFX indexMotor;

  public IndexSubsystem() {
    indexMotor = new TalonFX(Constants.Index.indexMotorID, Constants.Index.indexMotorCanBus);

  }
  
  public void index() {
    indexMotor.set(-1.00);
  }

  public void stop(){
    indexMotor.set(0);
  }

  public void eject(){
    indexMotor.set(1);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
