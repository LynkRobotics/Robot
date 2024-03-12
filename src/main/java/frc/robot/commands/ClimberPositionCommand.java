// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.TempState;

public class ClimberPositionCommand extends Command {
  private final ClimberSubsystem s_Climber;
  private final double position;
  private final LEDSubsystem.TempState ledState;
  private boolean cancelled = false;
  
  /** Creates a new PushClimberCommand. */
  public ClimberPositionCommand(double position, LEDSubsystem.TempState ledState, ClimberSubsystem s_Climber) {
    this.s_Climber = s_Climber;
    addRequirements(s_Climber);

    this.position = position;
    this.ledState = ledState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getMatchTime() > Constants.Climber.timeCutOff) {
      System.out.println("ERROR: Current match time " + DriverStation.getMatchTime() + " exceeds the cutoff time of " + Constants.Climber.timeCutOff);
      cancelled = true;
    } else if (SmartDashboard.getBoolean("climber/Climbers enabled", false)) {
      System.out.println("ERROR: Attempt to use climbers, but they are disabled");
      cancelled = true;
    }
    if (cancelled) {
      cancel();
      LEDSubsystem.setTempState(TempState.ERROR);
      return;
    }
    cancelled = false;
    LEDSubsystem.setTempState(ledState);
    s_Climber.setPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Climber.stop();

    if (cancelled) {
      LEDSubsystem.setTempState(TempState.ERROR);
    } else {
      LEDSubsystem.clearTempState();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (cancelled) {
      return true;
    }
    return Math.abs(s_Climber.getPosition() - position) < Constants.Climber.positionError;
  }
}