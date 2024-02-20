// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexCommand extends Command {
  /** Creates a new IndexCommand. */
  private final IndexSubsystem s_Index;
  private final ShooterSubsystem s_Shooter;
  private boolean running = false;

  public IndexCommand(IndexSubsystem s_Index, ShooterSubsystem s_Shooter) {
    addRequirements(s_Index);
    this.s_Index = s_Index;
    this.s_Shooter = s_Shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    running = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!running && s_Shooter.isReady()) {
      s_Index.index();
      running = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Index.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
