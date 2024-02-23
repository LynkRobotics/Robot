// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  private final IntakeSubsystem intake;
  private final IndexSubsystem index;

  public OuttakeCommand(IntakeSubsystem intake, IndexSubsystem index) {
    addRequirements(intake, index);
    this.intake = intake;
    this.index = index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.eject();
    index.eject();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    index.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}