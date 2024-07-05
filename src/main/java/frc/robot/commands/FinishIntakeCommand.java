// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.TempState;
import frc.robot.subsystems.ShooterSubsystem;

public class FinishIntakeCommand extends Command {
  private final IntakeSubsystem intake;
  private final IndexSubsystem index;
  private final ShooterSubsystem shooter;

  public FinishIntakeCommand(IntakeSubsystem intake, IndexSubsystem index, ShooterSubsystem shooter) {
    addRequirements(intake, index);
    this.intake = intake;
    this.index = index;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Motors should already be started
    LEDSubsystem.setTempState(TempState.INTAKING2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    index.stop();

    LEDSubsystem.clearTempState();

    if (intake.isStuck()) {
      CommandScheduler.getInstance().schedule(new EjectCommand(intake, index, shooter).withTimeout(1.5));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isStuck() || index.haveNote();
  }
}