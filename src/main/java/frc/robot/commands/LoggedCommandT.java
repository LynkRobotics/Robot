// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

// Wraps a command with logging
// Implementation was modeled in part with SequentialCommandGroup and WrappedCommand as references
public class LoggedCommandT<T extends Command> extends LoggedCommandBase {
  private final T wrappedCommand;

  public LoggedCommandT(T command) {
    wrappedCommand = command;
    for (Subsystem sub : wrappedCommand.getRequirements()) {
      addRequirements(sub);
    }
    CommandScheduler.getInstance().registerComposedCommands(wrappedCommand);
    setName(wrappedCommand.getName() + " (Logged)");
  }

  @Override
  public final void initialize() {
    super.initialize();
    wrappedCommand.initialize();
  }

  @Override
  public final void execute() {
    super.execute();
    wrappedCommand.execute();
  }

  @Override
  public final void end(boolean interrupted) {
    super.end(interrupted);
    wrappedCommand.end(interrupted);
  }

  @Override
  public final boolean isFinished() {
    return wrappedCommand.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return wrappedCommand.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return wrappedCommand.getInterruptionBehavior();
  }
}