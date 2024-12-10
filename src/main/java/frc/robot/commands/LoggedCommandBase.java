// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import dev.doglog.DogLog;

// Wraps a command with logging
public class LoggedCommandBase extends Command {
  public LoggedCommandBase() {
    super();
  }

  @Override
  public void initialize() {
    DogLog.log("Robot/Status", "Running " + getName());
  }

  @Override
  public void end(boolean interrupted) {
    DogLog.log("Robot/Status", "Finished " + getName() + (interrupted ? " (interrupted)" : ""));
  }
}