// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

// Provide a Commands-like interface, but producing LoggedCommands
public class LoggedCommands {

  // Try a simple method of wrapping a command
  public static Command wrap1(Command command) {
    return Commands.sequence(
      Commands.runOnce(() -> {
        DogLog.log("Robot/Status", "Running " + command.getName());
      }),
      command,
      Commands.runOnce(() -> {
        DogLog.log("Robot/Status", "Finished " + command.getName());
      })
    );
  }

  // Try another simple method of wrapping a command
  public static Command wrap2(Command command) {
    return command.deadlineWith(
      Commands.startEnd(
        () -> DogLog.log("Robot/Status", "Running " + command.getName()),
        () -> DogLog.log("Robot/Status", "Finished " + command.getName())
      )
    );
  }

  public static Command wrap3(Command command) {
    return new WrapperCommand(command) { 
      @Override
      public void initialize() {
        DogLog.log("Robot/Status", "Running " + getName());
        super.initialize();
      }

      @Override
      public void end(boolean interrupted) {
        DogLog.log("Robot/Status", "Finished " + getName() + (interrupted ? " (interrupted)" : ""));
        super.end(interrupted);
      }
    };
  }

  public static Command log(Command command) {
    return new LoggedCommand(command);
  }

  public static Command none() {
    return new LoggedCommand(Commands.none());
  }

  public static Command idle(Subsystem... requirements) {
    return new LoggedCommand(Commands.idle(requirements));
  }

  public static Command runOnce(Runnable action, Subsystem... requirements) {
    return new LoggedCommand(Commands.runOnce(action, requirements));
  }

  public static Command run(Runnable action, Subsystem... requirements) {
    return new LoggedCommand(Commands.run(action, requirements));
  }

  public static Command startEnd(Runnable start, Runnable end, Subsystem... requirements) {
    return new LoggedCommand(Commands.startEnd(start, end, requirements));
  }

  public static Command runEnd(Runnable run, Runnable end, Subsystem... requirements) {
    return new LoggedCommand(Commands.runEnd(run, end, requirements));
  }

  public static Command print(String message) {
    return new LoggedCommand(Commands.print(message));
  }

  public static Command waitSeconds(double seconds) {
    return new LoggedCommand(Commands.waitSeconds(seconds));
  }

  public static Command waitUntil(BooleanSupplier condition) {
    return new LoggedCommand(Commands.waitUntil(condition));
  }

  public static Command either(Command onTrue, Command onFalse, BooleanSupplier selector) {
    return new LoggedCommand(Commands.either(onTrue, onFalse, selector));
  }

  public static <K> Command select(Map<K, Command> commands, Supplier<? extends K> selector) {
    return new LoggedCommand(Commands.select(commands, selector));
  }
  public static Command defer(Supplier<Command> supplier, Set<Subsystem> requirements) {
    return new LoggedCommand(Commands.defer(supplier, requirements));
  }

  public static Command deferredProxy(Supplier<Command> supplier) {
    return new LoggedCommand(Commands.deferredProxy(supplier));
  }

  public static Command sequence(Command... commands) {
    return new LoggedCommand(Commands.sequence(commands));
  }

  public static Command repeatingSequence(Command... commands) {
    return new LoggedCommand(Commands.repeatingSequence(commands));
  }

  public static Command parallel(Command... commands) {
    return new LoggedCommand(Commands.parallel(commands));
  }

  public static Command race(Command... commands) {
    return new LoggedCommand(Commands.race(commands));
  }
  public static Command deadline(Command deadline, Command... otherCommands) {
    return new LoggedCommand(Commands.deadline(deadline, otherCommands));
  }

  private LoggedCommands() {
    throw new UnsupportedOperationException("This is a utility class");
  }
}