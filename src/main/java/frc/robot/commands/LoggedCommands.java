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

  public static void logInit(Command command) {
    DogLog.log("Robot/Status", "Running " + command.getName());
  }

  public static void logFinish(Command command, boolean interrupted) {
    DogLog.log("Robot/Status", "Finished " + command.getName() + (interrupted ? " (interrupted)" : ""));
  }

  public static Command log(Command command) {
    return new WrapperCommand(command) { 
      @Override
      public void initialize() {
        logInit(command);
        super.initialize();
      }

      @Override
      public void end(boolean interrupted) {
        logFinish(command, interrupted);
        super.end(interrupted);
      }
    }.withName(command.getName() + " (Logged)");
  }

  /* The following map to the static utilities from the standard Commands class */

  public static Command none() {
    return log(Commands.none());
  }

  public static Command idle(Subsystem... requirements) {
    return log(Commands.idle(requirements));
  }

  public static Command runOnce(Runnable action, Subsystem... requirements) {
    return log(Commands.runOnce(action, requirements));
  }

  public static Command run(Runnable action, Subsystem... requirements) {
    return log(Commands.run(action, requirements));
  }

  public static Command startEnd(Runnable start, Runnable end, Subsystem... requirements) {
    return log(Commands.startEnd(start, end, requirements));
  }

  public static Command runEnd(Runnable run, Runnable end, Subsystem... requirements) {
    return log(Commands.runEnd(run, end, requirements));
  }

  public static Command print(String message) {
    return log(Commands.print(message));
  }

  public static Command waitSeconds(double seconds) {
    return log(Commands.waitSeconds(seconds));
  }

  public static Command waitUntil(BooleanSupplier condition) {
    return log(Commands.waitUntil(condition));
  }

  public static Command either(Command onTrue, Command onFalse, BooleanSupplier selector) {
    return log(Commands.either(onTrue, onFalse, selector));
  }

  public static <K> Command select(Map<K, Command> commands, Supplier<? extends K> selector) {
    return log(Commands.select(commands, selector));
  }
  public static Command defer(Supplier<Command> supplier, Set<Subsystem> requirements) {
    return log(Commands.defer(supplier, requirements));
  }

  public static Command deferredProxy(Supplier<Command> supplier) {
    return log(Commands.deferredProxy(supplier));
  }

  public static Command sequence(Command... commands) {
    return log(Commands.sequence(commands));
  }

  public static Command repeatingSequence(Command... commands) {
    return log(Commands.repeatingSequence(commands));
  }

  public static Command parallel(Command... commands) {
    return log(Commands.parallel(commands));
  }

  public static Command race(Command... commands) {
    return log(Commands.race(commands));
  }
  public static Command deadline(Command deadline, Command... otherCommands) {
    return log(Commands.deadline(deadline, otherCommands));
  }

  private LoggedCommands() {
    throw new UnsupportedOperationException("This is a utility class");
  }
}