// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private final ShooterSubsystem shooter;
  private final IndexSubsystem index;
  private boolean feeding = false;

  public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index) {
    addRequirements(shooter, index);
    this.shooter = shooter;
    this.index = index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.shoot();
    feeding = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!feeding && shooter.isReady()) {
      index.feed();
      feeding = true;
    }
    shooter.shoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.idle();
    index.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO Consider ending command when shooter is done (especially for Auto)
    return false;
  }
}