// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LEDSubsystem.TempState;

public class ShootCommand extends Command {
  private final ShooterSubsystem shooter;
  private final IndexSubsystem index;
  private Swerve swerve = null;
  private boolean feeding = false;
  DoubleSupplier topSupplier = null;
  DoubleSupplier bottomSupplier = null;
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private boolean cancelled = false;
  private boolean gone = false;
  private Timer postShotTimer = new Timer();
  private boolean autoAim = true;
  private boolean shooterReady = false;

  public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index) {
    addRequirements(shooter, index);
    this.shooter = shooter;
    this.index = index;
    assert(vision != null);
  }

  public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index, Swerve swerve) {
    this(shooter, index);
    this.swerve = swerve;
  }

  public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index, boolean autoAim) {
    this(shooter, index);
    this.autoAim = autoAim;
  }

  public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index, DoubleSupplier topSupplier, DoubleSupplier bottomSupplier) {
    this(shooter, index);
    this.topSupplier = topSupplier;
    this.bottomSupplier = bottomSupplier;
    this.autoAim = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Spinning up shooter");
    shooterReady = false;
    LEDSubsystem.setTempState(TempState.SHOOTING);
    cancelled = false;
    feeding = false;
    gone = false;

    if (topSupplier != null && bottomSupplier != null) {
      shooter.shoot(topSupplier.getAsDouble(), bottomSupplier.getAsDouble());
    } else {
      if (!DriverStation.isAutonomous() && shooter.usingVision()) {
        cancelled = !vision.haveTarget(); // TODO Consider removing this vision is integrated into poses
      }
      if (!cancelled) {
        if (!shooter.shoot()) {
          cancelled = true;
        }
      }
      if (cancelled) {
        LEDSubsystem.setTempState(TempState.ERROR);
        cancel();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (cancelled) {
      return;
    }
    if (!feeding && shooter.isReady()) {
      boolean aligned = !autoAim || !SmartDashboard.getBoolean("Aiming enabled", true); // "Aligned" if not automatic aiming

      if (!shooterReady) {
        // System.out.println("Shooter is ready");
        shooterReady = true;
      }

      if (!aligned) {
        if (shooter.usingVision()) {
          // Aligned if vision is aligned with target
          aligned = vision.haveTarget() && Math.abs(vision.angleError().getDegrees()) < Constants.Vision.maxAngleError;
        } else if (shooter.dumping()) {
          if (swerve == null) {
            System.out.println("ERROR: Cannot aim for dumping without swerve object");
          } else {
            aligned = swerve.dumpShotAligned();
          }
        } else if (shooter.sliding()) {
          if (swerve == null) {
            System.out.println("ERROR: Cannot aim for sliding without swerve object");
          } else {
            aligned = swerve.slideShotAligned();
          }
        } else {
          // "Aligned" because all other shots don't require alignment
          aligned = true;
        }
      }
      if (aligned) {
        index.feed();
        feeding = true;
      }
    }
    if (topSupplier == null || bottomSupplier == null) {
      // Update shooter speed every iteration, unless we specifically set a certain speed at the onset of the command
      if (!shooter.shoot()) {
        cancelled = true;
        cancel();
        LEDSubsystem.setTempState(TempState.ERROR);
      }
    }
    if (feeding) {
      if (!gone && !index.haveNote()) {
        postShotTimer.restart();
        gone = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop feeding
    index.stop();

    // Restore idle speed
    shooter.idle();

    // Restore default shot
    shooter.setNextShot(null);

    // Adjust LED state
    if (interrupted) {
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
    if (gone && postShotTimer.hasElapsed(Constants.Shooter.postShotTimeout)) {
      return true;
    }
    return false;
  }
}