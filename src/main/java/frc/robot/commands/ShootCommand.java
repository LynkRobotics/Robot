// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LEDSubsystem.TempState;
import frc.robot.subsystems.PoseSubsystem.Target;
import frc.robot.subsystems.PoseSubsystem.Zone;
import frc.robot.subsystems.ShooterSubsystem.ShotType;

public class ShootCommand extends Command {
  private final ShooterSubsystem shooter;
  private final IndexSubsystem index;
  private boolean feeding = false;
  DoubleSupplier topSupplier = null;
  DoubleSupplier bottomSupplier = null;
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private boolean cancelled = false;
  private boolean gone = false;
  private Timer postShotTimer = new Timer();
  private boolean autoAim = true;
  private boolean shooterReady = false;
  private boolean seenTarget = false;
  private ShotType shot = ShotType.AUTO;
  private Target target = Target.SPEAKER;

  public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index) {
    addRequirements(shooter, index);
    this.shooter = shooter;
    this.index = index;
    assert(vision != null);

    SmartDashboard.putBoolean("pose/Update when shooting", true);
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

  Target zoneToTarget(Zone zone) {
    if (zone == Zone.FAR) {
      return Target.FAR_SHUTTLE;
    } else if (zone == Zone.MIDDLE) {
      return Target.SHUTTLE;
    } else {
      return Target.SPEAKER;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DogLog.log("Shooter/Status", "Shot triggered");

    shot = shooter.nextShot();
    if (shot == ShotType.AUTO) {
      target = zoneToTarget(PoseSubsystem.getZone());
    } else if (shot == ShotType.DUMP) {
      target = Target.FIXED_DUMP;
    } else if (shot == ShotType.SLIDE) {
      target = Target.FIXED_SLIDE;
    } else if (shot == ShotType.AMP) {
      target = Target.FIXED_AMP;
    } else {
      target = Target.SPEAKER;
    }
    LEDSubsystem.setTempState(TempState.SHOOTING);
    shooterReady = false;
    cancelled = false;
    seenTarget = false;
    feeding = false;
    gone = false;

    if (topSupplier != null && bottomSupplier != null) {
      shooter.setCurrentSpeed(topSupplier.getAsDouble(), bottomSupplier.getAsDouble());
    } else {
      if (!shooter.shoot()) {
        DogLog.log("Shooter/Status", "ERROR: Cancelling ShootCommand due to shoot() failure");
        LEDSubsystem.setTempState(TempState.ERROR);
        cancelled = true;
        cancel();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PoseSubsystem s_Pose = PoseSubsystem.getInstance();

    if (cancelled) {
      return;
    }
    if (shooter.requireVision() && !seenTarget) {
      seenTarget = vision.haveSpeakerTarget();
      if (!seenTarget) {
        DogLog.log("Shooter/Status", "Shoot Command waiting for speaker target");
        return;
      }
    }
    boolean precise = target == Target.SPEAKER && s_Pose.getDistance(Target.SPEAKER) > Constants.Shooter.farDistance;
    if (!feeding && shooter.isReady(precise)) {
      boolean aligned = !autoAim || !SmartDashboard.getBoolean("Aiming enabled", true); // "Aligned" if not automatic aiming
      if (!shooterReady) {
        DogLog.log("Shooter/Status", "Shooter is ready");
        shooterReady = true;
      }

      if (!aligned) {
        // TODO Simplify this if/else sequence
        if (shooter.usingVision()) {
          // TODO Revisit this conditional
          // Aligned if vision is aligned with target
          aligned = vision.haveTarget() && Math.abs(vision.angleError().getDegrees()) < Constants.Shooter.maxAngleError.get(Target.SPEAKER);
        } else {
          aligned = PoseSubsystem.getInstance().targetAligned(target);
        }
      }
      if (aligned) {
        index.feed();
        feeding = true;
        DogLog.log("Shooter/Status", String.format("Shooting from vision angle %01.1f deg @ %01.1f inches",
          vision.angleToTarget(Target.SPEAKER).getDegrees(), Units.metersToInches(vision.distanceToTarget(Target.SPEAKER))));
        DogLog.log("Shooter/Shot Pose", s_Pose.getPose());
        if (shooter.usingVision() && DriverStation.isAutonomousEnabled() && SmartDashboard.getBoolean("pose/Update when shooting", true)) {
          Pose2d pose = vision.lastPose();
          DogLog.log("Shooter/Status", "Setting pose based on vision: " + pose);
          s_Pose.setPose(pose);
        }
      }
    }
    if (topSupplier == null || bottomSupplier == null) {
      // Update shooter speed every iteration, unless we specifically set a certain speed at the onset of the command
      if (!shooter.shoot()) {
        DogLog.log("Shooter/Status", "Cancelling ShootCommand due to shoot() failure [2]");
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