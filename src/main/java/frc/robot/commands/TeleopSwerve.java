package frc.robot.commands;

import frc.lib.util.TunableOption;
import frc.robot.Constants;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.Pose;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.PoseSubsystem.Target;
import frc.robot.subsystems.PoseSubsystem.Zone;
import frc.robot.subsystems.ShooterSubsystem.ShotType;

import static frc.robot.Options.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private final Swerve s_Swerve;
    private final ShooterSubsystem s_Shooter;
    private final VisionSubsystem s_Vision;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private DoubleSupplier speedLimitRotSupplier;
    private PoseSubsystem s_Pose = null;
    private Rotation2d lastAngle = null;
    private AimingMode aimingMode = AimingMode.MANUAL;
    private static final TunableOption optFullFieldAiming = new TunableOption("Full field aiming", true);

    private enum AimingMode {
        MANUAL,
        TARGET,
        MAINTAIN
    }

    public TeleopSwerve(Swerve s_Swerve, ShooterSubsystem s_Shooter, VisionSubsystem s_Vision, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier speedLimitRotSupplier) {
        this.s_Swerve = s_Swerve;
        this.s_Shooter = s_Shooter;
        this.s_Vision = s_Vision;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.speedLimitRotSupplier = speedLimitRotSupplier;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if (s_Pose == null) {
            s_Pose = PoseSubsystem.getInstance();
        }

        // Driver position is inverted for Red alliance, so adjust field-oriented controls
        if (Robot.isRed()) {
            translationVal *= -1.0;
            strafeVal *= -1.0;
        }

        // Override rotation automatically aiming
        if (Swerve.isAimingActive()) {
            Rotation2d angleError;

            if (s_Shooter.requiresVision()) {
                if (s_Vision.haveTarget()) {
                    angleError = s_Vision.angleError();
                } else {
                    angleError = new Rotation2d();
                }
            } else {
                angleError = PoseSubsystem.getInstance().targetAngleError(s_Shooter.currentTarget());
            }
            
            if (aimingMode != AimingMode.TARGET) {
                PoseSubsystem.angleErrorReset();
                aimingMode = AimingMode.TARGET;
            }

            rotationVal = PoseSubsystem.angleErrorToSpeed(angleError);
        } else if (Math.abs(rotationVal) < Constants.aimingOverride) {
            boolean haveNote = IndexSubsystem.getInstance().haveNote();

            if (haveNote && s_Vision.haveSpeakerTarget()) {              
                if (aimingMode != AimingMode.TARGET) {
                    PoseSubsystem.angleErrorReset();
                    aimingMode = AimingMode.TARGET;
                }
                rotationVal = PoseSubsystem.angleErrorToSpeed(optShootWithVision.get () ? s_Vision.angleError() : s_Pose.targetAngleError(Target.SPEAKER));
            } else if (haveNote && optFullFieldAiming.get()) {
                PoseSubsystem.Zone zone = PoseSubsystem.getZone();
                Target target = ShootCommand.zoneToTarget(zone);
                Rotation2d angleError = null;

                if (zone == Zone.SPEAKER && s_Shooter.nextShot() == ShotType.AMP) {
                    if (optAimAtAmp.get()) {
                        //angleError = s_Pose.targetAngleError(Target.AMP);
                        angleError = Pose.ampAngle.minus(PoseSubsystem.getInstance().getPose().getRotation());
                    } else {
                        angleError = null; // Don't override aiming
                    }
                } else {
                    angleError = s_Pose.targetAngleError(target);
                }
                if (angleError != null) {
                    rotationVal = PoseSubsystem.angleErrorToSpeed(angleError);
                }
            } else if (PoseSubsystem.getTargetAngle() != null) {
                // Move to a fixed angle
                if (aimingMode != AimingMode.TARGET) {
                    PoseSubsystem.angleErrorReset();
                    aimingMode = AimingMode.TARGET;
                }
                rotationVal = PoseSubsystem.angleErrorToSpeed(PoseSubsystem.getTargetAngle().minus(PoseSubsystem.getInstance().getPose().getRotation()));
            } else {
                if (aimingMode == AimingMode.TARGET) {
                    aimingMode = AimingMode.MANUAL;
                }
            }
        } else {
            if (aimingMode == AimingMode.TARGET) {
                aimingMode = AimingMode.MANUAL;
            }
        }

        // Maintain angle when not explicitly changing
        Rotation2d currentAngle = s_Pose.getPose().getRotation();
        if (optMaintainAngle.get() && Math.abs(rotationVal) < Constants.aimingOverride) {
            if (lastAngle != null && aimingMode != AimingMode.TARGET) {
                if (aimingMode != AimingMode.MAINTAIN) {
                    PoseSubsystem.angleErrorReset(Pose.maintainPID);
                    lastAngle = currentAngle;
                    aimingMode = AimingMode.MAINTAIN;
                } else {
                    Rotation2d angleError = lastAngle.minus(currentAngle);
                    rotationVal = PoseSubsystem.angleErrorToSpeed(angleError, Pose.maintainPID);
                }
            }
        } else {
            lastAngle = currentAngle;
            aimingMode = AimingMode.MANUAL;
        }

        // Drive
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            rotationVal * Constants.Swerve.maxAngularVelocity * speedLimitRotSupplier.getAsDouble(), 
            true
        );
    }
}