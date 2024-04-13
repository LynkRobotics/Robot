package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private final Swerve s_Swerve;
    private final ShooterSubsystem s_Shooter;
    private final VisionSubsystem s_Vision;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private DoubleSupplier speedLimitRotSupplier;
    private boolean inProgress = false;

    public TeleopSwerve(Swerve s_Swerve, ShooterSubsystem s_Shooter, VisionSubsystem s_Vision, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier speedLimitRotSupplier) {
        this.s_Swerve = s_Swerve;
        this.s_Shooter = s_Shooter;
        this.s_Vision = s_Vision;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.speedLimitRotSupplier = speedLimitRotSupplier;

        SmartDashboard.putBoolean("Aiming enabled", true);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        // Driver position is inverted for Red alliance, so adjust field-oriented controls
        if (Robot.isRed()) {
            translationVal *= -1.0;
            strafeVal *= -1.0;
        }

        /* Override rotation if using vision to aim */
        if (SmartDashboard.getBoolean("Aiming enabled", true)) {
            if (s_Shooter.isAutoAimingActive()) {
                Rotation2d angleError = s_Shooter.usingVision() ? s_Vision.angleError() : (s_Shooter.dumping() ? s_Swerve.dumpShotError() : (s_Shooter.sliding() ? s_Swerve.slideShotError() : new Rotation2d()));
                
                if (!inProgress) {
                    Swerve.angleErrorReset();
                }
                inProgress = true;
                if (s_Shooter.usingVision() && !s_Vision.haveTarget()) {
                    rotationVal = 0.0;
                } else {
                    rotationVal = Swerve.angleErrorToSpeed(angleError);
                }
            } else if (Math.abs(rotationVal) < Constants.aimingOverride) {
                /* Testing -- auto-aim when available */
                if (IndexSubsystem.getInstance().haveNote() && s_Vision.haveSpeakerTarget()) {                
                    if (!inProgress) {
                        Swerve.angleErrorReset();
                    }
                    inProgress = true;
                    rotationVal = Swerve.angleErrorToSpeed(s_Vision.angleError());
                } else {
                    inProgress = false;
                }
                inProgress = false;
            } else {
                inProgress = false;
            }
        } else {
            inProgress = false;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            rotationVal * Constants.Swerve.maxAngularVelocity * speedLimitRotSupplier.getAsDouble(), 
            true
        );
        // SmartDashboard.putNumber("rotationValue", speedLimitRotSupplier.getAsDouble());
    }
}