package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AimCommand extends Command {
    private final Swerve s_Swerve;
    private final VisionSubsystem s_Vision;

    public AimCommand(Swerve s_Swerve, VisionSubsystem s_Vision) {
        this.s_Swerve = s_Swerve;
        this.s_Vision = s_Vision;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        // TODO Remove code duplication with TeleopSwerve (or need for this entirely)
        double rotationVal = 0.0;
        double angleError = s_Vision.angleError().getDegrees();

        // TODO Tune!
        // TODO More precise from longer distance
        // TODO Turn into a PID, etc.
        double magnitude = Math.abs(angleError);
        if (magnitude > 10.0) {
            rotationVal = 0.10 * Math.signum(angleError);
        } else if (magnitude > 7.0) {
            rotationVal = 0.06 * Math.signum(angleError);
        } else if (magnitude > 3.0) {
            rotationVal = 0.04 * Math.signum(angleError);
        } else if (magnitude > 0.3) {
            rotationVal = 0.03 * Math.signum(angleError);
        } else {
            rotationVal = 0.0;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(0, 0),
            rotationVal * Constants.Swerve.maxAngularVelocity,
            false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 0.0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}