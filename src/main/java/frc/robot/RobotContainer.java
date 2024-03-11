package frc.robot;

import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem.Speed;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    //private final Joystick driver = new Joystick(0);
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private final Supplier<Double> translation = driver::getLeftY;
    private final Supplier<Double> strafe = driver::getLeftX;
    private final Supplier<Double> rotation = driver::getRightX;

    /* Driver Buttons */
    private final Trigger intakeButton = driver.leftBumper();
    private final Trigger shooterButton = driver.rightBumper();
    private final Trigger ejectButton = driver.start();
    private final Trigger climberExtendButton = driver.leftTrigger();
    private final Trigger climberRetractButton = driver.rightTrigger();
    
    /* Different Position Test Buttons */
    private final Trigger ampButton = driver.a();
    private final Trigger defaultShotButton = driver.b();
    private final Trigger getNoteButton = driver.x();
    private final Trigger trapButton = driver.y();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final IndexSubsystem s_Index = new IndexSubsystem();
    private final ClimberSubsystem s_Climber = new ClimberSubsystem();
    @SuppressWarnings ("unused")
    private final LEDSubsystem s_Led = new LEDSubsystem();
    private final VisionSubsystem s_Vision = new VisionSubsystem();

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        s_Shooter,
                        s_Vision,
                        () -> -translation.get() * Constants.driveStickSensitivity,
                        () -> -strafe.get() * Constants.driveStickSensitivity,
                        () -> -rotation.get() * Constants.turnStickSensitivity));

        s_Shooter.setDefaultCommand(Commands.startEnd(s_Shooter::idle, () -> {}, s_Shooter));
        s_Index.setDefaultCommand(Commands.startEnd(s_Index::stop, () -> {}, s_Index));

        // Default named commands for PathPlanner
        SmartDashboard.putNumber("auto/Startup delay", 0.0);
        NamedCommands.registerCommand("Startup delay", Commands.print("Begin startup delay")
            .andThen(new DeferredCommand(() ->Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()))
            .andThen(Commands.print("End startup delay")));
        NamedCommands.registerCommand("Shoot",
            Commands.print("Named 'Shoot' command starting")
            .andThen(
                (Commands.print("Before ShootCommand").andThen(new ShootCommand(s_Shooter, s_Index)).andThen(Commands.print("After ShootCommand")))
                 .raceWith(Commands.print("Before AimCommand").andThen(new AimCommand(s_Swerve, s_Vision)).andThen(Commands.print("After AimCommand")))
                 .raceWith(Commands.print("Before waitSeconds").andThen(Commands.waitSeconds(2.50)).andThen(Commands.print("After waitSeconds"))))
            .andThen(Commands.print("After race group"))
            //.andThen(Commands.print("Before idle").andThen(Commands.startEnd(s_Shooter::idle, () -> {}, s_Shooter)).andThen(Commands.print("After idle")))
            .andThen(Commands.print("Named 'Shoot' command ending"))
        );
        NamedCommands.registerCommand("Shoot without aiming",
            Commands.print("Begin shot w/o aim")
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.50))))
            .andThen(Commands.print("Shot w/o aim complete"))
            //.andThen(Commands.startEnd(s_Shooter::idle, () -> {}, s_Shooter))
            .andThen(Commands.print("Idling again"))
            
        );
        NamedCommands.registerCommand("Intake note",
            Commands.print("Beginning intake")
            .andThen(new IntakeCommand(s_Intake, s_Index, driver.getHID()))
            .andThen(Commands.print("Intake complete")));

        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("auto/Auto Chooser", autoChooser);

        // During calibration allow for direct control
        //SmartDashboard.putNumber("Shooter voltage direct", 0.0);
        //SmartDashboard.putData("Set shooter voltage", s_Shooter.runOnce(() -> { s_Shooter.setVoltage(SmartDashboard.getNumber("Shooter voltage direct", 0)); }));
        //SmartDashboard.putData("Stop shooter", s_Shooter.runOnce(() -> { s_Shooter.setVoltage(0); }));

        // Allow for direct RPM setting
        SmartDashboard.putBoolean("Direct set RPM", false);
        SmartDashboard.putNumber("Shooter top RPM", 1000.0);
        SmartDashboard.putNumber("Shooter bottom RPM", 1000.0);
        SmartDashboard.putData("Idle shooter", s_Shooter.runOnce(() -> { s_Shooter.setRPM(500); }));
        SmartDashboard.putData("Zero Gyro", Commands.runOnce(s_Swerve::zeroGyro, s_Swerve)); //TODO: Test

        // Allow for direct climber control
        SmartDashboard.putData("Stop climbers", Commands.runOnce(s_Climber::stop, s_Climber));
        SmartDashboard.putData("Left down slow", Commands.runOnce(() -> { s_Climber.applyVoltageLeft(Constants.Climber.slowVoltage); }, s_Climber));
        SmartDashboard.putData("Right down slow", Commands.runOnce(() -> { s_Climber.applyVoltageRight(Constants.Climber.slowVoltage); }, s_Climber));

        SmartDashboard.putNumber("Left climber voltage", 0.0);
        SmartDashboard.putNumber("Right climber voltage", 0.0);
        SmartDashboard.putData("Set climber voltage", Commands.runOnce(() -> { s_Climber.applyVoltageLeft(SmartDashboard.getNumber("Left climber voltage", 0.0)); s_Climber.applyVoltageRight(SmartDashboard.getNumber("Right climber voltage", 0.0));}, s_Climber));

/*
        SmartDashboard.putNumber("Left climber target position", 0.0);
        SmartDashboard.putData("Set left climber position", Commands.runOnce(() -> { s_Climber.setPositionLeft(SmartDashboard.getNumber("Left climber target position", 0.0));}, s_Climber));
        SmartDashboard.putNumber("Right climber target position", 0.0);
        SmartDashboard.putData("Set right climber position", Commands.runOnce(() -> { s_Climber.setPositionRight(SmartDashboard.getNumber("Right climber target position", 0.0));}, s_Climber));
*/

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        SmartDashboard.putBoolean("Shooter intake", false);

        /* Driver Buttons */
        intakeButton.whileTrue(Commands.either(
            new ShooterIntakeCommand(s_Shooter, s_Index, driver.getHID()),
            new IntakeCommand(s_Intake, s_Index, driver.getHID()),
            () -> SmartDashboard.getBoolean("Shooter intake", false)));
        shooterButton.whileTrue(
            Commands.either(new ShootCommand(s_Shooter, s_Index,
                () -> SmartDashboard.getNumber("Shooter top RPM", 0.0),
                () -> SmartDashboard.getNumber("Shooter bottom RPM", 0.0)),
            new ShootCommand(s_Shooter, s_Index),
            () -> SmartDashboard.getBoolean("Direct set RPM", false)));
        climberExtendButton.onTrue(new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_Climber));
        climberRetractButton.whileTrue(new ClimberPositionCommand(Constants.Climber.retractedPosition, LEDSubsystem.TempState.RETRACTING, s_Climber));

        /* Buttons to set the next shot */
        ampButton.onTrue(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.AMP); }));
        defaultShotButton.onTrue(Commands.runOnce(() -> { s_Shooter.setNextShot(null); }));
        getNoteButton.onTrue(Commands.print("Getting notes not yet implemented"));
        trapButton.onTrue(Commands.print("Trap shooting not yet implemented"));

        ejectButton.whileTrue(new EjectCommand(s_Intake, s_Index));
    }

    public void hack(){
        s_Swerve.hack();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}