package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton intakingButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton shooterButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Different Position Test Buttons */
    private final JoystickButton ampButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton subwooferButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton midLineButton = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final IndexSubsystem s_Index = new IndexSubsystem();

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        // TODO Remove temp limitation
                        () -> -driver.getRawAxis(translationAxis) * 0.3,
                        () -> -driver.getRawAxis(strafeAxis) * 0.3,
                        () -> -driver.getRawAxis(rotationAxis) * 0.5));

        s_Shooter.setDefaultCommand(Commands.startEnd(s_Shooter::idle, () -> {}, s_Shooter));
        s_Index.setDefaultCommand(Commands.startEnd(s_Index::stop, () -> {}, s_Index));

        // Default named commands for PathPlanner
        // TODO Delay from SmartDashboard
        // TODO Replace prints with real commands
        NamedCommands.registerCommand("Startup delay", new PrintCommand("Begin startup delay")
                .andThen(new WaitCommand(2.0)).andThen(new PrintCommand("End startup delay")));
        //NamedCommands.registerCommand("Shoot", new PrintCommand("COMMAND: Shoot").andThen(new WaitCommand(3.0)));
        NamedCommands.registerCommand("Shoot", new ShootCommand(s_Shooter, Speed.MIDLINE).alongWith(new IndexCommand(s_Index)).raceWith(new WaitCommand(1.25)));      
        NamedCommands.registerCommand("Intake note", new IntakeCommand(s_Intake).alongWith(new IndexCommand(s_Index)).until(s_Index.getIndexSensor()));
        //NamedCommands.registerCommand("Intake note", new PrintCommand("COMMAND: Intake note").andThen(new WaitCommand(1.0)));

        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

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
        /* Driver Buttons */
        intakingButton.whileTrue(Commands.run(s_Intake::intake, s_Intake)
                .alongWith(Commands.run(s_Index::index, s_Index)).until(s_Index.getIndexSensor()));

        shooterButton
                .whileTrue(Commands.run(s_Shooter::shoot, s_Shooter).alongWith(Commands.run(s_Index::feed, s_Index)));

        ampButton.whileTrue(new ShootCommand(s_Shooter, Speed.AMP));

        subwooferButton.whileTrue(new ShootCommand(s_Shooter, Speed.SUBWOOFER));

        midLineButton.whileTrue(new ShootCommand(s_Shooter, Speed.MIDLINE));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(s_Swerve);
        // return new PathPlannerAuto("Straight score");
        return autoChooser.getSelected();
    }
}
