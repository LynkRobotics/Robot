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
    private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton shooterButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton ejectButton = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* Different Position Test Buttons */
    private final JoystickButton ampButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton subwooferButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton midLineButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton visionButton = new JoystickButton(driver, XboxController.Button.kY.value);

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

        // TODO Restore default idle
        //s_Shooter.setDefaultCommand(Commands.startEnd(s_Shooter::idle, () -> {}, s_Shooter));
        s_Index.setDefaultCommand(Commands.startEnd(s_Index::stop, () -> {}, s_Index));

        // Default named commands for PathPlanner
        // TODO Delay from SmartDashboard
        NamedCommands.registerCommand("Startup delay", Commands.print("Begin startup delay")
            .andThen(Commands.waitSeconds(2.0)).andThen(Commands.print("End startup delay")));
        NamedCommands.registerCommand("Shoot", new ShootCommand(s_Shooter, s_Index).raceWith(Commands.waitSeconds(1.00)));      
        NamedCommands.registerCommand("Intake note", new IntakeCommand(s_Intake, s_Index, driver));

        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // During calibration allow for direct control
        //SmartDashboard.putNumber("Shooter voltage direct", 0.0);
        //SmartDashboard.putData("Set shooter voltage", s_Shooter.runOnce(() -> { s_Shooter.setVoltage(SmartDashboard.getNumber("Shooter voltage direct", 0)); }));
        //SmartDashboard.putData("Stop shooter", s_Shooter.runOnce(() -> { s_Shooter.setVoltage(0); }));

        // Allow for direct RPM setting
        //SmartDashboard.putNumber("Shooter RPM direct", 3000.0);
        //SmartDashboard.putData("Set shooter RPM", s_Shooter.runOnce(() -> { s_Shooter.setRPM(SmartDashboard.getNumber("Shooter RPM direct", 0.0)); }));
        SmartDashboard.putNumber("Shooter top RPM", 1000.0);
        SmartDashboard.putNumber("Shooter bottom RPM", 1000.0);
        SmartDashboard.putData("Idle shooter", s_Shooter.runOnce(() -> { s_Shooter.setRPM(500); }));

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
        intakeButton.whileTrue(new IntakeCommand(s_Intake, s_Index, driver));
        // TODO Restore normal shoot command operation: shooterButton.whileTrue(new ShootCommand(s_Shooter, s_Index));
        shooterButton.whileTrue(new ShootCommand(s_Shooter, s_Index,
            () -> SmartDashboard.getNumber("Shooter top RPM", 0.0), () -> SmartDashboard.getNumber("Shooter bottom RPM", 0.0)));

        /* Buttons to set the next shot */
        ampButton.onTrue(Commands.runOnce(() -> { s_Shooter.setTargetSpeed(Speed.AMP); }));
        subwooferButton.onTrue(Commands.runOnce(() -> { s_Shooter.setTargetSpeed(Speed.SUBWOOFER); }));
        midLineButton.onTrue(Commands.runOnce(() -> { s_Shooter.setTargetSpeed(Speed.MIDLINE); }));
        // visionButton.onTrue(Commands.runOnce(() -> { s_Shooter.setTargetSpeed(null); }));
        visionButton.onTrue(Commands.runOnce(() -> { s_Shooter.setTargetSpeed(Speed.PODIUM); }));

        ejectButton.whileTrue(new EjectCommand(s_Intake, s_Index));
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