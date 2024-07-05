package frc.robot;

import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ClimberSubsystem.ClimberSelection;
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
    private final Trigger leftClimberButton = driver.leftTrigger();
    private final Trigger rightClimberButton = driver.rightTrigger();
    
    /* Different Position Test Buttons */
    private final Trigger ampButton = driver.a();
    private final Trigger dumpShotButton = driver.b();
    private final Trigger defaultShotButton = driver.back();
    private final Trigger slideShotButton = driver.x();
    private final Trigger climberExtendButton = driver.y();
    private final Trigger ampShotButton = driver.povDown();
    private final Trigger sourceAlignButton = driver.povUp();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final IndexSubsystem s_Index = new IndexSubsystem();
    private final ClimberSubsystem s_LeftClimber = new ClimberSubsystem(ClimberSelection.LEFT);
    private final ClimberSubsystem s_RightClimber = new ClimberSubsystem(ClimberSelection.RIGHT);
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
                        () -> -rotation.get() * Constants.turnStickSensitivity,
                        s_Swerve::getSpeedLimitRot
                        ));

        s_Shooter.setDefaultCommand(Commands.startEnd(s_Shooter::idle, () -> {}, s_Shooter).withName("Shooter Idle"));
        s_Index.setDefaultCommand(Commands.startEnd(s_Index::stop, () -> {}, s_Index).withName("Index Stop"));

        SmartDashboard.putData("Command scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Shoot Command", new ShootCommand(s_Shooter, s_Index, s_Swerve).withTimeout(3.0));

        // Default named commands for PathPlanner
        SmartDashboard.putNumber("auto/Startup delay", 0.0);
        NamedCommands.registerCommand("Done", new PrintCommand("Done"));
        NamedCommands.registerCommand("Start", new PrintCommand("Hello World"));
        NamedCommands.registerCommand("Startup delay", new DeferredCommand(() ->Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()));
        NamedCommands.registerCommand("Shoot",
            Commands.print("Named 'Shoot' command starting")
            .andThen(
                (Commands.print("Before ShootCommand").andThen(new ShootCommand(s_Shooter, s_Index, s_Swerve)).andThen(Commands.print("After ShootCommand")))
                 .raceWith(Commands.print("Before AimCommand").andThen(new AimCommand(s_Swerve, s_Vision)).andThen(Commands.print("After AimCommand")))
                 .raceWith(Commands.print("Before waitSeconds").andThen(Commands.waitSeconds(2.50)).andThen(Commands.print("After waitSeconds"))))
            .andThen(Commands.print("After race group"))
            .andThen(Commands.print("Named 'Shoot' command ending"))
        );
        NamedCommands.registerCommand("Shoot without aiming",
            Commands.print("Begin shot w/o aim")
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, s_Swerve, false)
                .raceWith(Commands.waitSeconds(1.50))))
            .andThen(Commands.print("Shot w/o aim complete"))
            
        );
        NamedCommands.registerCommand("Fixed SW shot",
            Commands.print("Begin SW shot")
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.SUBWOOFER); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.50))))
            .andThen(Commands.print("SW shot complete"))
            
        );
        NamedCommands.registerCommand("Shoot OTF",
            Commands.print("Begin OTF")
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.OTF); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.50))))
            .andThen(Commands.print("Shot OTF complete"))
            
        );
        NamedCommands.registerCommand("Amp-side OTF Shot",
            Commands.print("Begin Amp-side OTF Shot")
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.AMPSIDEOTF); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.print("Amp-side OTF Shot complete"))
        );
        NamedCommands.registerCommand("Source-side OTF Shot",
            Commands.print("Begin Source-side OTF Shot")
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.SOURCESIDEOTF); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.print("Source-side OTF Shot complete"))
        );
        NamedCommands.registerCommand("Intake note",
            Commands.print("Beginning intake")
            .andThen(new StartIntakeCommand(s_Intake, s_Index, s_Shooter, driver.getHID()))
            .andThen(Commands.print("Intake complete")));

        NamedCommands.registerCommand("Amp Shot",
            Commands.print("Begin Amp Shot")
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.AMP); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.print("Amp shot complete"))
        );
        NamedCommands.registerCommand("Bloop Shot",
            Commands.print("Begin Bloop Shot")
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.BLOOP); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.print("Bloop shot complete"))
        );
        NamedCommands.registerCommand("Slide Shot",
            Commands.print("Begin Slide Shot")
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.SLIDE); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.print("Slide shot complete"))
        );
        NamedCommands.registerCommand("Short Slide Shot",
            Commands.print("Begin Short Slide Shot")
            .andThen(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.SHORTSLIDE); }))
            .andThen(
                (new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.00))))
            .andThen(Commands.print("Short Slide shot complete"))
        );
        NamedCommands.registerCommand("Override rotation", Commands.runOnce(s_Vision::enableRotationTargetOverride));
        NamedCommands.registerCommand("Restore rotation", Commands.runOnce(s_Vision::disableRotationTargetOverride));

        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("auto/Auto Chooser", autoChooser);
        buildAutos(autoChooser);

        // During calibration allow for direct control
        //SmartDashboard.putNumber("Shooter voltage direct", 0.0);
        //SmartDashboard.putData("Set shooter voltage", s_Shooter.runOnce(() -> { s_Shooter.setVoltage(SmartDashboard.getNumber("Shooter voltage direct", 0)); }));
        //SmartDashboard.putData("Stop shooter", s_Shooter.runOnce(() -> { s_Shooter.setVoltage(0); }));

        // Allow for direct RPM setting
        SmartDashboard.putBoolean("Direct set RPM", false);
        SmartDashboard.putNumber("Shooter top RPM", 1000.0);
        SmartDashboard.putNumber("Shooter bottom RPM", 1000.0);
        SmartDashboard.putData("Idle shooter", s_Shooter.runOnce(() -> { s_Shooter.setRPM(500); }));
        SmartDashboard.putData("Zero Gyro", Commands.print("Zeroing gyro").andThen(Commands.runOnce(s_Swerve::zeroGyro, s_Swerve)).andThen(Commands.print("Gyro zeroed")));
        SmartDashboard.putData("Zero heading", Commands.print("Zeroing heading").andThen(Commands.runOnce(s_Swerve::zeroHeading, s_Swerve)).andThen(Commands.print("Heading zeroed")));
        SmartDashboard.putData("Reset heading", Commands.print("Resetting heading").andThen(Commands.runOnce(s_Swerve::resetHeading, s_Swerve)).andThen(Commands.print("Heading reset")));

        // Allow for direct climber control
        SmartDashboard.putData("Stop climbers", Commands.runOnce(() -> { s_LeftClimber.stop(); s_RightClimber.stop(); }, s_LeftClimber, s_RightClimber));
        SmartDashboard.putData("Left down slow", Commands.runOnce(() -> { s_LeftClimber.applyVoltage(Constants.Climber.slowVoltage); }, s_LeftClimber));
        SmartDashboard.putData("Right down slow", Commands.runOnce(() -> { s_RightClimber.applyVoltage(Constants.Climber.slowVoltage); }, s_RightClimber));

        SmartDashboard.putNumber("Left climber voltage", 0.0);
        SmartDashboard.putNumber("Right climber voltage", 0.0);
        SmartDashboard.putData("Set climber voltage", Commands.runOnce(() -> { s_LeftClimber.applyVoltage(SmartDashboard.getNumber("Left climber voltage", 0.0)); s_RightClimber.applyVoltage(SmartDashboard.getNumber("Right climber voltage", 0.0));}, s_LeftClimber, s_RightClimber));
        SmartDashboard.putData("Zero climbers", Commands.runOnce(() -> { s_LeftClimber.zero(); s_RightClimber.zero(); }, s_LeftClimber, s_RightClimber));
        SmartDashboard.putBoolean("climber/Climbers enabled", true);

        SmartDashboard.putNumber("Left climber target position", 0.0);
        SmartDashboard.putData("Set left climber position", new ClimberPositionCommand(SmartDashboard.getNumber("Left climber target position", 0.0), LEDSubsystem.TempState.RETRACTING, s_LeftClimber));
        SmartDashboard.putNumber("Right climber target position", 0.0);
        SmartDashboard.putData("Set right climber position", new ClimberPositionCommand(SmartDashboard.getNumber("Right climber target position", 0.0), LEDSubsystem.TempState.RETRACTING, s_RightClimber));

        // Testing...
        // SmartDashboard.putData("Score in Amp", new PathPlannerAuto("Score in Amp"));
        // SmartDashboard.putData("Amp Path Command", ampPathCommand());
        
        // SmartDashboard.putNumber("Rotation value", 0.0);
        // SmartDashboard.putData("Test Rotation", Commands.startEnd(
        //     () -> { s_Swerve.drive(new Translation2d(0, 0), SmartDashboard.getNumber("Rotation value", 0) * Constants.Swerve.maxAngularVelocity, true); },
        //     () -> { s_Swerve.drive(new Translation2d(0, 0), 0.0, true); },
        //     s_Swerve).withName("Test Rotation").withTimeout(3.0));

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
        intakeButton.whileTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::enableSpeedLimit),
                Commands.either(
                    new ShooterIntakeCommand(s_Shooter, s_Index, driver.getHID()),
                    new StartIntakeCommand(s_Intake, s_Index, s_Shooter,driver.getHID()),
                    () -> SmartDashboard.getBoolean("Shooter intake", false)),
                Commands.runOnce(s_Swerve::disableSpeedLimit))
            .handleInterrupt(s_Swerve::disableSpeedLimit)
            .withName("Intake"));
        shooterButton.whileTrue(
            Commands.either(new ShootCommand(s_Shooter, s_Index,
                () -> SmartDashboard.getNumber("Shooter top RPM", 0.0),
                () -> SmartDashboard.getNumber("Shooter bottom RPM", 0.0)),
            new ShootCommand(s_Shooter, s_Index, s_Swerve),
            () -> SmartDashboard.getBoolean("Direct set RPM", false))
            .withName("Shoot"));
        climberExtendButton.onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::enableSpeedLimit),
                Commands.parallel(
                    new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_LeftClimber),
                    new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_RightClimber)))
            .withName("Extend Climbers"));
        SmartDashboard.putData("Disable speed limit", Commands.runOnce(s_Swerve::disableSpeedLimit));
        // climberExtendButton.onTrue(
        //     Commands.either(
        //         new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_LeftClimber)
        //         .alongWith(new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_RightClimber)),
        //         new ClimberPositionCommand(Constants.Climber.midPosition, LEDSubsystem.TempState.EXTENDING, s_LeftClimber)
        //         .alongWith(new ClimberPositionCommand(Constants.Climber.midPosition, LEDSubsystem.TempState.EXTENDING, s_RightClimber)),
        //         () -> s_LeftClimber.getPosition() < Constants.Climber.midPosition + 2 * Constants.Climber.positionError ||
        //               s_RightClimber.getPosition() < Constants.Climber.midPosition + 2 * Constants.Climber.positionError));

        leftClimberButton.whileTrue(new ClimberPositionCommand(Constants.Climber.retractedPosition, LEDSubsystem.TempState.RETRACTING, s_LeftClimber));
        rightClimberButton.whileTrue(new ClimberPositionCommand(Constants.Climber.retractedPosition, LEDSubsystem.TempState.RETRACTING, s_RightClimber));

        /* Buttons to set the next shot */
        ampButton.onTrue(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.AMP); }).withName("Set amp shot"));
        defaultShotButton.onTrue(Commands.runOnce(() -> { s_Shooter.setNextShot(null); }).withName("Set default shot"));
        dumpShotButton.onTrue(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.DUMP); }).withName("Set dump shot"));
        slideShotButton.onTrue(Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.SLIDE); }).withName("Set slide shot"));

        ejectButton.whileTrue(new EjectCommand(s_Intake, s_Index, s_Shooter));

        ampShotButton.whileTrue(ampPathCommand().withName("Amp path & shoot"));
        sourceAlignButton.whileTrue(sourcePathCommand().withName("Source align"));
        SmartDashboard.putData("Speaker align", speakerPathCommand());        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void buildAutos(SendableChooser<Command> chooser) {
        Command smartHG =
            Commands.sequence(
                new PathPlannerAuto("SS Angled Start to H"),
                Commands.runOnce(() -> { System.out.println("Ready for conditional part: " + s_Index.haveNote()); }),
                Commands.either(
                    Commands.print("Running H-Shoot-G-Shoot").andThen(new PathPlannerAuto("H-Shoot-G-Shoot")),
                    Commands.print("Running H-G-Shoot").andThen(new PathPlannerAuto("H-G-Shoot")),
                    s_Index::haveNote
                ),
                Commands.print("Conditional part over")
            ).withName("Smart HG");
        chooser.addOption("Smart HG", smartHG);

        // Command smartOTFHG =
        //     Commands.sequence(
        //         new PathPlannerAuto("Source-side OTF to H"),
        //         Commands.either(
        //             Commands.print("Running H-Shoot-G-Shoot").andThen(new PathPlannerAuto("H-Shoot-G-Shoot")),
        //             Commands.print("Running H-G-Shoot").andThen(new PathPlannerAuto("H-G-Shoot")),
        //             s_Index::haveNote
        //         )
        //     ).withName("Smart OTF HG");
        // chooser.addOption("Smart OTF HG", smartOTFHG);

        Command smartADEClose =
            Commands.sequence(
                new PathPlannerAuto("AS Angled + AD"),
                Commands.runOnce(() -> { System.out.println("Ready for conditional part: " + s_Index.haveNote()); }),
                Commands.either(
                    Commands.print("Running DE from close").andThen(new PathPlannerAuto("DE from close")),
                    Commands.print("Running D-E-Shoot").andThen(new PathPlannerAuto("D-E-Shoot")),
                    s_Index::haveNote
                ),
                Commands.print("Conditional part over")
            ).withName("Smart ADE from Close");
        chooser.addOption("Smart ADE from Close", smartADEClose);

        Command smartADE =
            Commands.sequence(
                new PathPlannerAuto("AS Angled + AD"),
                Commands.runOnce(() -> { System.out.println("Ready for conditional part: " + s_Index.haveNote()); }),
                Commands.either(
                    Commands.print("Running DE from A").andThen(new PathPlannerAuto("DE from A")),
                    Commands.print("Running D-E-Shoot").andThen(new PathPlannerAuto("D-E-Shoot")),
                    s_Index::haveNote
                ),
                Commands.print("Conditional part over")
            ).withName("Smart ADE");
        chooser.addOption("Smart ADE", smartADE);

        Command smartBCAD =
        Commands.sequence(
            new PathPlannerAuto("BCAD start"),
            Commands.runOnce(() -> { System.out.println("Ready for conditional part: " + s_Index.haveNote()); }),
            Commands.either(
                Commands.print("Running DE from A").andThen(new PathPlannerAuto("DE from A")),
                Commands.print("Running D-E-Shoot").andThen(new PathPlannerAuto("D-E-Shoot")),
                s_Index::haveNote
            ),
            Commands.print("Conditional part over")
        ).withName("Smart BCAD");
        chooser.addOption("Smart BCAD", smartBCAD);

        Command smartBCdirectAD =
        Commands.sequence(
            new PathPlannerAuto("BC-direct-AD start"),
            Commands.runOnce(() -> { System.out.println("Ready for conditional part: " + s_Index.haveNote()); }),
            Commands.either(
                Commands.print("Running DE from A").andThen(new PathPlannerAuto("DE from A")),
                Commands.print("Running D-E-Shoot").andThen(new PathPlannerAuto("D-E-Shoot")),
                s_Index::haveNote
            ),
            Commands.print("Conditional part over")
        ).withName("Smart BC-direct-AD");
        chooser.addOption("Smart BC-direct-AD", smartBCdirectAD);

        Command smartADEOTF =
            Commands.sequence(
                new PathPlannerAuto("Amp-side OTF + AD"),
                Commands.either(
                    new PathPlannerAuto("DE from A"),
                    new PathPlannerAuto("D-E-Shoot"),
                    s_Index::haveNote
                )
            ).withName("Smart ADE OTF");
        chooser.addOption("Smart ADE OTF", smartADEOTF);
    }

    public void teleopInit() {
        s_Vision.disableRotationTargetOverride();
    }

    private Pose2d getAmpPose() {
        // Get pose from Vision
        if (!s_Vision.haveAmpTarget()) {
            return s_Swerve.getPose();
        }
        Pose2d pose = s_Vision.lastPose();

        // Update pose in case we lose the amp target
        s_Swerve.setPose(pose);

        return pose;
    }

    private Pose2d getSourcePose() {
        // Get pose from Vision
        if (!s_Vision.haveSourceTarget()) {
            return s_Swerve.getPose();
        }
        Pose2d pose = s_Vision.lastPose();

        // Update pose in case we lose the source target
        s_Swerve.setPose(pose);

        return pose;
    }

    private Pose2d getSpeakerPose() {
        // Get pose from Vision
        if (!s_Vision.haveSpeakerTarget()) {
            return s_Swerve.getPose();
        }
        Pose2d pose = s_Vision.lastPose();

        // Update pose in case we lose the speaker target
        s_Swerve.setPose(pose);

        return pose;
    }

    private Command ampPathCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("To Amp");

        return Commands.sequence(
            Commands.runOnce(s_Vision::enableRotationAmpOverride),
            new FollowPathHolonomic(
                path,
                this::getAmpPose, // Robot pose supplier
                s_Swerve::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                s_Swerve::driveRobotRelativeAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(8.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(2.0, 0.0, 0.0), // Rotation PID constants
                    Constants.Swerve.maxSpeed, // Max module speed, in m/s
                    Constants.Swerve.driveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                Robot::isRed,
                s_Swerve // Reference to this subsystem to set requirements
            ),
            Commands.runOnce(s_Vision::disableRotationAmpOverride),
            Commands.runOnce(() -> { s_Shooter.setNextShot(Speed.AMP); }),
            new ShootCommand(s_Shooter, s_Index, false)
        ).handleInterrupt(s_Vision::disableRotationAmpOverride);
    }

    private Command sourcePathCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("To Source");

        return Commands.sequence(
            Commands.runOnce(s_Vision::enableRotationSourceOverride),
            new FollowPathHolonomic(
                path,
                this::getSourcePose, // Robot pose supplier
                s_Swerve::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                s_Swerve::driveRobotRelativeAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(8.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(2.0, 0.0, 0.0), // Rotation PID constants
                    Constants.Swerve.maxSpeed, // Max module speed, in m/s
                    Constants.Swerve.driveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                Robot::isRed,
                s_Swerve // Reference to this subsystem to set requirements
            ),
            Commands.runOnce(s_Vision::disableRotationSourceOverride)
        ).handleInterrupt(s_Vision::disableRotationSourceOverride);
    }


    private Command speakerPathCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("To Speaker");

        return Commands.sequence(
            Commands.runOnce(s_Vision::enableRotationTargetOverride),
            new FollowPathHolonomic(
                path,
                this::getSpeakerPose, // Robot pose supplier
                s_Swerve::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                s_Swerve::driveRobotRelativeAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(8.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(2.0, 0.0, 0.0), // Rotation PID constants
                    Constants.Swerve.maxSpeed, // Max module speed, in m/s
                    Constants.Swerve.driveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                Robot::isRed,
                s_Swerve // Reference to this subsystem to set requirements
            ),
            Commands.runOnce(s_Vision::disableRotationTargetOverride)
        ).handleInterrupt(s_Vision::disableRotationTargetOverride)
        .withName("Speaker align");
    }
}