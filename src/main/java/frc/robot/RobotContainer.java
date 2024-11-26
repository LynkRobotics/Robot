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

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import static frc.robot.Options.*;
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
    @SuppressWarnings ("unused")
    private final PoseSubsystem s_Pose = new PoseSubsystem(s_Swerve, s_Vision);

    private final SendableChooser<Command> autoChooser;

    private static void autoNamedCommand(String name, Command command) {
        NamedCommands.registerCommand(name, command.withName(name + " (auto)"));
    }

    private Command setShotCommand(Speed speed) {
        return Commands.runOnce(() -> { s_Shooter.setNextShot(speed); }).withName("Set " + (speed == null ? "default" : speed) + " shot");
    }

    private Command fixedShotCommand(Speed speed) {
        return setShotCommand(speed)
            .andThen(
                new ShootCommand(s_Shooter, s_Index, false)
                .raceWith(Commands.waitSeconds(1.5)))
            .withName("Fixed " + speed + " shot");
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
    // Set the scheduler to log when a command initializes, interrupts, or finishes
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(command -> DogLog.log("Misc/Robot Status", "Initialized: " + command.getName()));
        scheduler.onCommandInterrupt(command -> DogLog.log("Misc/Robot Status", "Interrupted: " + command.getName()));
        scheduler.onCommandFinish(command -> DogLog.log("Misc/Robot Status", "Finished: " + command.getName()));

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
        SmartDashboard.putData(new ShootCommand(s_Shooter, s_Index, s_Swerve).withTimeout(3.0).withName("Shoot Commmand"));

        // Default named commands for PathPlanner
        SmartDashboard.putNumber("auto/Startup delay", 0.0);
        autoNamedCommand("Done", new PrintCommand("Done"));
        autoNamedCommand("Start", new PrintCommand("Starting"));
        autoNamedCommand("Startup delay", new DeferredCommand(() ->Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()));
        autoNamedCommand("Shoot",
            new ShootCommand(s_Shooter, s_Index, s_Swerve)
                .raceWith(new AimCommand(s_Swerve, s_Vision))
                .raceWith(Commands.waitSeconds(2.50)));
        autoNamedCommand("Shoot without aiming",
            new ShootCommand(s_Shooter, s_Index, s_Swerve, false)
                .raceWith(Commands.waitSeconds(1.50))
                .withName("Shoot w/o aiming (Auto)"));
        autoNamedCommand("Fixed SW shot", fixedShotCommand(Speed.SUBWOOFER));
        autoNamedCommand("Fixed AS shot", fixedShotCommand(Speed.AMPSIDE));
        autoNamedCommand("Shoot OTF", fixedShotCommand(Speed.OTF));
        autoNamedCommand("Amp-side OTF Shot", fixedShotCommand(Speed.AMPSIDEOTF));
        autoNamedCommand("Source-side OTF Shot", fixedShotCommand(Speed.SOURCESIDEOTF));
        autoNamedCommand("Intake note", new IntakeCommand(s_Intake, s_Index, driver.getHID()));
        autoNamedCommand("Amp Shot", fixedShotCommand(Speed.AMP));
        autoNamedCommand("Bloop Shot", fixedShotCommand(Speed.BLOOP));
        autoNamedCommand("Slide Shot", fixedShotCommand(Speed.SLIDE));
        autoNamedCommand("Short Slide Shot", fixedShotCommand(Speed.SHORTSLIDE));
        autoNamedCommand("Special Shot", fixedShotCommand(Speed.SPECIAL));
        autoNamedCommand("Override rotation", Commands.runOnce(s_Vision::enableRotationTargetOverride));
        autoNamedCommand("Restore rotation", Commands.runOnce(s_Vision::disableRotationTargetOverride));
        autoNamedCommand("Stop", Commands.runOnce(s_Swerve::stopSwerve));
        autoNamedCommand("Set Instant Pose", Commands.runOnce(() ->
            {
                if (s_Vision.haveSpeakerTarget()) {
                    Pose2d pose = s_Vision.lastPose();
                    s_Pose.setPose(pose);
                    DogLog.log("Auto/Status", "Pose updated from vision: " + PoseSubsystem.prettyPose(pose));
                } else {
                    DogLog.log("Auto/Status", "Refusing to update pose from vision without a current speaker target");
                }
            } ));
        autoNamedCommand("Coast after auto", new CoastAfterAuto(s_Swerve));
        autoNamedCommand("Coast drive motors", Commands.runOnce(s_Swerve::setDriveMotorsToCoast));

        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("auto/Auto Chooser", autoChooser);
        buildAutos(autoChooser);

        // During calibration allow for direct control
        // SmartDashboard.putNumber("Shooter voltage direct", 0.0);
        // SmartDashboard.putData(s_Shooter.runOnce(() -> { s_Shooter.setVoltage(SmartDashboard.getNumber("Shooter voltage direct", 0)); }).withName("Set shooter voltage"));
        // SmartDashboard.putData(s_Shooter.runOnce(() -> { s_Shooter.setVoltage(0); }).withName("Stop shooter"));

        // Allow for direct RPM setting
        SmartDashboard.putNumber("Shooter top RPM", 1000.0);
        SmartDashboard.putNumber("Shooter bottom RPM", 1000.0);
        SmartDashboard.putData(s_Shooter.runOnce(() -> { s_Shooter.setRPM(500); }).withName("Idle shooter"));
        SmartDashboard.putData(Commands.runOnce(s_Pose::zeroGyro, s_Swerve).withName("Zero Gyro"));
        SmartDashboard.putData(Commands.runOnce(s_Pose::resetHeading, s_Swerve).withName("Reset heading"));

        // Allow for direct climber control
        SmartDashboard.putData(Commands.runOnce(() -> { s_LeftClimber.stop(); s_RightClimber.stop(); }, s_LeftClimber, s_RightClimber).withName("Stop climbers"));
        SmartDashboard.putData(s_LeftClimber.runOnce(() -> { s_LeftClimber.applyVoltage(Constants.Climber.slowVoltage); }).withName("Left down slow"));
        SmartDashboard.putData(s_RightClimber.runOnce(() -> { s_RightClimber.applyVoltage(Constants.Climber.slowVoltage); }).withName("Right down slow"));

        SmartDashboard.putNumber("Left climber voltage", 0.0);
        SmartDashboard.putNumber("Right climber voltage", 0.0);
        SmartDashboard.putData(Commands.runOnce(() -> { s_LeftClimber.applyVoltage(SmartDashboard.getNumber("Left climber voltage", 0.0)); s_RightClimber.applyVoltage(SmartDashboard.getNumber("Right climber voltage", 0.0));}, s_LeftClimber, s_RightClimber).withName("Set climber voltage"));
        SmartDashboard.putData(Commands.runOnce(() -> { s_LeftClimber.zero(); s_RightClimber.zero(); }, s_LeftClimber, s_RightClimber).withName("Zero climbers"));

        SmartDashboard.putNumber("Left climber target position", 0.0);
        SmartDashboard.putData(new ClimberPositionCommand(SmartDashboard.getNumber("Left climber target position", 0.0), LEDSubsystem.TempState.RETRACTING, s_LeftClimber).withName("Set left climber position"));
        SmartDashboard.putNumber("Right climber target position", 0.0);
        SmartDashboard.putData(new ClimberPositionCommand(SmartDashboard.getNumber("Right climber target position", 0.0), LEDSubsystem.TempState.RETRACTING, s_RightClimber).withName("Set right climber position"));

        SmartDashboard.putData(s_Swerve.runOnce(s_Swerve::setMotorsToCoast).ignoringDisable(true).withName("autoSetup/SetSwerveCoast"));
        SmartDashboard.putData(s_Swerve.runOnce(s_Swerve::setMotorsToBrake).ignoringDisable(true).withName("autoSetup/SetSwerveBrake"));
        SmartDashboard.putData(s_Swerve.run(s_Swerve::alignStraight).ignoringDisable(true).withName("autoSetup/SetSwerveAligned"));

        DogLog.setOptions(new DogLogOptions(
            Constants.atHQ, //Whether logged values should be published to NetworkTables
            false, //Whether all NetworkTables fields should be saved to the log file.
            true, //Whether driver station data (robot enable state and joystick inputs) should be saved to the log file.
            true, //Whether to log extra data, like PDH currents, CAN usage, etc.
            1000 //The size of the log message queue to use
        ));

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
        intakeButton.whileTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::enableSpeedLimit).withName("Enable speed limit"),
                Commands.either(
                    new ShooterIntakeCommand(s_Shooter, s_Index, driver.getHID()),
                    new IntakeCommand(s_Intake, s_Index, driver.getHID()),
                    optShooterIntake),
                Commands.runOnce(s_Swerve::disableSpeedLimit).withName("Disable speed limit"))
            .handleInterrupt(s_Swerve::disableSpeedLimit)
            .withName("Intake"));
        shooterButton.whileTrue(
            Commands.either(
                new ShootCommand(s_Shooter, s_Index,
                    () -> SmartDashboard.getNumber("Shooter top RPM", 0.0),
                    () -> SmartDashboard.getNumber("Shooter bottom RPM", 0.0)),
                new ShootCommand(s_Shooter, s_Index, s_Swerve),
                optDirectRPM)
            .withName("Shoot"));
        climberExtendButton.onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::enableSpeedLimit).withName("Enable speed limit"),
                Commands.parallel(
                    new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_LeftClimber).withName("Extend left climber"),
                    new ClimberPositionCommand(Constants.Climber.extendedPosition, LEDSubsystem.TempState.EXTENDING, s_RightClimber).withName("Extend right climber")))
            .withName("Extend Climbers"));
        SmartDashboard.putData(Commands.runOnce(s_Swerve::disableSpeedLimit).withName("Disable speed limit"));

        leftClimberButton.whileTrue(new ClimberPositionCommand(Constants.Climber.retractedPosition, LEDSubsystem.TempState.RETRACTING, s_LeftClimber).withName("Retract left climber"));
        rightClimberButton.whileTrue(new ClimberPositionCommand(Constants.Climber.retractedPosition, LEDSubsystem.TempState.RETRACTING, s_RightClimber).withName("Retract right climber"));

        /* Buttons to set the next shot */
        ampButton.onTrue(Commands.runOnce(s_Shooter::toggleAmp).withName("Toggle amp shot"));
        defaultShotButton.onTrue(setShotCommand(null));
        dumpShotButton.onTrue(setShotCommand(Speed.DUMP));
        slideShotButton.onTrue(setShotCommand(Speed.SLIDE));

        ejectButton.whileTrue(new EjectCommand(s_Intake, s_Index, s_Shooter));

        ampShotButton.whileTrue(ampPathCommand());
        sourceAlignButton.whileTrue(sourcePathCommand());
        SmartDashboard.putData(pathCommand("To Speaker").withName("Speaker align"));
        SmartDashboard.putData(pathCommand("To Speaker-AmpSide").withName("Speaker Amp-Side align"));
        SmartDashboard.putData(pathCommand("To Speaker-SourceSide").withName("Speaker Source-Side align"));

        SmartDashboard.putData(Commands.runOnce(() -> { PoseSubsystem.setTargetAngle(new Rotation2d()); }).withName("pose/Align to zero"));
        SmartDashboard.putData(Commands.runOnce(() -> { PoseSubsystem.setTargetAngle(new Rotation2d(Math.PI / 2.0)); }).withName("pose/Align to 90"));
        SmartDashboard.putData(Commands.runOnce(() -> { PoseSubsystem.setTargetAngle(null); }).withName("pose/Clear target angle"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void addAutoCommand(SendableChooser<Command> chooser, Command command) {
        chooser.addOption(command.getName(), command);
    }

    private void buildAutos(SendableChooser<Command> chooser) {
        addAutoCommand(chooser,
            Commands.sequence(
                new PathPlannerAuto("SS Angled Start to H"),
                Commands.either(
                    new PathPlannerAuto("H-Shoot-G-Shoot"),
                    new PathPlannerAuto("H-G-Shoot"),
                    s_Index::haveNote
                )
            ).withName("Smart HG"));

        // addAutoCommand(chooser,
        //     Commands.sequence(
        //         new PathPlannerAuto("Source-side OTF to H"),
        //         Commands.either(
        //             new PathPlannerAuto("H-Shoot-G-Shoot"),
        //             new PathPlannerAuto("H-G-Shoot"),
        //             s_Index::haveNote
        //         )
        //     ).withName("Smart OTF HG"));

        addAutoCommand(chooser,
            Commands.sequence(
                new PathPlannerAuto("AS Angled + AD"),
                Commands.either(
                    new PathPlannerAuto("DE from close"),
                    new PathPlannerAuto("D-E-Shoot"),
                    s_Index::haveNote
                )
            ).withName("Smart ADE from Close"));

        addAutoCommand(chooser,
            Commands.sequence(
                new PathPlannerAuto("AS Angled + AD"),
                Commands.either(
                    new PathPlannerAuto("DE from A"),
                    new PathPlannerAuto("D-E-Shoot"),
                    s_Index::haveNote
                )
            ).withName("Smart ADE"));

        addAutoCommand(chooser,
            Commands.sequence(
                new PathPlannerAuto("BCAD start"),
                Commands.either(
                    new PathPlannerAuto("DE from A"),
                    new PathPlannerAuto("D-E-Shoot"),
                    s_Index::haveNote
                )
            ).withName("Smart BCAD"));

        addAutoCommand(chooser,
            Commands.sequence(
                new PathPlannerAuto("BC-direct-AD start"),
                Commands.either(
                    new PathPlannerAuto("DE from A"),
                    new PathPlannerAuto("D-E-Shoot"),
                    s_Index::haveNote
                ),
                Commands.runOnce(() -> { DogLog.log("Auto/Status", "Conditional part over");})
            ).withName("Smart BC-direct-AD"));

        addAutoCommand(chooser,
            Commands.sequence(
                new PathPlannerAuto("Amp-side OTF + AD"),
                Commands.either(
                    new PathPlannerAuto("DE from A"),
                    new PathPlannerAuto("D-E-Shoot"),
                    s_Index::haveNote
                )
            ).withName("Smart ADE OTF"));

        addAutoCommand(chooser, choreoTestCommand());
    }

    public void teleopInit() {
        s_Swerve.stopSwerve();
        s_Vision.disableRotationTargetOverride();
        s_Swerve.setDriveMotorsToCoast();
    }

    public void teleopExit() {
        if (optBrakeAfterTeleop.get()) {
            s_Swerve.setDriveMotorsToBrake();
        }
    }

    private Command ampPathCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("To Amp");

        return Commands.sequence(
            Commands.runOnce(s_Vision::enableRotationAmpOverride).withName("Enable rotation amp override"),
            new FollowPathHolonomic(
                path,
                s_Pose::getPose, // Robot pose supplier
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
            ).withName("Follow path to amp"),
            Commands.runOnce(s_Vision::disableRotationAmpOverride).withName("Disable rotation amp override"),
            fixedShotCommand(Speed.AMP)
        ).handleInterrupt(s_Vision::disableRotationAmpOverride)
        .withName("Amp path & shoot");
    }

    private Command sourcePathCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("To Source");

        return Commands.sequence(
            Commands.runOnce(s_Vision::enableRotationSourceOverride).withName("Enable rotation source override"),
            new FollowPathHolonomic(
                path,
                s_Pose::getPose, // Robot pose supplier
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
            ).withName("Follow path to source"),
            Commands.runOnce(s_Vision::disableRotationSourceOverride).withName("Disable rotation source override")
        ).handleInterrupt(s_Vision::disableRotationSourceOverride)
        .withName("Source align");
    }

    private Command pathCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return Commands.sequence(
            Commands.runOnce(s_Vision::enableRotationTargetOverride).withName("Enable rotation target override"),
            new FollowPathHolonomic(
                path,
                s_Pose::getPose, // Robot pose supplier
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
            ).withName("Follow path: " + pathName),
            Commands.runOnce(s_Vision::disableRotationTargetOverride).withName("Disable rotation target override")
        ).handleInterrupt(s_Vision::disableRotationTargetOverride)
        .withName("Path: " + pathName);
    }

    private Command choreoTestCommand() {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("Choreo-Straight");

        return Commands.sequence(
            new FollowPathHolonomic(
                path,
                s_Pose::getPose, // Robot pose supplier
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
            ).withName("Follow choreo path")
        ).handleInterrupt(s_Vision::disableRotationSourceOverride)
        .withName("Choreo Test");
    }
}