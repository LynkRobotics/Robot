package frc.robot;

import frc.lib.util.TunableOption;

public final class Options {
    /* Global Toggleable Options and their defaults */

    // Enable to switch in intaking through the shooter mechanism, instead of the normal intake
    public static final TunableOption optShooterIntake = new TunableOption("Shooter intake", false);

    // Enable to directly specify the RPM of the shooter motors (used during calibration, testing, or demos)
    public static final TunableOption optDirectRPM = new TunableOption("Direct set RPM", false);

    // Disable to aim into the speaking using the current pose, instead of directly referencing the current vision results
    public static final TunableOption optShootWithVision = new TunableOption("Shoot with Vision", true);

    // When enabled, the robot will automatically pick what to target to aim at based on the Pose
    public static final TunableOption optAutoTarget = new TunableOption("Auto-select target", true);

    // When enabled, the robot will automatically aim at the current target when it has a note
    public static final TunableOption optAimingEnabled = new TunableOption("Aiming enabled", true);

    // When enabled, the robot will brake at the end of teleOp; disable to coast the motors as teleOp ends
    public static final TunableOption optBrakeAfterTeleop = new TunableOption("Brake after teleOp", true);
}