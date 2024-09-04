package frc.robot;

import frc.lib.util.TunableOption;

public final class Options {
    /* Toggleable Options and their defaults */
    public static final TunableOption optShooterIntake = new TunableOption("Shooter intake", false);
    public static final TunableOption optDirectRPM = new TunableOption("Direct set RPM", false);
    public static final TunableOption optShootWithVision = new TunableOption("Shoot with Vision", true);
    public static final TunableOption optAimingEnabled = new TunableOption("Aiming enabled", true);
    public static final TunableOption optFullFieldAiming = new TunableOption("Full field aiming", true);
    // public static final TunableOption optVisionPoseInAuto = new TunableOption("Use Vision Pose in Auto", false);
    // public static final TunableOption optVisionPoseInTeleop = new TunableOption("Disable Vision Pose in Teleop", true);
    public static final TunableOption optSetPoseWhenShooting = new TunableOption("Set pose when shooting", true);
    public static final TunableOption optUpdateVisionDashboard = new TunableOption("Update vision dashboard", false);
    public static final TunableOption optUpdatePoseWithVisionAuto = new TunableOption("Update pose with vision in Auto", false);
    public static final TunableOption optUpdatePoseWithVisionTeleop = new TunableOption("Update pose with vision in Teleop", true);
    public static final TunableOption optLeftIndexSensorEnabled = new TunableOption("Left index sensor enabled", false);
    public static final TunableOption optRightIndexSensorEnabled = new TunableOption("Right index sensor enabled", true);
    public static final TunableOption optClimbersEnabled = new TunableOption("Climbers enabled", true);
}