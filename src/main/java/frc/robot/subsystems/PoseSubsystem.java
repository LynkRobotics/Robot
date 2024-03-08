// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseSubsystem extends SubsystemBase {
    Swerve s_Swerve;
    VisionSubsystem s_Vision;

    public PoseSubsystem(Swerve s_Swerve, VisionSubsystem s_Vision) {
        this.s_Swerve = s_Swerve;
        this.s_Vision = s_Vision;
    }

    @Override
    public void periodic() {
    }
}