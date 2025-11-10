// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arms.Arm;

public class SetArmAngleTemplate extends Command {
  /** Creates a new setArmAngle. */
  private final Arm arm;

  private final double setpointDegs;
  private final double thresholdDegs;

  public SetArmAngleTemplate(Arm arm, double setpointDegs, double thresholdDegs) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.setpointDegs = setpointDegs;
    this.thresholdDegs = thresholdDegs;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmGoal(setpointDegs);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(arm.getArmPositionDegs() - setpointDegs) <= thresholdDegs;
  }
}
