// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HeadingLock extends Command {
  /** Creates a new HeadingLock. */
    private Drive drive;
    private DoubleSupplier omegaSupplier;
    private JoystickDrive jStick;
    private JoystickDriveAtAngle jStickAtAngle;

  public HeadingLock(Drive drive, Supplier<Translation2d> translationSupplier, DoubleSupplier omegaSupplier) {
    this.jStick = new JoystickDrive(drive, translationSupplier, omegaSupplier);
    this.jStickAtAngle = new JoystickDriveAtAngle(drive, translationSupplier, () -> Rotation2d.kCW_Pi_2);

    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (omegaSupplier.getAsDouble() > 0.1) {
      jStick.execute();
    } else {
      jStickAtAngle.execute();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
