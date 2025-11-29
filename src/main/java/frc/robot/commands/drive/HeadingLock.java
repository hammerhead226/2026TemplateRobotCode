// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.holonomic.HolonomicDrive;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HeadingLock extends Command {
  /** Creates a new HeadingLock. */
    private Drive drive;
    private final Supplier<Translation2d> translationSupplier;
    private DoubleSupplier omegaSupplier;

    private HolonomicDrive jStickDrive;
    private HolonomicDrive headingHoldDrive;

    private Rotation2d targetAngle;


  public HeadingLock(Drive drive, Supplier<Translation2d> translationSupplier, DoubleSupplier omegaSupplier) {
    this.drive = drive;
    this.translationSupplier = translationSupplier;
    this.omegaSupplier = omegaSupplier;


    addRequirements(drive);
  }

  @Override
  public void initialize() {
    targetAngle = drive.getPose().getRotation();

    jStickDrive = new HolonomicDrive(drive,
     () -> {
            Translation2d driver = translationSupplier.get();
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    driver.getX(), driver.getY(), omegaSupplier.getAsDouble(), drive.getRotation());
        });

    headingHoldDrive = new HolonomicDrive(drive,
    () -> {
      Translation2d driver = translationSupplier.get();
      return ChassisSpeeds.fromFieldRelativeSpeeds(
              driver.getX(),
              driver.getY(),
              0.0, 
              targetAngle 
    );
  });   
    


  }

  @Override
  public void execute() {
    if (omegaSupplier.getAsDouble() > 0.1) {
      jStickDrive.execute();
    } else {
      headingHoldDrive.execute();
    }
    double omega = omegaSupplier.getAsDouble();
    if (omega >0.1 || omega < -0.1) {
      targetAngle = drive.getPose().getRotation();
      jStickDrive.execute();
    } else {
      headingHoldDrive.execute();
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
