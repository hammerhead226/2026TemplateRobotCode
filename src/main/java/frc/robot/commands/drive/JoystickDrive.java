package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldMirroring;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class JoystickDrive extends Command {
  private Drive drive;
  private Supplier<Translation2d> translationSupplier;
  private DoubleSupplier omegaSupplier;

  public JoystickDrive(
      Drive drive, Supplier<Translation2d> translationSupplier, DoubleSupplier omegaSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.translationSupplier = translationSupplier;
    this.omegaSupplier = omegaSupplier;
  }

  @Override
  public void execute() {
    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            translationSupplier.get().getX() * drive.getMaxLinearSpeedMetersPerSec(),
            translationSupplier.get().getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omegaSupplier.getAsDouble() * drive.getMaxAngularSpeedRadPerSec());
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, drive.getPose().getRotation().plus(FieldMirroring.driverStationFacing())));
  }
}
