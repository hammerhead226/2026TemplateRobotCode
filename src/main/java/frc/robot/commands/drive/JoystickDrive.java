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
  private Supplier<Translation2d> velocitySupplier;
  private DoubleSupplier omegaSupplier;

  public JoystickDrive(
      Drive drive, Supplier<Translation2d> velocitySupplier, DoubleSupplier omegaSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.velocitySupplier = velocitySupplier;
    this.omegaSupplier = omegaSupplier;
  }

  @Override
  public void execute() {
    // Convert to field relative speeds & send command

    Translation2d velocity = velocitySupplier.get();
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            velocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            velocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omegaSupplier.getAsDouble() * drive.getMaxAngularSpeedRadPerSec());
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, drive.getPose().getRotation().plus(FieldMirroring.driverStationFacing())));
  }
}
