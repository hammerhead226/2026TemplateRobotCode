package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.FieldConstants;

public class FieldMirroring {
  public static boolean shouldApply() {
    return DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue)
        .equals(DriverStation.Alliance.Red);
  }

  public static Rotation2d driverStationFacing() {
    return shouldApply() ? Rotation2d.k180deg : Rotation2d.kZero;
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldApply() ? Rotation2d.k180deg.minus(rotation) : rotation;
  }

  public static Translation2d apply(Translation2d translation2d) {
    return shouldApply()
        ? new Translation2d(FieldConstants.fieldLength - translation2d.getX(), translation2d.getY())
        : translation2d;
  }

  public static Pose2d apply(Pose2d pose2d) {
    return shouldApply()
        ? new Pose2d(apply(pose2d.getTranslation()), apply(pose2d.getRotation()))
        : pose2d;
  }
}
