package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ObjectDetection extends SubsystemBase {
  private final ObjectDetectionConsumer consumer;
  private final ObjectDetectionIO io;
  private final VisionDetectionIOInputsAutoLogged inputs = new VisionDetectionIOInputsAutoLogged();

  public ObjectDetection(ObjectDetectionConsumer consumer, ObjectDetectionIO io) {
    this.io = io;
    this.consumer = consumer;
  }

  public Translation2d getNotePositionRobotRelative() {

    double staticOffset = 0; // camera degrees

    double distInch = (1 / (40 - ((30) * inputs.iTY / 23)) * 1000); // Convert degrees to inch
    double noteYawAngleDegCorrected =
        -inputs.iTX - staticOffset; // account for static offset, reverse to be CCW+
    double radiusInchCorrected =
        distInch / Math.cos(Units.degreesToRadians(noteYawAngleDegCorrected));

    double noteYawAngleDegRaw = -inputs.iTX; // account for static offset, reverse to be CCW+
    // double radiusInchRaw = distInch / Math.cos(Units.degreesToRadians(noteYawAngleDegRaw));

    Logger.recordOutput("NoteTracking/distInch", distInch);
    Logger.recordOutput("NoteTracking/noteYawAngleDegCorrected", noteYawAngleDegCorrected);
    Logger.recordOutput("NoteTracking/noteYawAngleDegRaw", noteYawAngleDegRaw);
    Logger.recordOutput("NoteTracking/radiusCorrected", radiusInchCorrected);

    // camera relative -> bot relative -> field relative
    Translation2d camRelNoteLocT2dCorrected =
        new Translation2d(
            Units.inchesToMeters(radiusInchCorrected),
            Rotation2d.fromDegrees(noteYawAngleDegCorrected));
    Logger.recordOutput("NoteTracking/camRelNoteLocT2dCorrected", camRelNoteLocT2dCorrected);

    // Translation2d camRelNoteLocT2dRaw =
    //   new Translation2d(
    //     Units.inchesToMeters(radiusInchRaw), Rotation2d.fromDegrees(noteYawAngleDegRaw));

    // Translation2d roboRelNoteLocT2dRaw =
    //     camRelNoteLocT2dRaw
    //         .rotateBy(Rotation2d.fromDegrees(0))
    //         .plus(new Translation2d(Units.inchesToMeters(12), 0));

    Translation2d roboRelNoteLocT2dCorrected =
        camRelNoteLocT2dCorrected
            .rotateBy(Rotation2d.fromDegrees(0))
            .plus(new Translation2d(Units.inchesToMeters(12), 0));

    return roboRelNoteLocT2dCorrected;
  }

  public static Pose2d calculateNotePositionFieldRelative(
      Pose2d pickedRobotPoseMeters, Translation2d objectRobotRelativePoseMeters) {

    Translation2d fieldRelNoteLocT2dCorrected =
        objectRobotRelativePoseMeters
            .rotateBy(pickedRobotPoseMeters.getRotation())
            .plus(pickedRobotPoseMeters.getTranslation());

    // Translation2d fieldRelNoteLocT2dRaw =
    //     roboRelNoteLocT2dRaw
    //         .rotateBy(pickedRobotPose.getRotation())
    //         .plus(pickedRobotPose.getTranslation());

    // Logger.recordOutput("NoteTracking/fieldRelNoteLocT2dRaw", fieldRelNoteLocT2dRaw);
    Logger.recordOutput("NoteTracking/fieldRelNoteLocT2dCorrected", fieldRelNoteLocT2dCorrected);
    Logger.recordOutput(
        "distance from center of robot",
        Units.metersToInches(
            fieldRelNoteLocT2dCorrected.getDistance(pickedRobotPoseMeters.getTranslation())));

    // Translation3d fieldRelNoteLocT3d = new Translation3d(fieldRelNoteLocT2dCorrected.getX(),
    // fieldRelNoteLocT2dCorrected.getY(), 10);
    Pose3d fieldRelNoteLocP3d =
        new Pose3d(new Pose2d(fieldRelNoteLocT2dCorrected, new Rotation2d()));

    Logger.recordOutput("NoteTracking/fieldRelNoteLocP3d", fieldRelNoteLocP3d);
    return new Pose2d(fieldRelNoteLocT2dCorrected, new Rotation2d());
  }

  @FunctionalInterface
  public static interface ObjectDetectionConsumer {
    public void accept(Translation2d objectRobotRelativePoseMeters, double timeStampSeconds);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    consumer.accept(getNotePositionRobotRelative(), inputs.timestamp);
  }
  ;
}
