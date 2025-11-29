package frc.robot.commands.drive.holonomic;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.Fiducial;
import java.util.Optional;

// TODO: test
public class ServoingController {
    private final Drive drive;
    private final Vision vision;
    private final int cameraIndex;
    private final int tagIndex;

    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;

    private ProfiledPIDController angleController;
    private double lastOkTx;

    public ServoingController(Drive drive, Vision vision, int cameraIndex, int tagIndex) {
        this.drive = drive;
        this.vision = vision;
        this.cameraIndex = cameraIndex;
        this.tagIndex = tagIndex;

        angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset() {
        double robotToTargetRadians = robotToTargetRadians();
        angleController.reset(robotToTargetRadians, drive.getChassisSpeeds().omegaRadiansPerSecond);
    }

    public ChassisSpeeds getSpeeds() {
        double robotToTargetRadians = robotToTargetRadians();
        double omega = angleController.calculate(robotToTargetRadians);

        return new ChassisSpeeds(0, 0, omega * drive.getMaxAngularSpeedRadPerSec());
    }

    private double robotToTargetRadians() {
        Optional<Fiducial> fiducal = vision.getFiducial(cameraIndex, tagIndex);
        if (fiducal.isPresent()) lastOkTx = fiducal.get().tx();
        return Math.toRadians(lastOkTx);
    }
}
