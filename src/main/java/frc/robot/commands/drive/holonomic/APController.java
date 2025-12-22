package frc.robot.commands.drive.holonomic;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class APController implements DriveController {
    private static final APConstraints kConstraints =
            new APConstraints().withAcceleration(5.0).withJerk(2.0);

    private static final APProfile kProfile = new APProfile(kConstraints)
            .withErrorXY(Centimeters.of(2))
            .withErrorTheta(Degrees.of(0.5))
            .withBeelineRadius(Centimeters.of(8));

    private static final Autopilot kAutopilot = new Autopilot(kProfile);

    private static final double EPSILON = 1e-6;

    private final Drive drive;
    private final APTarget target;
    private final double transistionRadius = 3;
    private final PIDPoseController rotationController;
    private Rotation2d targetAngle = Rotation2d.kZero;

    public APController(Drive drive, APTarget target) {
        this.target = target;
        this.drive = drive;
        this.rotationController = new PIDPoseController(drive, drive::getPose, () -> new Pose2d(0, 0, targetAngle));
    }

    @Override
    public void reset() {
        APResult out = kAutopilot.calculate(drive.getPose(), drive.getChassisSpeeds(), target);
        targetAngle = out.targetAngle();
        rotationController.reset();
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        if (drive.getPose().getTranslation().getDistance(target.getReference().getTranslation())
                > transistionRadius + EPSILON) {
            Translation2d transitionPoint =
                    getTransitionPoint(drive.getPose().getTranslation(), target, transistionRadius);
            Translation2d speedsVector = new Translation2d(
                    drive.getMaxLinearSpeedMetersPerSec(),
                    transitionPoint.minus(drive.getPose().getTranslation()).getAngle());
            ChassisSpeeds speeds = new ChassisSpeeds(speedsVector.getX(), speedsVector.getY(), 0);
            return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation());
        }

        APResult out = kAutopilot.calculate(drive.getPose(), drive.getChassisSpeeds(), target);
        targetAngle = out.targetAngle();
        Logger.recordOutput("APvx", out.vx());
        Logger.recordOutput("APvy", out.vy());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                        out.vx(),
                        out.vy(),
                        AngularVelocity.ofBaseUnits(
                                rotationController.getSpeeds().omegaRadiansPerSecond, RadiansPerSecond)),
                drive.getRotation());
    }

    private Translation2d getTransitionPoint(Translation2d robotPosition, APTarget target, double transistionRadius) {
        // inputs within the autopilot radius are invalid
        if (robotPosition.getDistance(target.getReference().getTranslation()) < transistionRadius - EPSILON) {
            return Translation2d.kZero;
        }

        Transform2d targetToField = new Transform2d(
                target.getReference().getTranslation(), target.getEntryAngle().orElse(Rotation2d.kZero));
        Translation2d robotTargetSpace = new Pose2d(robotPosition, Rotation2d.kZero)
                .transformBy(targetToField.inverse())
                .getTranslation();
        Translation2d translationPointTargetSpace;
        if (robotTargetSpace.getY() < 0) {
            robotTargetSpace = new Translation2d(robotTargetSpace.getX(), -robotTargetSpace.getY());
            translationPointTargetSpace = getTransitionPointTargetSpace(robotTargetSpace, transistionRadius);
            translationPointTargetSpace =
                    new Translation2d(translationPointTargetSpace.getX(), -translationPointTargetSpace.getY());
        } else {
            robotTargetSpace = new Translation2d(robotTargetSpace.getX(), robotTargetSpace.getY());
            translationPointTargetSpace = getTransitionPointTargetSpace(robotTargetSpace, transistionRadius);
        }
        return translationPointTargetSpace.plus(targetToField.getTranslation());
    }

    // theta at which the cos(theta) = theta * sin(theta) and the path to the transition point is vertical
    // calculuted via wolfram alpha
    private static final double THETA_VERTICAL = 0.860333589019380;

    private Translation2d getTransitionPointTargetSpace(Translation2d outerPoint, double radius) {
        // inputs with negative y are invalid
        if (outerPoint.getY() <= 0) {
            return Translation2d.kZero;
        }

        // inputs within the autopilot radius are invalid
        if (outerPoint.getNorm() <= radius - EPSILON) {
            return Translation2d.kZero;
        }

        Translation2d verticalSlopePoint = new Translation2d(radius, new Rotation2d(THETA_VERTICAL));
        if (outerPoint.getX() >= verticalSlopePoint.getX() + EPSILON) {
            // binary search for solution between f(0) <= 0 <= f(thetaVertical - EPSILON)
            // f(outerPoint radians) >= 0 until it flips at outerPoint raidans = thetaVertical
            double transitionTheta = binarySearchTransitionTheta(
                    outerPoint, radius, 0, Math.min(outerPoint.getAngle().getRadians(), THETA_VERTICAL - EPSILON));
            return new Translation2d(radius, transitionTheta);
        } else if (outerPoint.getX() > verticalSlopePoint.getX() - EPSILON) {
            return verticalSlopePoint;
        }

        // binary search for solution between f(pi) <= 0 <= f(thetaVertical + EPSILON)
        // f(outerPoint radians) <= 0 until it flips at outerPoint raidans ~ 3.4
        double transitionTheta = binarySearchTransitionTheta(
                outerPoint, radius, outerPoint.getAngle().getRadians(), THETA_VERTICAL + EPSILON);
        return new Translation2d(radius, transitionTheta);
    }

    private double binarySearchTransitionTheta(
            Translation2d outerPoint, double radius, double lowerTheta, double upperTheta) {
        if (signDistance(outerPoint, radius, upperTheta) <= 0) return upperTheta;
        if (signDistance(outerPoint, radius, lowerTheta) >= 0) return lowerTheta;
        double estimatedTheta = (lowerTheta + upperTheta) / 2.0;
        for (int i = 0; i < Math.ceil(Math.log(EPSILON) / Math.log(0.5)); i++) { // 20 iterations for EPSILON = 1e-6
            if (signDistance(outerPoint, radius, estimatedTheta) >= 0) {
                upperTheta = estimatedTheta;
            } else {
                lowerTheta = estimatedTheta;
            }
            estimatedTheta = (lowerTheta + upperTheta) / 2.0;
        }
        return estimatedTheta;
    }

    private double signDistance(Translation2d outerPoint, double radius, double theta) {
        Translation2d transitionPoint = new Translation2d(radius, new Rotation2d(theta));
        return getSlopeOfREqualsTheta(theta) * (outerPoint.getX() - transitionPoint.getX())
                - outerPoint.getY()
                + transitionPoint.getY();
    }

    private double getSlopeOfREqualsTheta(double theta) {
        return (Math.cos(theta) - theta * Math.sin(theta)) / (theta * Math.cos(theta) + Math.sin(theta));
    }
}
