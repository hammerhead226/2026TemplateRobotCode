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

    private final APTarget target;
    private final Drive drive;
    private final PIDPoseController rotationController;
    private Rotation2d targetAngle = Rotation2d.kZero;
 
    //TODO We should add entry angle as an input, that's the main appeal of AutoPilot https://therekrab.github.io/autopilot/technical.html#a-quick-note-about-entry-angle
    public APController(APTarget target, Drive drive) {
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
        APResult out = kAutopilot.calculate(drive.getPose(), drive.getChassisSpeeds(), target);
        targetAngle = out.targetAngle();
        Logger.recordOutput("APvx", out.vx());
        Logger.recordOutput("APvy", out.vy());
        //TODO the speeds from out.vx are field relative, not robot relative https://therekrab.github.io/autopilot/examples.html
        return new ChassisSpeeds(
                out.vx(),
                out.vy(),
                AngularVelocity.ofBaseUnits(rotationController.getSpeeds().omegaRadiansPerSecond, RadiansPerSecond));
    }
}
