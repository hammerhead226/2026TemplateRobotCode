package frc.robot.commands.drive;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.Fiducial;

// TODO: test
public class TrigSolveTag extends Command {
    private final Drive drive;
    private final Vision vision;
    private final int cameraIndex;
    private final int tagIndex;
    private final Transform2d robotToTargetIdeal;

    // TODO: make this suck less and find reasonable default values
    private static final double X_KP = 5.0;
    private static final double X_KD = 0.4;
    private static final double X_MAX_VELOCITY = 8.0;
    private static final double X_MAX_ACCELERATION = 20.0;

    private static final double Y_KP = 5.0;
    private static final double Y_KD = 0.4;
    private static final double Y_MAX_VELOCITY = 8.0;
    private static final double Y_MAX_ACCELERATION = 20.0;
    
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;

    private Transform3d robotToCamera;
    private Pose3d tagPose3d;

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController angleController;
    private double lastOkTx;
    private double lastOkTy;

    public TrigSolveTag(Drive drive, Vision vision, int cameraIndex, int tagIndex, Transform2d robotToTargetIdeal) {
        this.drive = drive;
        this.vision = vision;
        this.cameraIndex = cameraIndex;
        this.tagIndex = tagIndex;
        this.robotToTargetIdeal = robotToTargetIdeal;
        
        addRequirements(drive);

        robotToCamera = vision.getRobotToCamera(cameraIndex);
        tagPose3d = VisionConstants.aprilTagLayout.getTagPose(tagIndex).get();

        xController = new ProfiledPIDController(X_KP, 0.0, X_KD, new TrapezoidProfile.Constraints(X_MAX_VELOCITY, X_MAX_ACCELERATION));
        yController = new ProfiledPIDController(Y_KP, 0.0, Y_KD, new TrapezoidProfile.Constraints(Y_MAX_VELOCITY, Y_MAX_ACCELERATION));
        angleController = new ProfiledPIDController(ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        angleController.reset(0);
    }

    @Override
    public void execute() {
        // get tx and ty from camera
        Optional<Fiducial> fiducal = vision.getFiducial(cameraIndex, tagIndex);
        if (fiducal.isPresent()) {
            lastOkTx = fiducal.get().tx();
            lastOkTy = fiducal.get().ty();
        }
        double lastOkTxRadians = Math.toRadians(lastOkTx);
        double lastOkTyRadians = Math.toRadians(lastOkTy);
    
        // trig solve for the 2d distance and yaw to tag (on the ground plane)
        double groundToTargetRadians = robotToCamera.getRotation().getY()+lastOkTyRadians;
        double cameraToTargetMeters = (tagPose3d.getY()-robotToCamera.getY()) / Math.tan(groundToTargetRadians);
        Rotation2d projectedGroundAngle = new Rotation2d(
            Math.tan(lastOkTxRadians), 
            Math.cos(groundToTargetRadians)
        );

        // find the error between the current robotToTarget and desired robotToTarget
        Transform2d cameraToTarget = new Transform2d(
            new Translation2d(cameraToTargetMeters,projectedGroundAngle),
            projectedGroundAngle
        );
        Transform2d robotToCamera2d = new Transform2d(
            robotToCamera.getTranslation().toTranslation2d(), 
            robotToCamera.getRotation().toRotation2d()
        );
        Transform2d robotToTargetEstimate = cameraToTarget.plus(robotToCamera2d.inverse());
        Transform2d error = robotToTargetEstimate.plus(robotToTargetIdeal.inverse());

        ChassisSpeeds speeds = new ChassisSpeeds(
            xController.calculate(error.getX()),
            yController.calculate(error.getY()),
            angleController.calculate(error.getRotation().getRadians())
        );
        drive.runVelocity(speeds);
    }
}
