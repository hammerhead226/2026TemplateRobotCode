package frc.robot.commands.drive;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.Fiducial;

public class ServoToTag extends Command {
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

    public ServoToTag(Drive drive, Vision vision, int cameraIndex, int tagIndex) {
        this.drive = drive;
        this.vision = vision;
        this.cameraIndex = cameraIndex;
        this.tagIndex = tagIndex;
        
        addRequirements(drive);

        angleController = new ProfiledPIDController(
            ANGLE_KP, 
            0.0, 
            ANGLE_KD, 
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        angleController.reset(0);
    }

    @Override
    public void execute() {
        Optional<Fiducial> fiducal = vision.getFiducial(cameraIndex, tagIndex);
        if (fiducal.isPresent()) lastOkTx = fiducal.get().tx();
        double lastOkTxRadians = Math.toRadians(lastOkTx);

        double omega = angleController.calculate(lastOkTxRadians);

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds = new ChassisSpeeds(0,0,omega);
        drive.runVelocity(speeds);
    }
}
