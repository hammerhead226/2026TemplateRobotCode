package frc.robot.commands.drive;

import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.RotationUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PathfindToPose extends Command {
    private final Drive drive; 
    private final Pose2d targetPose;
    private final Command pathCommand;

private final PIDController xController = new PIDController(1.0, 0.0, 0.0);
private final PIDController yController = new PIDController(1.0, 0.0, 0.0);
private final PIDController rotationController = new PIDController(1.0, 0.0, 0.0);


    private boolean untilTrajectoryTimeoutCalled = false;

    public PathfindToPose(Drive drive, Pose2d targetPose, PathConstraints constraints) { 
        this.drive = drive;
        this.targetPose = targetPose;
        pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints);

        addRequirements(drive);
    }

    public PathfindToPose(Drive drive, Pose2d targetPose, Rotation2d goalAngle) { 
        this(drive, targetPose, SubsystemConstants.PathConstants.DEFAULT_PATH_CONSTRAINTS);
    }
        
    public void initialize() {
        PPHolonomicDriveController.overrideXFeedback(() -> {
            double currentX = drive.getPose().getX();
            double targetX = targetPose.getX();
            return xController.calculate(currentX, targetX);
        });
    
        PPHolonomicDriveController.overrideYFeedback(() -> {
            double currentY = drive.getPose().getY();
            double targetY = targetPose.getY();
            return yController.calculate(currentY, targetY);
        });
    
        PPHolonomicDriveController.overrideRotationFeedback(() -> {
            double currentHeading = drive.getPose().getRotation().getRadians();
            double targetHeading = targetPose.getRotation().getRadians();
            return rotationController.calculate(currentHeading, targetHeading);
        });
    
        pathCommand.initialize();
    }
    
    
    public void execute() {
        pathCommand.execute();
    }   

    public void end(boolean interrupted) {
        PPHolonomicDriveController.clearFeedbackOverrides();

        drive.stop();
    }

    @Override
    public boolean isFinished() {
        if (untilTrajectoryTimeoutCalled)
            if (pathCommand.isFinished())
                return true;
        
        return false;
    }

    
}
