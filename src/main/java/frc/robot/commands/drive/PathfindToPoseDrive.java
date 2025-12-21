package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.holonomic.DriveController;
import frc.robot.subsystems.drive.Drive;

public class PathfindToPoseDrive extends Command {
    private final Drive drive;
    private final Pose2d targetPose;
    private final PathConstraints constraints;

    private Command pathCommand;

    private boolean withPathTimeout;
    private DriveController overrideController;
    private double linearThreshold;
    private double angularThreshold;

    public PathfindToPoseDrive(Drive drive, Pose2d targetPose, PathConstraints constraints) {
        addRequirements(drive);
        this.drive = drive;
        this.targetPose = targetPose;
        this.constraints = constraints;
    }

    /*
     * custom decorator
     * due to type safety, must be used before WPIlib's Command decorators
     * since Commands cannot be readily coverted back to this class's type
     */
    public PathfindToPoseDrive withPathTimeout() {
        this.withPathTimeout = true;
        return this;
    }

    public PathfindToPoseDrive withOverrides(
            DriveController overrideController, double linearThreshold, double angularThreshold) {
        this.overrideController = overrideController;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        return this;
    }

    @Override
    public void initialize() {
        pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
        pathCommand.initialize();
        if (overrideController != null) overrideController.reset();
    }

    @Override
    public void execute() {
        PPHolonomicDriveController.clearFeedbackOverrides();
        if (overrideController != null) {
            ChassisSpeeds overrideSpeeds =
                    ChassisSpeeds.fromRobotRelativeSpeeds(overrideController.getSpeeds(), drive.getRotation());
            if (Math.hypot(overrideSpeeds.vxMetersPerSecond, overrideSpeeds.vyMetersPerSecond) >= linearThreshold) {
                PPHolonomicDriveController.overrideXFeedback(() -> overrideSpeeds.vxMetersPerSecond);
                PPHolonomicDriveController.overrideYFeedback(() -> overrideSpeeds.vyMetersPerSecond);
            }
            if (overrideSpeeds.omegaRadiansPerSecond >= angularThreshold) {
                PPHolonomicDriveController.overrideRotationFeedback(() -> overrideSpeeds.omegaRadiansPerSecond);
            }
        }
        pathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        if (withPathTimeout) if (pathCommand.isFinished()) return true;

        return false;
    }
}
