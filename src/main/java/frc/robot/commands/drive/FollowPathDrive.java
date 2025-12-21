package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.holonomic.DriveController;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class FollowPathDrive extends Command {
    private final Drive drive;
    private final Supplier<PathPlannerPath> pathSupplier;
    private Command pathCommand;

    private boolean withPathTimeout;
    private boolean withFlipping;
    private DriveController overrideController;
    private double linearThreshold;
    private double angularThreshold;

    public FollowPathDrive(Drive drive, Supplier<PathPlannerPath> pathSupplier) {
        addRequirements(drive);
        this.drive = drive;
        this.pathSupplier = pathSupplier;
    }

    /*
     * custom decorator
     * due to type safety, must be used before WPIlib's Command decorators
     * since Commands cannot be readily coverted back to this class's type
     */
    public FollowPathDrive withPathTimeout() {
        withPathTimeout = true;
        return this;
    }

    public FollowPathDrive withFlipping() {
        withFlipping = true;
        return this;
    }

    public FollowPathDrive withOverrides(
            DriveController overrideController, double linearThreshold, double angularThreshold) {
        this.overrideController = overrideController;
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        return this;
    }

    @Override
    public void initialize() {
        PathPlannerPath path = pathSupplier.get();
        // force the path to not be flipped
        if (AutoBuilder.shouldFlip() && !withFlipping) {
            path = path.flipPath();
        }
        pathCommand = AutoBuilder.followPath(path);
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
    public boolean isFinished() {
        if (withPathTimeout) return pathCommand.isFinished();
        return false;
    }
}
