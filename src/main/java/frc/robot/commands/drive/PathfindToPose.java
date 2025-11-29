package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class PathfindToPose extends Command {
    private final Drive drive;
    private final Command pathCommand;

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;

    private boolean untilTrajectoryTimeoutCalled = false;

    public PathfindToPose(Drive drive, Pose2d targetPose, PathConstraints constraints) {
        addRequirements(drive);
        this.drive = drive;
        this.pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    public PathfindToPose(
            Drive drive,
            Pose2d targetPose,
            PathConstraints constraints,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        this(drive, targetPose, constraints);
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
    }

    @Override
    public void initialize() {
        pathCommand.initialize();
    }

    @Override
    public void execute() {
        if (xSupplier != null) PPHolonomicDriveController.overrideXFeedback(xSupplier);
        if (ySupplier != null) PPHolonomicDriveController.overrideYFeedback(ySupplier);
        if (omegaSupplier != null) PPHolonomicDriveController.overrideRotationFeedback(omegaSupplier);
        pathCommand.execute();
    }

    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        if (untilTrajectoryTimeoutCalled) if (pathCommand.isFinished()) return true;

        return false;
    }

    public PathfindToPose untilTrajectoryTimeout() {
        this.untilTrajectoryTimeoutCalled = true;
        return this;
    }
}
