package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PathfindToPose extends Command {

    private final Drive drive;
    private final Pose2d targetPose;
    private final Command pathCommand;

    private CommandXboxController Xcon;
    private Supplier<Translation2d> translationSupplierX;
    private Supplier<Translation2d> translationSupplierY;
    DoubleSupplier XSup;
    DoubleSupplier YSup;

    private DoubleSupplier omegaSupplier;

    private boolean untilTrajectoryTimeoutCalled = false;

    PPHolonomicDriveController holonomicDriveController =
            new PPHolonomicDriveController(new PIDConstants(1.0, 0.0, 0.0), new PIDConstants(1.0, 0.0, 0.0));

    public PathfindToPose(Drive drive, Pose2d targetPose, PathConstraints constraints) {
        this.drive = drive;
        this.targetPose = targetPose;

        this.pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints);

        addRequirements(drive);
    }

    public PathfindToPose(
            Drive drive,
            Pose2d targetPose,
            Rotation2d goalAngle,
            CommandXboxController Xcon,
            DoubleSupplier XSup,
            DoubleSupplier YSup,
            DoubleSupplier omegaSupplier) {

        this(drive, targetPose, SubsystemConstants.PathConstants.DEFAULT_PATH_CONSTRAINTS);

        this.Xcon = Xcon;
        this.XSup = XSup;
        this.YSup = YSup;

        this.omegaSupplier = omegaSupplier;
    }

    @Override
    public void initialize() {

        PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();

        // if(Xcon.getLeftX()>0.7||Xcon.getLeftY()>0.7){
        // holonomicDriveController.calculateRobotRelativeSpeeds(drive.getPose(), targetState);
        // }
        PPHolonomicDriveController.overrideRotationFeedback(omegaSupplier);
        // should be a percentage of the robot max speed, because right now max speed is 1m/s
        PPHolonomicDriveController.overrideXFeedback(XSup);
        PPHolonomicDriveController.overrideYFeedback(YSup);
        pathCommand.initialize();
    }

    @Override
    public void execute() {
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
