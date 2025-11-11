package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.drive.Drive;

public class PathfindToPose extends Command {
  private final Drive drive;
  private final Pose2d targetPose;
  private final Command pathCommand;

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
    pathCommand.initialize();
  }

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

  // custom decorator
  // due to type safety, must be used before WPIlib's Command decorators
  // as Commands cannot be readily coverted back to OnTheFlyPath commands
  public PathfindToPose untilTrajectoryTimeout() {
    untilTrajectoryTimeoutCalled = true;
    return this;
  }
}
