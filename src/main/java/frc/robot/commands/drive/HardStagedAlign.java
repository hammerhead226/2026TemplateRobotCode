package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class HardStagedAlign extends SequentialCommandGroup {
    private final Command roughPathCommand;
    private final Command precisePathCommand;

    public HardStagedAlign(
            Drive drive,
            Translation2d roughTranslation,
            Translation2d preciseTranslation,
            PathConstraints roughConstraints,
            PathConstraints preciseConstraints) {
        addRequirements(drive);

        ChassisSpeeds fieldRelChassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
        double chassisSpeedsMagnitude =
                Math.hypot(fieldRelChassisSpeeds.vxMetersPerSecond, fieldRelChassisSpeeds.vyMetersPerSecond);

        Rotation2d startingHeading;
        if (chassisSpeedsMagnitude >= 0.2) {
            // the robot's speed is substantial, consider it in the path's starting conditions
            startingHeading =
                    new Rotation2d(fieldRelChassisSpeeds.vxMetersPerSecond, fieldRelChassisSpeeds.vyMetersPerSecond);
        } else {
            // the robot's speed is small, just head to the roughTranslation
            Translation2d robotToRoughTranslation =
                    roughTranslation.minus(drive.getPose().getTranslation());

            Logger.recordOutput("HardStagedAlign/robotToRoughTranslation", robotToRoughTranslation);

            if (robotToRoughTranslation.getNorm() <= 1e-6) { // from Rotation2d's internal epilson value
                // too close, instead default direction to 0
                startingHeading = Rotation2d.kZero;
            } else {
                startingHeading = new Rotation2d(robotToRoughTranslation.getX(), robotToRoughTranslation.getY());
            }
        }

        Logger.recordOutput("HardStagedAlign/startingHeading", startingHeading);

        Translation2d roughToPrecise = preciseTranslation.minus(roughTranslation);
        Rotation2d alignmentHeading = new Rotation2d(roughToPrecise.getX(), roughToPrecise.getY());

        List<Waypoint> roughWaypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(drive.getPose().getTranslation(), startingHeading),
                new Pose2d(roughTranslation, alignmentHeading));

        List<Waypoint> preciseWaypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(roughTranslation, alignmentHeading), new Pose2d(preciseTranslation, alignmentHeading));

        PathPlannerPath roughPath = new PathPlannerPath(
                roughWaypoints,
                roughConstraints,
                new IdealStartingState(chassisSpeedsMagnitude, drive.getRotation()),
                new GoalEndState(preciseConstraints.maxVelocity(), alignmentHeading));

        PathPlannerPath precisePath = new PathPlannerPath(
                preciseWaypoints,
                preciseConstraints,
                new IdealStartingState(preciseConstraints.maxVelocity(), alignmentHeading),
                new GoalEndState(0.0, alignmentHeading));

        // force field relative coordinates
        if (AutoBuilder.shouldFlip()) {
            roughPath = roughPath.flipPath();
            precisePath = precisePath.flipPath();
        }

        roughPathCommand = AutoBuilder.followPath(roughPath);
        precisePathCommand = AutoBuilder.followPath(precisePath);

        addCommands(roughPathCommand, precisePathCommand);
    }
}
