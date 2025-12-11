package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
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
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

public class SoftStagedAlign extends Command {
    private final double ROUGH_CONSTRAINTS_MAX_POSITION = 0.9;

    private final Drive drive;
    private final Translation2d roughTranslation;
    private final Translation2d preciseTranslation;
    private final PathConstraints roughConstraints;
    private final PathConstraints preciseConstraints;

    private Command pathCommand;

    private boolean withTrajectoryTimeoutCalled = false;

    public SoftStagedAlign(
            Drive drive,
            Translation2d roughTranslation,
            Translation2d preciseTranslation,
            PathConstraints roughConstraints,
            PathConstraints preciseConstraints) {
        addRequirements(drive);
        this.drive = drive;
        this.roughTranslation = roughTranslation;
        this.preciseTranslation = preciseTranslation;
        this.roughConstraints = roughConstraints;
        this.preciseConstraints = preciseConstraints;
    }

    public void initialize() {
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

            if (robotToRoughTranslation.getNorm() <= 1e-6) { // from Rotation2d's internal epilson value
                // too close, instead default direction to 0
                startingHeading = Rotation2d.kZero;
            } else {
                startingHeading = new Rotation2d(robotToRoughTranslation.getX(), robotToRoughTranslation.getY());
            }
        }

        Translation2d roughToPrecise = preciseTranslation.minus(roughTranslation);
        Rotation2d alignmentHeading = new Rotation2d(roughToPrecise.getX(), roughToPrecise.getY());

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(drive.getPose().getTranslation(), startingHeading),
                new Pose2d(roughTranslation, alignmentHeading),
                new Pose2d(preciseTranslation, alignmentHeading));

        List<ConstraintsZone> constraintsZones = new ArrayList<>();
        constraintsZones.add(new ConstraintsZone(0.0, ROUGH_CONSTRAINTS_MAX_POSITION, roughConstraints));

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                new ArrayList<>(), // holonomicRotations
                new ArrayList<>(), // pointTowardsZones
                constraintsZones,
                new ArrayList<>(), // eventMarkers
                preciseConstraints, // globalConstraints
                new IdealStartingState(chassisSpeedsMagnitude, drive.getRotation()),
                new GoalEndState(0.0, alignmentHeading),
                false);

        // force field relative coordinates
        if (AutoBuilder.shouldFlip()) {
            path = path.flipPath();
        }

        pathCommand = AutoBuilder.followPath(path);
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
        if (withTrajectoryTimeoutCalled) if (pathCommand.isFinished()) return true;

        return false;
    }

    // custom decorator
    // due to type safety, must be used before WPIlib's Command decorators
    // as Commands cannot be readily coverted back to StagedAlign commands
    public SoftStagedAlign withTrajectoryTimeout() {
        withTrajectoryTimeoutCalled = true;
        return this;
    }
}
