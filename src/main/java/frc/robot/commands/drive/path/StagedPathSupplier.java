package frc.robot.commands.drive.path;

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
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

// TODO add short description to class to differntiate from HardStagedAlign
public class StagedPathSupplier implements Supplier<PathPlannerPath> {
    private final double INITAL_VELOCITY_THRESHOLD = 0.2;
    private final double ROUGH_CONSTRAINTS_MAX_POSITION = 0.9;

    private final Drive drive;
    private final Translation2d roughTranslation;
    private final Translation2d preciseTranslation;
    private final PathConstraints roughConstraints;
    private final PathConstraints preciseConstraints;

    public StagedPathSupplier(
            Drive drive,
            Translation2d roughTranslation,
            Translation2d preciseTranslation,
            PathConstraints roughConstraints,
            PathConstraints preciseConstraints) {
        this.drive = drive;
        this.roughTranslation = roughTranslation;
        this.preciseTranslation = preciseTranslation;
        this.roughConstraints = roughConstraints;
        this.preciseConstraints = preciseConstraints;
    }

    @Override
    public PathPlannerPath get() {
        ChassisSpeeds fieldRelChassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
        double chassisSpeedsMagnitude =
                Math.hypot(fieldRelChassisSpeeds.vxMetersPerSecond, fieldRelChassisSpeeds.vyMetersPerSecond);

        Rotation2d startingHeading;
        if (chassisSpeedsMagnitude >= INITAL_VELOCITY_THRESHOLD) {
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

        return new PathPlannerPath(
                waypoints,
                new ArrayList<>(), // holonomicRotations
                new ArrayList<>(), // pointTowardsZones
                constraintsZones,
                new ArrayList<>(), // eventMarkers
                preciseConstraints, // globalConstraints
                new IdealStartingState(chassisSpeedsMagnitude, drive.getRotation()),
                new GoalEndState(0.0, alignmentHeading),
                false);
    }
}
