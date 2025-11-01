package frc.robot.commands;

import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Rotation2d;



public class onTheFlyPath{

    private final Drive drive; 
    Pose2d startPose;
    Pose2d targetPose;
    private final Rotation2d goalAngle;
    private final double vel;
    private final double acc;


    public onTheFlyPath(Drive drive, Pose2d startPose, Pose2d targetPose, Rotation2d goalAngle, double vel, double acc) { 
        this.drive = drive;
        this.startPose = startPose;
        this.targetPose = targetPose; 
        this.goalAngle = goalAngle;  
        this.vel = vel;
        this.acc = acc;
    }
    public onTheFlyPath(Drive drive, Pose2d startPose, Pose2d targetPose, Rotation2d goalAngle) { 
        this.drive = drive;
        this.startPose = startPose;
        this.targetPose = targetPose; 
        this.goalAngle = goalAngle;  
        this.vel = 2.5; // default velocity
        this.acc = 2.5; // default acceleration
    }
      
    public void initialize() {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d (drive.getPose().getX(), drive.getPose().getY(), drive.getRotation()),
            new Pose2d (targetPose.getX(), targetPose.getY(), goalAngle));

        PathConstraints constraints = new PathConstraints(vel, acc, Math.toRadians(100), Math.toRadians(100)); 

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null,
         new GoalEndState(0.0, Rotation2d.fromDegrees(0.0)));
        
    }


}
