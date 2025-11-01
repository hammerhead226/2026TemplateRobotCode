package frc.robot.commands;

import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;




public class OnTheFlyPath extends Command {

    private final Drive drive; 
    Pose2d targetPose;
    private final Rotation2d goalAngle;
    private final double vel;
    private final double acc;
    boolean Finished; 
    BooleanSupplier continuePath;
    Command pathCommand;
    double distanceToTarget;


    public OnTheFlyPath(Drive drive,  Pose2d targetPose, Rotation2d goalAngle, double vel, double acc) { 
        this.drive = drive;
        this.targetPose = targetPose; 
        this.goalAngle = goalAngle;  
        this.vel = vel;
        this.acc = acc;
    }
    public OnTheFlyPath(Drive drive,  Pose2d targetPose, Rotation2d goalAngle) { 
        this.drive = drive;
        this.targetPose = targetPose; 
        this.goalAngle = goalAngle;  
        this.vel = SubsystemConstants.PathConstants.DEFAULT_VEL; // default velocity
        this.acc = SubsystemConstants.PathConstants.DEFAULT_ACC; // default acceleration
    }
      
    public void initialize() {
        
        PathConstraints constraints = 
        new PathConstraints(vel, acc, Math.toRadians(180), Math.toRadians(200)); 
         pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
         pathCommand.initialize();

        
        
    }
    
    public void execute() {
        distanceToTarget = drive.getPose()
        .getTranslation()
        .getDistance
        (targetPose.getTranslation());
        pathCommand.execute();
        
    }   

    public void end (boolean interrupted) {
        drive.stop();

        if(distanceToTarget<=Units.inchesToMeters(4)){
            drive.stop();
        }
    }
    public boolean isFinished() {
        return distanceToTarget<=Units.inchesToMeters(4);
    }
}
