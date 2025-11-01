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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;





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
     PPHolonomicDriveController controller;
    private final CommandXboxController driverController;



    public OnTheFlyPath(Drive drive,  Pose2d targetPose, Rotation2d goalAngle, double vel, double acc, PPHolonomicDriveController controller, CommandXboxController driverController) { 
        this.drive = drive;
        this.targetPose = targetPose; 
        this.goalAngle = goalAngle;  
        this.vel = vel;
        this.acc = acc;
        this.controller = controller;
        this.driverController = driverController;

    }
    public OnTheFlyPath(Drive drive,  Pose2d targetPose, Rotation2d goalAngle, PPHolonomicDriveController controller, CommandXboxController driverController) { 
        this.drive = drive;
        this.targetPose = targetPose; 
        this.goalAngle = goalAngle;  
        this.vel = SubsystemConstants.PathConstants.DEFAULT_VEL; // default velocity
        this.acc = SubsystemConstants.PathConstants.DEFAULT_ACC; 
        this.controller = controller; 
        this.driverController = driverController;
        // default acceleration
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

        if(driverController.getRightTriggerAxis()>0.5 || driverController.getLeftTriggerAxis()>0.5){
            pathCommand.end(true);
        }
        
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
