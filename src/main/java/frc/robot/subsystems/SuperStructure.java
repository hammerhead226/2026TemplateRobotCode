package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.PathfindToPose;
import frc.robot.commands.drive.SoftStagedAlign;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.SuperstructureState;
import frc.robot.subsystems.arms.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.led.LED;

public class SuperStructure {
    @SuppressWarnings("unused")
    private final Drive drive;

    @SuppressWarnings("unused")
    private final Flywheel flywheel;

    private final Arm arm;
    private final LED led;
    private final Elevator elevator;
    private SuperstructureState currentState;
    private SuperstructureState wantedState;
    private SuperstructureState lastState;

    public SuperStructure(Drive drive, Flywheel flywheel, Arm arm, LED led, Elevator elevator) {

        this.drive = drive;
        this.flywheel = flywheel;
        this.arm = arm;
        this.led = led;
        this.elevator = elevator;
    }

    public void setWantedState(SuperstructureState wantedState) {
        if (wantedState == SuperstructureState.SCORELOW) {
            led.setState(LED_STATE.RED);
        } else if (wantedState == SuperstructureState.SCOREMID) {
            led.setState(LED_STATE.GREEN);
        } else if (wantedState == SuperstructureState.SCOREHIGH) {
            led.setState(LED_STATE.BLUE);
        }
        this.wantedState = wantedState;
    }

    public void setCurrentState(SuperstructureState currentState) {
        this.currentState = currentState;
    }

    public SuperstructureState getWantedState() {
        return wantedState;
    }

    public SuperstructureState getCurrenState() {
        return currentState;
    }

    public SuperstructureState getLastState() {
        return lastState;
    }

    public boolean atGoals() {
        switch (currentState) {
            case INTAKE:
                return (arm.hasReachedGoal(SubsystemConstants.ArmConstants.INTAKE_ANGLE_DEGREES)
                        && elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.INTAKE_SETPOINT_INCH));

            case SCORELOW:
                return (arm.hasReachedGoal(SubsystemConstants.ArmConstants.LOW_SETPOINT_DEG)
                        && elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.LOW_SETPOINT_INCH));

            case SCOREMID:
                return (arm.hasReachedGoal(SubsystemConstants.ArmConstants.MID_SETPOINT_DEG)
                        && elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.MID_SETPOINT_INCH));

            case SCOREHIGH:
                return (arm.hasReachedGoal(SubsystemConstants.ArmConstants.HIGH_SETPOINT_DEG)
                        && elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.HIGH_SETPOINT_INCH));

            case STOW:
                return (arm.hasReachedGoal(SubsystemConstants.ArmConstants.STOW_SETPOINT_DEG)
                        && elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH));

            default:
                return false;
        }
    }
    public SequentialCommandGroup getSuperStructureCommand(){
        switch(wantedState){
            case INTAKE:
                currentState = SuperstructureState.INTAKE;
                PathfindToPose command;
                command = new PathfindToPose(drive, SubsystemConstants.PathConstants.TARG_POSE2D, SubsystemConstants.PathConstants.ROUG_CONSTRAINTS);

                return new SequentialCommandGroup(
                    command
                ).andThen(new InstantCommand(() -> nextState()));
            case STOW:
                currentState = SuperstructureState.STOW;
                SoftStagedAlign softCommand;
                softCommand = new SoftStagedAlign(
                    drive, 
                    SubsystemConstants.PathConstants.ROUGH_TRANSLATION2D, 
                    SubsystemConstants.PathConstants.TARG_POSE2D.getTranslation(), 
                    SubsystemConstants.PathConstants.ROUG_CONSTRAINTS, 
                    SubsystemConstants.PathConstants.PRECISE_CONSTRAINTS);
                return new SequentialCommandGroup(
                    softCommand
                ).andThen(new InstantCommand(() -> nextState()));
                
        default:
            return null;
        }
    }        

    public void nextState() {
        switch (currentState) {
            case IDLE:
                lastState = currentState;
                setWantedState(SuperstructureState.INTAKE);
                break;

            case INTAKE:
                lastState = currentState;

                setWantedState(SuperstructureState.SCORELOW);
                break;

            case SCORELOW:
                lastState = currentState;

                setWantedState(SuperstructureState.SCOREMID);
                break;

            case SCOREMID:
                lastState = currentState;

                setWantedState(SuperstructureState.SCOREHIGH);
                break;

            case SCOREHIGH:
                lastState = currentState;

                setWantedState(SuperstructureState.STOW);
                break;

            case STOW:
                lastState = currentState;

                setWantedState(SuperstructureState.IDLE);
                break;

            default:
                break;
        }
    }
}
