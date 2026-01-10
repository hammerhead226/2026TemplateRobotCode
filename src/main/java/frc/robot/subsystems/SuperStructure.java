package frc.robot.subsystems;

import frc.robot.constants.SubsystemConstants.*;
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
    private SuperstructureState currentState = SubsystemConstants.SuperstructureState.IDLE;
    private SuperstructureState wantedState = SubsystemConstants.SuperstructureState.IDLE;
    private SuperstructureState lastState = SubsystemConstants.SuperstructureState.IDLE;

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

    public SuperstructureState getCurrentState() {
        return currentState;
    }

    public SuperstructureState getLastState() {
        return lastState;
    }

    public boolean atGoals() {
        switch (currentState) {
            case INTAKE:
                return (arm.hasReachedGoal(ArmConstants.INTAKE_ANGLE_DEGREES)
                        && elevator.hasReachedGoal(ElevatorConstants.INTAKE_SETPOINT_INCH));

            case SCORELOW:
                return (arm.hasReachedGoal(ArmConstants.LOW_SETPOINT_DEG)
                        && elevator.hasReachedGoal(ElevatorConstants.LOW_SETPOINT_INCH));

            case SCOREMID:
                return (arm.hasReachedGoal(ArmConstants.MID_SETPOINT_DEG)
                        && elevator.hasReachedGoal(ElevatorConstants.MID_SETPOINT_INCH));

            case SCOREHIGH:
                return (arm.hasReachedGoal(ArmConstants.HIGH_SETPOINT_DEG)
                        && elevator.hasReachedGoal(ElevatorConstants.HIGH_SETPOINT_INCH));

            case STOW:
                return (arm.hasReachedGoal(ArmConstants.STOW_SETPOINT_DEG)
                        && elevator.hasReachedGoal(ElevatorConstants.STOW_SETPOINT_INCH));

            default:
                return false;
        }
    }

    public SequentialCommandGroup getSuperStructureCommand() {
        switch (wantedState) {
            case INTAKE:
                currentState = SuperstructureState.INTAKE;
                PathfindToPose command;
                command = new PathfindToPose(
                        drive,
                        SubsystemConstants.PathConstants.TARG_POSE2D_1,
                        SubsystemConstants.PathConstants.ROUG_CONSTRAINTS);

                return new SequentialCommandGroup(command).andThen(new InstantCommand(() -> nextState()));

            case STOW:
                currentState = SuperstructureState.STOW;
                SoftStagedAlign softCommand;
                softCommand = new SoftStagedAlign(
                        drive,
                        SubsystemConstants.PathConstants.ROUGH_TRANSLATION2D,
                        SubsystemConstants.PathConstants.TARG_POSE2D_2.getTranslation(),
                        SubsystemConstants.PathConstants.ROUG_CONSTRAINTS,
                        SubsystemConstants.PathConstants.PRECISE_CONSTRAINTS);
                return softCommand.andThen(new InstantCommand(() -> nextState()));
            case SCOREMID:
                currentState = SuperstructureState.STOW;
                SoftStagedAlign softCommandOne;
                softCommandOne = new SoftStagedAlign(
                        drive,
                        SubsystemConstants.PathConstants.ROUGH_TRANSLATION2D,
                        SubsystemConstants.PathConstants.TARG_POSE2D_3.getTranslation(),
                        SubsystemConstants.PathConstants.ROUG_CONSTRAINTS,
                        SubsystemConstants.PathConstants.PRECISE_CONSTRAINTS);
                return softCommandOne.andThen(new InstantCommand(() -> nextState()));
            case SCOREHIGH:
                currentState = SuperstructureState.STOW;
                SoftStagedAlign softCommandTwo;
                softCommandTwo = new SoftStagedAlign(
                        drive,
                        SubsystemConstants.PathConstants.ROUGH_TRANSLATION2D,
                        SubsystemConstants.PathConstants.TARG_POSE2D_4.getTranslation(),
                        SubsystemConstants.PathConstants.ROUG_CONSTRAINTS,
                        SubsystemConstants.PathConstants.PRECISE_CONSTRAINTS);
                return softCommandTwo.andThen(new InstantCommand(() -> nextState()));
            case SCORELOW:
                currentState = SuperstructureState.STOW;
                SoftStagedAlign softCommandThree;
                softCommandThree = new SoftStagedAlign(
                        drive,
                        SubsystemConstants.PathConstants.ROUGH_TRANSLATION2D,
                        SubsystemConstants.PathConstants.TARG_POSE2D_5.getTranslation(),
                        SubsystemConstants.PathConstants.ROUG_CONSTRAINTS,
                        SubsystemConstants.PathConstants.PRECISE_CONSTRAINTS);
                return softCommandThree.andThen(new InstantCommand(() -> nextState()));

            default:
                return new SequentialCommandGroup().andThen(new InstantCommand(() -> nextState()));
                // return SubsystemConstants.SuperstructureState.IDLE;
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
