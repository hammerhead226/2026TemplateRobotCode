package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.Conversions;
// TODO It looks like this class was based on the 2025 template code. I highly highly recommend basing it on the 2025
// actual code as there were a tremendous number of improvements

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leader;
    private final TalonFX follower;

    private double positionSetpoint;
    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> leaderStatorCurrentAmps;
    private final StatusSignal<Current> leaderSupplyCurrentAmps;
    private final StatusSignal<Current> followerStatorCurrentAmps;
    private final StatusSignal<Current> followerSupplyCurrentAmps;

    public ElevatorIOTalonFX(int lead, int follow) {
        TalonFXConfiguration leadConfig = new TalonFXConfiguration();
        leadConfig.CurrentLimits.StatorCurrentLimit = SubsystemConstants.ElevatorConstants.CURRENT_LIMIT;
        leadConfig.CurrentLimits.StatorCurrentLimitEnable = SubsystemConstants.ElevatorConstants.CURRENT_LIMIT_ENABLED;
        leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leadConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leadConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leader = new TalonFX(lead, SubsystemConstants.CANBUS);
        follower = new TalonFX(follow, SubsystemConstants.CANBUS);

        leader.getConfigurator().apply(leadConfig);

        positionSetpoint = SubsystemConstants.ElevatorConstants.RETRACT_SETPOINT_INCH;

        follower.setControl(new Follower(lead, true));

        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        follower.getConfigurator().apply(followerConfig);

        elevatorPosition = leader.getPosition();
        elevatorVelocity = leader.getVelocity();
        appliedVolts = leader.getMotorVoltage();
        leaderStatorCurrentAmps = leader.getStatorCurrent();
        leaderSupplyCurrentAmps = leader.getSupplyCurrent();
        followerStatorCurrentAmps = follower.getStatorCurrent();
        followerSupplyCurrentAmps = follower.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                elevatorPosition,
                elevatorVelocity,
                appliedVolts,
                leaderStatorCurrentAmps,
                leaderSupplyCurrentAmps,
                followerStatorCurrentAmps,
                followerSupplyCurrentAmps);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                elevatorPosition,
                elevatorVelocity,
                appliedVolts,
                leaderStatorCurrentAmps,
                leaderSupplyCurrentAmps,
                followerStatorCurrentAmps,
                followerSupplyCurrentAmps);
        inputs.elevatorPositionInch = Conversions.motorRotToInches(
                elevatorPosition.getValueAsDouble(),
                SubsystemConstants.ElevatorConstants.SPROCKET_CIRCUMFERENCE_INCH,
                SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO);
        inputs.elevatorVelocityInchesPerSecond = Conversions.motorRotToInches(
                elevatorVelocity.getValueAsDouble() * 60.,
                SubsystemConstants.ElevatorConstants.SPROCKET_CIRCUMFERENCE_INCH,
                SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO);
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.leaderStatorCurrentAmps = leaderStatorCurrentAmps.getValueAsDouble();
        inputs.leaderSupplyCurrentAmps = leaderSupplyCurrentAmps.getValueAsDouble();
        inputs.followerStatorCurrentAmps = followerStatorCurrentAmps.getValueAsDouble();
        inputs.followerSupplyCurrentAmps = followerSupplyCurrentAmps.getValueAsDouble();
        inputs.positionSetpointInch = positionSetpoint;
    }

    @Override
    public void runCharacterization(double volts) {
        leader.setVoltage(volts);
    }

    @Override
    public void setPositionSetpoint(double position, double ffVolts) {
        this.positionSetpoint = position;
        leader.setControl(new PositionVoltage(Conversions.inchesToMotorRot(
                position,
                SubsystemConstants.ElevatorConstants.SPROCKET_CIRCUMFERENCE_INCH,
                SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO)));
    }

    @Override
    public void zeroElevator() {
        leader.setPosition(0);
    }

    public void stop() {
        this.positionSetpoint = elevatorPosition.getValueAsDouble();
        leader.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        Slot0Configs config = new Slot0Configs();

        config.kP = kP;
        config.kI = kI;
        config.kD = kD;

        leader.getConfigurator().apply(config);
    }
}
