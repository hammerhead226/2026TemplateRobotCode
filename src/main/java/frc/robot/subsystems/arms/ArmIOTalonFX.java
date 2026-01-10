package frc.robot.subsystems.arms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.Conversions;
import org.littletonrobotics.junction.Logger;
// TODO It looks like this class was based on the 2025 template code. I highly highly recommend basing it on the 2025
// actual code as there were a tremendous number of improvements

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX leader;

    private final CANcoder encoder;
    private double positionSetpointDegs;

    private double startAngleDegs;

    private final StatusSignal<Angle> leaderPositionRotations;
    private final StatusSignal<AngularVelocity> velocityDegsPerSec;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorCurrentAmps;
    private final StatusSignal<Current> supplyCurrentAmps;

    public ArmIOTalonFX(int leadID, int followID, int canID) {
        CANcoderConfiguration coderConfig = new CANcoderConfiguration();

        coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = SubsystemConstants.ArmConstants.CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = SubsystemConstants.ArmConstants.CURRENT_LIMIT_ENABLED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leader = new TalonFX(leadID, SubsystemConstants.CANBUS);

        encoder = new CANcoder(canID, SubsystemConstants.CANBUS);

        leader.getConfigurator().apply(config);

        if (encoder.isConnected()) {
            leader.setPosition((encoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(57 - 12))
                    * SubsystemConstants.ArmConstants.ARM_GEAR_RATIO);
        } else {
            leader.setPosition(Units.degreesToRotations(SubsystemConstants.ArmConstants.STOW_SETPOINT_DEG)
                    * SubsystemConstants.ArmConstants.ARM_GEAR_RATIO);
        }

        leaderPositionRotations = leader.getPosition();
        velocityDegsPerSec = leader.getVelocity();
        appliedVolts = leader.getMotorVoltage();
        statorCurrentAmps = leader.getStatorCurrent();
        supplyCurrentAmps = leader.getSupplyCurrent();

    positionSetpointDegs = SubsystemConstants.ArmConstants.ARM_DEFAULT_POSITION;

        Logger.recordOutput("Starting Angle", startAngleDegs);

        encoder.optimizeBusUtilization();
        leader.optimizeBusUtilization();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100, leaderPositionRotations, velocityDegsPerSec, appliedVolts, statorCurrentAmps, supplyCurrentAmps);

      }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leaderPositionRotations, velocityDegsPerSec, appliedVolts, statorCurrentAmps, supplyCurrentAmps);

        inputs.positionDegs = Units.rotationsToDegrees(leaderPositionRotations.getValueAsDouble())
                / SubsystemConstants.ArmConstants.ARM_GEAR_RATIO;

        inputs.velocityDegsPerSec = velocityDegsPerSec.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.positionSetpointDegs = positionSetpointDegs;
    }

    @Override
    public void setBrakeMode(boolean bool) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        if (bool) {
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }

        leader.getConfigurator().apply(config);
    }

    @Override
    public void setPositionSetpointDegs(double positionDegs, double ffVolts) {
        this.positionSetpointDegs = positionDegs;
        leader.setControl(new PositionVoltage(Conversions.degreesToFalcon(
                positionDegs,
                SubsystemConstants.ArmConstants.ARM_GEAR_RATIO))); // CHECK FOR STOW ANGLE (positionDegs - 59)
    }

    @Override
    public void stop() {
        this.positionSetpointDegs = Units.rotationsToDegrees(leaderPositionRotations.getValueAsDouble());
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
