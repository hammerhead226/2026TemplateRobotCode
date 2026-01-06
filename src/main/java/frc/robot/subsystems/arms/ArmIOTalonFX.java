package frc.robot.subsystems.arms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.Conversions;
import org.littletonrobotics.junction.Logger;
// TODO It looks like this class was based on the 2025 template code. I highly highly recommend basing it on the 2025
// actual code as there were a tremendous number of improvements

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX leader;
    private final TalonFX follower;


    private final CANcoder encoder;
    private double positionSetpointDegs;

    private double startAngleDegs;

    private final StatusSignal<Angle> leaderPositionRotations;
    private final StatusSignal<AngularVelocity> velocityDegsPerSec;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorCurrentAmps;
    private final StatusSignal<Current> supplyCurrentAmps;

    public ArmIOTalonFX(int leadID, int followID, int canID) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = SubsystemConstants.ArmConstants.CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = SubsystemConstants.ArmConstants.CURRENT_LIMIT_ENABLED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leader = new TalonFX(leadID, SubsystemConstants.CANBUS);
        follower = new TalonFX(followID, SubsystemConstants.CANBUS);
        encoder = new CANcoder(canID, SubsystemConstants.CANBUS);
        

        leader.getConfigurator().apply(config);

        follower.setControl(new Follower(leadID, true));

        startAngleDegs = Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());

        leader.setPosition(
            Units.degreesToRotations(startAngleDegs)
                * SubsystemConstants.ArmConstants.ARM_GEAR_RATIO);
      
      // -- COMPLETED -- setPosition uses rotations not degrees or Falcons

        follower.setPosition(
                Units.degreesToRotations(startAngleDegs)*SubsystemConstants.ArmConstants.ARM_GEAR_RATIO);

        leaderPositionRotations = leader.getPosition();
        velocityDegsPerSec = leader.getVelocity();
        appliedVolts = leader.getMotorVoltage();
        statorCurrentAmps = leader.getStatorCurrent();
        supplyCurrentAmps = leader.getSupplyCurrent();


        positionSetpointDegs = SubsystemConstants.ArmConstants.STOW_SETPOINT_DEG;

        Logger.recordOutput("Starting Angle", startAngleDegs);

        encoder.optimizeBusUtilization();
        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100, leaderPositionRotations, velocityDegsPerSec, appliedVolts, statorCurrentAmps, supplyCurrentAmps);

        // setBrakeMode(false);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(leaderPositionRotations, velocityDegsPerSec, appliedVolts, statorCurrentAmps, supplyCurrentAmps);
        inputs.pitch = pitch.getValueAsDouble() + SubsystemConstants.ArmConstants.ARM_ZERO_ANGLE;
        inputs.positionDegs = Conversions.falconToDegrees(
                (leaderPositionDegs.getValueAsDouble()), SubsystemConstants.ArmConstants.ARM_GEAR_RATIO)
                + SubsystemConstants.ArmConstants.ARM_ZERO_ANGLE;

        inputs.velocityDegsPerSec = velocityDegsPerSec.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
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
        follower.getConfigurator().apply(config);
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
        this.positionSetpointDegs = leaderPositionDegs.getValueAsDouble();
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
