package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants.IntakeGains;
import frc.robot.subsystems.intake.IntakeConstants.IntakeHardware;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMotorConfiguration;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX motorIntakePivot;
  // private final CANcoder canCoder; // TODO: charlie

  private final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> statorCurrentAmps;
  private StatusSignal<Temperature> temperatureCelsius;

  private final PositionVoltage positionControl = new PositionVoltage(0.0);

  public IntakeIOTalonFX(
      IntakeHardware hardware, IntakeMotorConfiguration configuration, IntakeGains gains) {

    motorIntakePivot = new TalonFX(hardware.motorIdPivot());
    // canCoder = new CANcoder(hardware.canCoderId()); // TODO: charlie

    motorConfiguration.Slot0.kP = gains.p();
    motorConfiguration.Slot0.kI = gains.i();
    motorConfiguration.Slot0.kD = gains.d();
    motorConfiguration.Slot0.kS = gains.s();
    motorConfiguration.Slot0.kV = gains.v();
    motorConfiguration.Slot0.kA = gains.a();
    motorConfiguration.Slot0.kG = gains.g();
    motorConfiguration.Slot0.GravityType = IntakeConstants.gravityType;
    
    // motorConfiguration.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
    motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfiguration.Feedback.SensorToMechanismRatio = IntakeConstants.kGearingRatioSensorToMechanism;

    // Motion Magic in ROTATIONS, convert from meters
    motorConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        Units.radiansToRotations(gains.maxVelocityMetersPerSecond());
    motorConfiguration.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(gains.maxAccelerationMetersPerSecondSquared());
    motorConfiguration.MotionMagic.MotionMagicJerk =
        Units.radiansToRotations(gains.jerkMetersPerSecondCubed());

    motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.supplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable =
        configuration.enableSupplyCurrentLimit();
    motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable =
        configuration.enableStatorCurrentLimit();

    motorConfiguration.MotorOutput.NeutralMode = configuration.neutralMode();
    motorConfiguration.MotorOutput.Inverted =
        configuration.invert()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    motorIntakePivot.setPosition(Intake.IntakeGoal.kStow.getGoalRadians());

    motorIntakePivot.getConfigurator().apply(motorConfiguration, 1.0);

    position = motorIntakePivot.getPosition();
    velocity = motorIntakePivot.getVelocity();
    appliedVolts = motorIntakePivot.getMotorVoltage();
    supplyCurrentAmps = motorIntakePivot.getSupplyCurrent();
    statorCurrentAmps = motorIntakePivot.getStatorCurrent();
    temperatureCelsius = motorIntakePivot.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.kStatusSignalUpdateFrequencyHz,
        position,
        velocity,
        appliedVolts,
        supplyCurrentAmps,
        statorCurrentAmps,
        temperatureCelsius);

    motorIntakePivot.optimizeBusUtilization(0.0, 1.0);

    // Right motor follows left
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isMotorConnected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                appliedVolts,
                supplyCurrentAmps,
                statorCurrentAmps,
                temperatureCelsius)
            .isOK();

    inputs.position = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadianssPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setPosition(double positionRadians) {
    motorIntakePivot.setControl(
        positionControl.withPosition(Units.radiansToRotations(positionRadians)).withSlot(0));
  }

  @Override
  public void stop() {
    motorIntakePivot.stopMotor();
  }

  @Override
  public void resetPosition() {
    motorIntakePivot.setPosition(0.0);
  }

  @Override
  public void setGains(double p, double i, double d, double v, double s, double g, double a) {
    var slot0 = new Slot0Configs();
    slot0.kP = p;
    slot0.kI = i;
    slot0.kD = d;
    slot0.kS = s;
    slot0.kV = v;
    slot0.kA = a;
    slot0.kG = g;

    motorIntakePivot.getConfigurator().apply(slot0);
  }

  @Override
  public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {
    var motionMagic = motorConfiguration.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = Units.radiansToRotations(maxVelocity);
    motionMagic.MotionMagicAcceleration = Units.radiansToRotations(maxAcceleration);
    motionMagic.MotionMagicJerk = 10.0 * Units.radiansToRotations(maxAcceleration);

    motorIntakePivot.getConfigurator().apply(motionMagic);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motorIntakePivot.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

@Override
public double getPosition() {
  return position.getValueAsDouble();
}
}
