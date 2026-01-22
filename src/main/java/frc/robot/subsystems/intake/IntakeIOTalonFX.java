package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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

  private final TalonFX motorLeft;
  private final TalonFX motorRight;
  private final CANcoder canCoder;

  private final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  private StatusSignal<Angle> positionLeft;
  private StatusSignal<AngularVelocity> velocityLeft;
  private StatusSignal<Voltage> appliedVoltsLeft;
  private StatusSignal<Current> supplyCurrentAmpsLeft;
  private StatusSignal<Current> statorCurrentAmpsLeft;
  private StatusSignal<Temperature> temperatureCelsiusLeft;

  private final VoltageOut voltageControl = new VoltageOut(0.0);
  private final PositionVoltage positionControl = new PositionVoltage(0.0);

  public IntakeIOTalonFX(
      IntakeHardware hardware, IntakeMotorConfiguration configuration, IntakeGains gains) {

    motorLeft = new TalonFX(hardware.motorIdLeft());
    motorRight = new TalonFX(hardware.motorIdRight());
    canCoder = new CANcoder(hardware.canCoderId());

    motorConfiguration.Slot0.kP = gains.p();
    motorConfiguration.Slot0.kI = gains.i();
    motorConfiguration.Slot0.kD = gains.d();
    motorConfiguration.Slot0.kS = gains.s();
    motorConfiguration.Slot0.kV = gains.v();
    motorConfiguration.Slot0.kA = gains.a();
    motorConfiguration.Slot0.kG = gains.g();
    
    motorConfiguration.Feedback.FeedbackRemoteSensorID= canCoder.getDeviceID();
    motorConfiguration.Feedback.FeedbackSensorSource= FeedbackSensorSourceValue.FusedCANcoder;
    motorConfiguration.Feedback.RotorToSensorRatio= IntakeConstants.kGearingRatio;

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

    motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfiguration.Feedback.SensorToMechanismRatio =
        1.0; // raw rotations, we handle meters conversion

    motorLeft.setPosition(0.0);
    motorRight.setPosition(0.0);

    motorLeft.getConfigurator().apply(motorConfiguration, 1.0);
    motorRight.getConfigurator().apply(motorConfiguration, 1.0);

    positionLeft = motorLeft.getPosition();
    velocityLeft = motorLeft.getVelocity();
    appliedVoltsLeft = motorLeft.getMotorVoltage();
    supplyCurrentAmpsLeft = motorLeft.getSupplyCurrent();
    statorCurrentAmpsLeft = motorLeft.getStatorCurrent();
    temperatureCelsiusLeft = motorLeft.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.kStatusSignalUpdateFrequencyHz,
        positionLeft,
        velocityLeft,
        appliedVoltsLeft,
        supplyCurrentAmpsLeft,
        statorCurrentAmpsLeft,
        temperatureCelsiusLeft);

    motorLeft.optimizeBusUtilization(0.0, 1.0);
    motorRight.optimizeBusUtilization(0.0, 1.0);

    // Right motor follows left
    motorRight.setControl(new Follower(hardware.motorIdLeft(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.isMotorConnected =
        BaseStatusSignal.refreshAll(
                positionLeft,
                velocityLeft,
                appliedVoltsLeft,
                supplyCurrentAmpsLeft,
                statorCurrentAmpsLeft,
                temperatureCelsiusLeft)
            .isOK();

    inputs.position = Units.rotationsToRadians(positionLeft.getValueAsDouble());
    inputs.velocityRadianssPerSec = Units.rotationsToRadians(velocityLeft.getValueAsDouble());
    inputs.appliedVolts = appliedVoltsLeft.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmpsLeft.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmpsLeft.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsiusLeft.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) 
  {

    motorLeft.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setPosition(double positionRadians) {
    motorLeft.setControl(
        positionControl.withPosition(Units.radiansToRotations(positionRadians)).withSlot(0));
  }

  @Override
  public void stop() {
    motorLeft.stopMotor();
  }

  @Override
  public void resetPosition() {
    motorLeft.setPosition(0.0);
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

    motorLeft.getConfigurator().apply(slot0);
  }

  @Override
  public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {
    var motionMagic = motorConfiguration.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = Units.radiansToRotations(maxVelocity);
    motionMagic.MotionMagicAcceleration = Units.radiansToRotations(maxAcceleration);
    motionMagic.MotionMagicJerk = 10.0 * Units.radiansToRotations(maxAcceleration);

    motorLeft.getConfigurator().apply(motionMagic);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motorLeft.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
