package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.subsystems.pivot.PivotConstants.PivotGains;
import frc.robot.subsystems.pivot.PivotConstants.PivotHardware;
import frc.robot.subsystems.pivot.PivotConstants.PivotMotorConfiguration;

public class PivotIOTalonFX implements PivotIO {

  private final TalonFX motor;

  private final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  private StatusSignal<Angle> positionDegreesLeft;
  private StatusSignal<AngularVelocity> velocityDegreesPerSecLeft;
  private StatusSignal<Voltage> appliedVoltsLeft;
  private StatusSignal<Current> supplyCurrentAmpsLeft;
  private StatusSignal<Current> statorCurrentAmpsLeft;
  private StatusSignal<Temperature> temperatureCelsiusLeft;

  private final PositionVoltage positionControl = new PositionVoltage(0.0);
  private final MotionMagicVoltage positionControlMM = new MotionMagicVoltage(0);

  public PivotIOTalonFX(
      PivotHardware hardware, PivotMotorConfiguration configuration, PivotGains gains, double minRotation, double maxRotation) {

    motor = new TalonFX(hardware.motorID());

    motorConfiguration.Slot0.kP = gains.p();
    motorConfiguration.Slot0.kI = gains.i();
    motorConfiguration.Slot0.kD = gains.d();
    motorConfiguration.Slot0.kS = gains.s();
    motorConfiguration.Slot0.kV = gains.v();
    motorConfiguration.Slot0.kA = gains.a();
    motorConfiguration.Slot0.kG = gains.g();

    motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxRotation;

    motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minRotation;

    // Motion Magic values converted to rotations
    // motorConfiguration.MotionMagic.MotionMagicCruiseVelocity =
    //     radiansToRotations(gains.maxVelocityDegPerSec());
    // motorConfiguration.MotionMagic.MotionMagicAcceleration =
    //     radiansToRotations(gains.maxAccelerationDegPerSec2());
    // motorConfiguration.MotionMagic.MotionMagicJerk = radiansToRotations(gains.maxJerkDegPerSec3());

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
    motorConfiguration.Feedback.SensorToMechanismRatio = hardware.gearRatio();
    motorConfiguration.Feedback.FeedbackRotorOffset = hardware.rotorOffset();

    motor.setPosition(0.0);

    motor.getConfigurator().apply(motorConfiguration, 1.0);

    positionDegreesLeft = motor.getPosition();
    velocityDegreesPerSecLeft = motor.getVelocity();
    appliedVoltsLeft = motor.getMotorVoltage();
    supplyCurrentAmpsLeft = motor.getSupplyCurrent();
    statorCurrentAmpsLeft = motor.getStatorCurrent();
    temperatureCelsiusLeft = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        PivotConstants.kStatusSignalUpdateFrequencyHz,
        positionDegreesLeft,
        velocityDegreesPerSecLeft,
        appliedVoltsLeft,
        supplyCurrentAmpsLeft,
        statorCurrentAmpsLeft,
        temperatureCelsiusLeft);

    motor.optimizeBusUtilization(0.0, 1.0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.isMotorConnected =
        BaseStatusSignal.refreshAll(
                positionDegreesLeft,
                velocityDegreesPerSecLeft,
                appliedVoltsLeft,
                supplyCurrentAmpsLeft,
                statorCurrentAmpsLeft,
                temperatureCelsiusLeft)
            .isOK();

    inputs.positionDegrees = positionDegreesLeft.getValueAsDouble();
    inputs.velocityDegreesPerSec = velocityDegreesPerSecLeft.getValueAsDouble();
    inputs.appliedVolts = appliedVoltsLeft.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmpsLeft.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmpsLeft.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsiusLeft.getValueAsDouble();
  }

  @Override
  public void setPosition(double rots) {
    motor.setControl(positionControl.withPosition(rots).withSlot(0));
  }

  @Override
  public void setPositionMM(double radians) {
    // motor.setControl(positionControlMM.withPosition(radiansToRotations(radians)).withSlot(0));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setGains(double p, double i, double d, double v, double s, double g, double a) {
    var slot0 = new Slot0Configs();
    slot0.kP = p;
    slot0.kI = i;
    slot0.kD = d;
    slot0.kS = s;
    slot0.kV = v;
    slot0.kG = g;

    motor.getConfigurator().apply(slot0);
  }

  @Override
  public void setMotionMagicConstraints(
      double maxVelocityDegPerSec, double maxAccelerationDegPerSec2) {
    // var motionMagic = motorConfiguration.MotionMagic;
    // motionMagic.MotionMagicCruiseVelocity = radiansToRotations(maxVelocityDegPerSec);
    // motionMagic.MotionMagicAcceleration = radiansToRotations(maxAccelerationDegPerSec2);
    // motionMagic.MotionMagicJerk = 10.0 * radiansToRotations(maxAccelerationDegPerSec2);

    // motor.getConfigurator().apply(motionMagic);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
