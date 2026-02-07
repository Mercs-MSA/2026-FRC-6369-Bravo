package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.turret.TurretConstants.TurretGains;
import frc.robot.subsystems.turret.TurretConstants.TurretHardware;
import frc.robot.subsystems.turret.TurretConstants.TurretMotorConfiguration;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX motor;
  private final CANcoder cancoder;

  private final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
  private final CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();

  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocityDegreesPerSec;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> statorCurrentAmps;
  private StatusSignal<Temperature> temperatureCelsius;

  private final VoltageOut voltageControl = new VoltageOut(0.0);
  private final PositionVoltage positionControl = new PositionVoltage(0.0);
  private final MotionMagicVoltage positionControlMM = new MotionMagicVoltage(0);

  public TurretIOTalonFX(
      TurretHardware hardware, TurretMotorConfiguration configuration, TurretGains gains, double minRadians, double maxRadians) {

    motor = new TalonFX(hardware.motorIDLeft());
    cancoder = new CANcoder(hardware.cancoderID());

    motorConfiguration.Slot0.kP = gains.p();
    motorConfiguration.Slot0.kI = gains.i();
    motorConfiguration.Slot0.kD = gains.d();
    motorConfiguration.Slot0.kS = gains.s();
    motorConfiguration.Slot0.kV = gains.v();
    motorConfiguration.Slot0.kA = gains.a();
    motorConfiguration.Slot0.kG = gains.g();

    motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.radiansToRotations(maxRadians);

    motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.radiansToRotations(minRadians);

    // Motion Magic values converted to rotations
    motorConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        degreesToRotations(gains.maxVelocityDegPerSec());
    motorConfiguration.MotionMagic.MotionMagicAcceleration =
        degreesToRotations(gains.maxAccelerationDegPerSec2());
    motorConfiguration.MotionMagic.MotionMagicJerk = degreesToRotations(gains.maxJerkDegPerSec3());

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

    // TODO: update for fused cancoder
    motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // motorConfiguration.Feedback.FeedbackRemoteSensorID = hardware.cancoderID();
    motorConfiguration.Feedback.SensorToMechanismRatio = hardware.sensorMechanismGearRatio();
    motorConfiguration.Feedback.RotorToSensorRatio = hardware.rotorSensorGearRatio();

    motor.getConfigurator().apply(motorConfiguration, 1.0);

    positionRotations = motor.getPosition();
    velocityDegreesPerSec = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrentAmps = motor.getSupplyCurrent();
    statorCurrentAmps = motor.getStatorCurrent();
    temperatureCelsius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        TurretConstants.kStatusSignalUpdateFrequencyHz,
        positionRotations,
        velocityDegreesPerSec,
        appliedVolts,
        supplyCurrentAmps,
        statorCurrentAmps,
        temperatureCelsius);

    motor.optimizeBusUtilization(0.0, 1.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.isMotorConnected =
        BaseStatusSignal.refreshAll(
                positionRotations,
                velocityDegreesPerSec,
                appliedVolts,
                supplyCurrentAmps,
                statorCurrentAmps,
                temperatureCelsius)
            .isOK();

    inputs.positionRadians = Units.rotationsToRadians(positionRotations.getValueAsDouble());
    inputs.velocityDegreesPerSec = velocityDegreesPerSec.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setPosition(double radians) {
    motor.setControl(positionControl.withPosition(Units.radiansToRotations(MathUtil.clamp(MathUtil.angleModulus(radians - TurretConstants.kHomeRadians), TurretConstants.kMinRadiansLimit, TurretConstants.kMaxRadiansLimit))).withSlot(0));
  }

  @Override
  public void setPositionMM(double radians) {
    motor.setControl(positionControlMM.withPosition(Units.radiansToRotations(MathUtil.clamp(MathUtil.angleModulus(radians - TurretConstants.kHomeRadians), TurretConstants.kMinRadiansLimit, TurretConstants.kMaxRadiansLimit))).withSlot(0));
  }

  @Override
  public double getPositionRadians() {
    return Units.rotationsToRadians(positionRotations.getValueAsDouble());
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void resetPosition() {
    motor.setPosition(0.0);
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
    var motionMagic = motorConfiguration.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = degreesToRotations(maxVelocityDegPerSec);
    motionMagic.MotionMagicAcceleration = degreesToRotations(maxAccelerationDegPerSec2);
    motionMagic.MotionMagicJerk = 10.0 * degreesToRotations(maxAccelerationDegPerSec2);

    motor.getConfigurator().apply(motionMagic);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  private double degreesToRotations(double degrees) {
    return degrees / TurretConstants.kRotorRotationsToDegrees;
  }
}
