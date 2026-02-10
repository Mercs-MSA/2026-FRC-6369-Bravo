package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX motorLeft;
  private final TalonFX motorRight;

  private final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  private StatusSignal<AngularVelocity> velocityLeft;
  private StatusSignal<Voltage> appliedVoltsLeft;
  private StatusSignal<Current> supplyCurrentAmpsLeft;
  private StatusSignal<Current> statorCurrentAmpsLeft;
  private StatusSignal<Temperature> temperatureCelsiusLeft;

  private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  public FlywheelIOTalonFX(
      FlywheelConstants.FlywheelHardware hardware,
      FlywheelConstants.FlywheelMotorConfiguration configuration,
      FlywheelConstants.FlywheelGains gains) {

    motorLeft = new TalonFX(hardware.motorIDLeft());
    motorRight = new TalonFX(hardware.motorIDRight());

    motorConfiguration.Slot0.kP = gains.p();
    motorConfiguration.Slot0.kI = gains.i();
    motorConfiguration.Slot0.kD = gains.d();
    motorConfiguration.Slot0.kS = gains.s();
    motorConfiguration.Slot0.kV = gains.v();
    motorConfiguration.Slot0.kA = gains.a();
    motorConfiguration.Slot0.kG = gains.g();

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
        hardware.motorRotationsToFlywheelRotations();

    motorLeft.setPosition(0.0);
    motorRight.setPosition(0.0);

    motorLeft.getConfigurator().apply(motorConfiguration, 1.0);
    motorRight.getConfigurator().apply(motorConfiguration, 1.0);

    velocityLeft = motorLeft.getVelocity();
    appliedVoltsLeft = motorLeft.getMotorVoltage();
    supplyCurrentAmpsLeft = motorLeft.getSupplyCurrent();
    statorCurrentAmpsLeft = motorLeft.getStatorCurrent();
    temperatureCelsiusLeft = motorLeft.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        FlywheelConstants.kStatusSignalUpdateFrequencyHz,
        velocityLeft,
        appliedVoltsLeft,
        supplyCurrentAmpsLeft,
        statorCurrentAmpsLeft,
        temperatureCelsiusLeft);

    motorLeft.optimizeBusUtilization(0.0, 1.0);
    motorRight.optimizeBusUtilization(0.0, 1.0);

    motorRight.setControl(new Follower(hardware.motorIDLeft(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.isMotorConnected =
        BaseStatusSignal.refreshAll(
                velocityLeft,
                appliedVoltsLeft,
                supplyCurrentAmpsLeft,
                statorCurrentAmpsLeft,
                temperatureCelsiusLeft)
            .isOK();

    inputs.velocityRotationsPerSec = velocityLeft.getValueAsDouble();
    inputs.appliedVolts = appliedVoltsLeft.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmpsLeft.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmpsLeft.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsiusLeft.getValueAsDouble();
  }

  @Override
  public void setVelocity(double rotationsPerSec) {
    motorLeft.setControl(velocityControl.withVelocity(rotationsPerSec));
  }

  @Override
  public void stop() {
    motorLeft.stopMotor();
  }

  @Override
  public void setGains(double p, double i, double d, double s, double g, double v, double a) {
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
  public void setBrakeMode(boolean brake) {
    motorLeft.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
