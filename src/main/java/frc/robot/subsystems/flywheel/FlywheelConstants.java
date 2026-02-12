package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class FlywheelConstants {

  public static final double kMotorRotationsToFlywheelRotations = 1.0;

  public static final double kStatusSignalUpdateFrequencyHz = 100.0;
  public static final int kLinearFilterSampleCount = 5;

  public record FlywheelHardware(
      int motorIDLeft, int motorIDRight, double motorRotationsToFlywheelRotations) {}

  public record FlywheelGains(
      double p,
      double i,
      double d,
      double s,
      double g,
      double v,
      double a,
      double maxVelocityDegPerSec,
      double maxAccelerationDegPerSec2,
      double maxJerkDegPerSec3) {}

  public record FlywheelMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public static final FlywheelHardware kFlywheelHardware =
      new FlywheelHardware(
          33, // left motor CAN ID
          10, // right motor CAN ID
          kMotorRotationsToFlywheelRotations);

  public static final FlywheelGains kFlywheelGains =
      switch (Constants.currentMode) {
        case REAL -> new FlywheelGains(0.22, 0.0, 0.0, 0.0, 0.0, 0.125, 0.0, 120.0, 240.0, 0);

        case SIM -> new FlywheelGains(8.0, 0.0, 0.2, 0.1, 0.3, 1.0, 0.03, 180.0, 360.0, 0);

        default -> new FlywheelGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      };

  public static final FlywheelMotorConfiguration kMotorConfiguration =
      new FlywheelMotorConfiguration(
          false, true, true, 80.0, 50.0, 12.0, -12.0, NeutralModeValue.Coast);
}
