package frc.robot.subsystems.index;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class IndexConstants {

  public static final double kSensorToMechanismGearing = 2.0;

  public static final double kStatusSignalUpdateFrequencyHz = 100.0;
  public static final int kLinearFilterSampleCount = 5;

  public static final double kSpeed = 35; // TODO: tune
  public static final double kIndexVelocityLimitRPS = 50.0; // TODO: tune

  public record IndexHardware(
      int motorIDIndex, double motorRotationsToIndexRotations) {}

  public record IndexGains(
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

  public record IndexMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public static final IndexHardware kIndexHardware =
      new IndexHardware(
          8, // TODO: replace with correct id
          kSensorToMechanismGearing);

  public static final IndexGains kIndexGains =
      switch (Constants.currentMode) {
        case REAL -> new IndexGains(0.4, 0.0, 0.0, 0.0, 0.0, 0.265, 0.0, 120.0, 240.0, 0);

        case SIM -> new IndexGains(8.0, 0.0, 0.2, 0.1, 0.3, 1.0, 0.03, 180.0, 360.0, 0);

        default -> new IndexGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      };

  public static final IndexMotorConfiguration kMotorConfiguration =
      new IndexMotorConfiguration(
          true, true, true, 80.0, 50.0, 12.0, -12.0, NeutralModeValue.Brake);
}
