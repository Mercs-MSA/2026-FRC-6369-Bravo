package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class PivotConstants {

  public static final double kGearRatio = 1; // rotations motor per rotations pivot
  public static final double kRotorOffset = -0.001953; // rotations, find in TunerX

  public static final double kMinRadians = 0.00;
  public static final double kMaxRadians = 2.0;

  public static final double kPositionToleranceRad = 0.005;

  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public record PivotHardware(
      int motorID, double gearRatio, double rotorOffset) {}

  public record PivotGains(
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

  public record PivotMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}
      // TODO: add min/max soft limits

  public static final PivotHardware kPivotHardware =
      new PivotHardware(
          7, // left motor CAN ID
          kGearRatio,
          kRotorOffset
          );

  public static final PivotGains kPivotGains =
      switch (Constants.currentMode) {
        case REAL -> new PivotGains(3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 120.0, 240.0, 0);

        case SIM -> new PivotGains(8.0, 0.0, 0.2, 0.1, 0.3, 1.0, 0.03, 180.0, 360.0, 0);

        default -> new PivotGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      };

  public static final PivotMotorConfiguration kMotorConfiguration =
      new PivotMotorConfiguration(
          true, true, true, 80.0, 50.0, 12.0, -12.0, NeutralModeValue.Brake);
}
