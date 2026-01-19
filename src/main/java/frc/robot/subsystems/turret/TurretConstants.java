package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class TurretConstants {

  public static final double kGearRatio = 100.0; // rotations motor per rotations pivot

  public static final double kRotorRotationsToDegrees = 360.0 / kGearRatio;

  public static final double kHomeRadians = 0.0;
  public static final double kToleranceRotations = 0.5;

  public static final double kMinRadiansLimit = -Math.PI;
  public static final double kMaxRadiansLimit = Math.PI;

  public static final double kTurretOffsetY = 0.3;

  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public static final double kLimelightTxP = 0.5;
  public static final double kLimelightTagTA = 8.0;

  public record TurretHardware(
      int motorIDLeft, int cancoderID, double gearRatio, double rotorRotationsToDegrees) {}

  public record TurretGains(
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

  public record TurretMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public static final TurretHardware kTurretHardware =
      new TurretHardware(
          22, // motor CAN ID
          23, // cancoder CAN ID
          kGearRatio,
          kRotorRotationsToDegrees);

  public static final TurretGains kTurretGains =
      switch (Constants.currentMode) {
        case REAL -> new TurretGains(12.0, 0.0, 0.4, 0.2, 0.6, 1.3, 0.05, 120.0, 240.0, 0);

        case SIM -> new TurretGains(8.0, 0.0, 0.2, 0.1, 0.3, 1.0, 0.03, 180.0, 360.0, 0);

        default -> new TurretGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      };

  public static final TurretMotorConfiguration kMotorConfiguration =
      new TurretMotorConfiguration(
          true, true, true, 80.0, 50.0, 12.0, -12.0, NeutralModeValue.Brake);
}
