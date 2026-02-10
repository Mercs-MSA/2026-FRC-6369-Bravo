package frc.robot.subsystems.spindexer;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class SpindexerConstants {

  public static final double kMotorRotationsToIndexRotations = 100.0;

  public static final double kStatusSignalUpdateFrequencyHz = 100.0;
  public static final int kLinearFilterSampleCount = 5;

  public static final double kDefaultSpeedRPS = 15.0; // TODO: tune


  public record SpindexerHardware(
      int motorIDIndex, double motorRotationsToIndexRotations) {}

  public record SpindexerGains(
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

  public record SpindexerMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public static final SpindexerHardware kIndexHardware =
      new SpindexerHardware(
          33, // TODO: replace with correct id
          kMotorRotationsToIndexRotations);

  public static final SpindexerGains kIndexGains =
      switch (Constants.currentMode) {
        case REAL -> new SpindexerGains(0.4, 0.0, 0.0, 0.0, 0.0, 0.14, 0.0, 120.0, 240.0, 0);  //TODO: tune

        case SIM -> new SpindexerGains(8.0, 0.0, 0.2, 0.1, 0.3, 1.0, 0.03, 180.0, 360.0, 0);

        default -> new SpindexerGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      };

  public static final SpindexerMotorConfiguration kMotorConfiguration =
      new SpindexerMotorConfiguration(
          false, true, true, 80.0, 50.0, 12.0, -12.0, NeutralModeValue.Brake);
}
