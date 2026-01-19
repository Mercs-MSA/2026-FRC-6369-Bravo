package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public class FlywheelIOInputs {
    public boolean isMotorConnected = false;
    public double positionRotations = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public void updateInputs(FlywheelIOInputs inputs);

  public void setVelocity(double velocityRotationsPerSec);

  public void resetPosition();

  public void stop();

  public void setGains(double p, double i, double d, double v, double s, double g, double a);

  public void setBrakeMode(boolean brake);
}
