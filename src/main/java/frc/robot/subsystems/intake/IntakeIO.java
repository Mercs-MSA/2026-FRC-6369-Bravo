package frc.robot.subsystems.intake;

public interface IntakeIO {

  public class IntakeIOInputs {
    public boolean isMotorConnected = false;

    public double position = 0.0;
    public double velocityRadianssPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public void updateInputs(IntakeIOInputs inputs);

  public void setPosition(double positionRadians);

  public void resetPosition();

  public void stop();

  public void setGains(double p, double i, double d, double v, double s, double g, double a);

  public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration);

  public void setBrakeMode(boolean brake);
 
  public double getPosition();
}
