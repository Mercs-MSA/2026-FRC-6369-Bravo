package frc.robot.subsystems.index;
import org.littletonrobotics.junction.AutoLog;

public interface IndexIO {

  @AutoLog
  public class IndexIOInputs {
    public boolean isMotorConnected = false;
    public double positionRotations = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public void updateInputs(IndexIOInputs inputs);

  public void setVelocity(double velocityRotationsPerSec);

  public void stop();

  public void setGains(double p, double i, double d, double v, double s, double g, double a);

  public void setBrakeMode(boolean brake);
}
