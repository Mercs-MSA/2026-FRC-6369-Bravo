package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.math.ShooterMathProvider;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;

public class Flywheel extends SubsystemBase {
  public enum FlywheelState {
    STOP,
    FIXED,
    PROVIDED
  }

  public FlywheelState currentState = FlywheelState.STOP;

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private double goalSpeedRPS = 0.0;

  private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);
  private final Debouncer atSpeedDebouncer = new Debouncer(0.1, DebounceType.kRising);

  private final ShooterMathProvider math;

  public Flywheel(FlywheelIO io, ShooterMathProvider math) {
    this.io = io;
    this.math = math;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel/Inputs", inputs);

    if (DriverStation.isDisabled()) {
      stop();
      return;
    }

    if (currentState == null) {
      io.stop();
      return;
    }

    if (currentState == FlywheelState.PROVIDED) {
      goalSpeedRPS = math.shooterVelocityTarget;
    }


    if (currentState == FlywheelState.STOP) {
      goalSpeedRPS = 0.0;
    }

    io.setVelocity(goalSpeedRPS);
  }

  public void setFlywheelState(FlywheelState state) {
    this.currentState = state;
  }

  public void setCustomSpeedRPS(double speedRPS) {
    goalSpeedRPS = speedRPS;
    currentState = FlywheelState.PROVIDED;
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Flywheel/AtSpeed")
  public boolean atSpeed() {
    double filteredVelocity = flywheelVelocity();
    // System.out.println(filteredVelocity - goalSpeedRPS);
    return atSpeedDebouncer.calculate(Math.abs(filteredVelocity - goalSpeedRPS) < 10);  //TODO: tune, this is probably way too tight; might need to be closer to 2 during fast shooting
  }

  @AutoLogOutput(key = "Flywheel/SpeedRPS")
  public double flywheelVelocity() {
    return velocityFilter.calculate(inputs.velocityRotationsPerSec);
  }

  @AutoLogOutput(key = "Flywheel/GoalSpeedRPS")
  public double getGoalSpeedRPS() {
    return goalSpeedRPS;
  }
}
