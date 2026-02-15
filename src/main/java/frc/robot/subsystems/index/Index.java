package frc.robot.subsystems.index;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.math.ShooterMathProvider;

public class Index extends SubsystemBase {
  public enum IndexState {
    STOP,
    PROVIDED
  }

  private IndexState currentState = IndexState.STOP;

  private final IndexIO io;
  private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

  private double goalSpeedRPS = 0.0;

  private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);
  private final Debouncer atSpeedDebouncer = new Debouncer(0.1, DebounceType.kRising);

  private final ShooterMathProvider math;

  public Index(IndexIO io, ShooterMathProvider math) {
    this.io = io;
    this.math = math;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Index/Inputs", inputs);

    if (DriverStation.isDisabled()) {
      stop();
      return;
    }

    if (currentState == null) {
      io.stop();
      return;
    }

    if (currentState == IndexState.PROVIDED) {
      goalSpeedRPS = IndexConstants.kSpeed;
    }

    if (currentState == IndexState.STOP) {
      goalSpeedRPS = 0.0;
    }

    io.setVelocity(Math.min(goalSpeedRPS, IndexConstants.kIndexVelocityLimitRPS));
  }

  public void setIndexState(IndexState state) {
    this.currentState = state;
  }

  public void setCustomSpeedRPS(double speedRPS) {
    goalSpeedRPS = speedRPS;
    currentState = IndexState.PROVIDED;
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Index/AtSpeed")
  public boolean atSpeed() {
    double filteredVelocity = velocityFilter.calculate(inputs.velocityRotationsPerSec);
    return atSpeedDebouncer.calculate(Math.abs(filteredVelocity - goalSpeedRPS) < 0.25);
  }

  @AutoLogOutput(key = "Index/GoalSpeedRPS")
  public double getGoalSpeedRPS() {
    return goalSpeedRPS;
  }
}
