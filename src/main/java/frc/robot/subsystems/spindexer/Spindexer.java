package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  public enum SpindexerState {
    STOP,
    RUNNING
  }

  private SpindexerState currentState = SpindexerState.STOP;

  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private double goalSpeedRPS = 0.0;

  private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);
  private final Debouncer atSpeedDebouncer = new Debouncer(0.1, DebounceType.kRising);

  public Spindexer(SpindexerIO io) {
    this.io = io;
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

    if (currentState == SpindexerState.RUNNING) {
      goalSpeedRPS = SpindexerConstants.kDefaultSpeedRPS;
    }

    if (currentState == SpindexerState.STOP) {
      goalSpeedRPS = 0.0;
    }

    io.setVelocity(goalSpeedRPS);
  }

  public void setIndexState(SpindexerState state) {
    this.currentState = state;
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Spindexer/AtSpeed")
  public boolean atSpeed() {
    double filteredVelocity = velocityFilter.calculate(inputs.velocityRotationsPerSec);
    return atSpeedDebouncer.calculate(Math.abs(filteredVelocity - goalSpeedRPS) < 0.25);  //TODO: tune; are we even using this?
  }

  @AutoLogOutput(key = "Spindexer/GoalSpeedRPS")
  public double getGoalSpeedRPS() {
    return goalSpeedRPS;
  }
}
