package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.ShooterMathProvider;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  public enum PivotGoal {
    STOW(() -> 0.00),
    PROVIDED(() -> 0.0);

    private final DoubleSupplier goalDegrees;

    PivotGoal(DoubleSupplier goalDegrees) {
      this.goalDegrees = goalDegrees;
    }

    public double getGoalRadians() {
      return goalDegrees.getAsDouble();
    }
  }

  public enum PivotState {
    STOW,
    PROVIDED
  }

  public PivotState currentState = PivotState.STOW;

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private PivotGoal currentGoal = null;
  private double goalRotations = 0.0;

  private Translation2d targetPoint = new Translation2d();

  private final ShooterMathProvider math;

  public Pivot(PivotIO io, ShooterMathProvider math) {
    this.io = io;
    this.math = math;

    //Take  in math, assume the values in there are updated

    io.setGains(
        PivotConstants.kPivotGains.p(),
        PivotConstants.kPivotGains.i(),
        PivotConstants.kPivotGains.d(),
        PivotConstants.kPivotGains.s(),
        PivotConstants.kPivotGains.g(),
        PivotConstants.kPivotGains.v(),
        PivotConstants.kPivotGains.a());

    io.setMotionMagicConstraints(
        PivotConstants.kPivotGains.maxVelocityDegPerSec(),
        PivotConstants.kPivotGains.maxAccelerationDegPerSec2());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot/Inputs", inputs);

    if (DriverStation.isDisabled()) {
      stop();
      return;
    }

    if (currentGoal != null) {
      if (currentState != PivotState.PROVIDED) {
        goalRotations = currentGoal.getGoalRadians();
      } else {
        goalRotations = math.shooterHoodAngle;
      }
    }
    io.setPosition(math.hoodStow ? PivotGoal.STOW.getGoalRadians() : goalRotations);
}

  public void setGoal(PivotGoal goal) {
    this.currentGoal = goal;
  }

  @AutoLogOutput(key = "Pivot/targetPoint")
  public Translation2d getTargetPoint() {
    return targetPoint;
  }

  public void setAngle(double angleDeg) {
    goalRotations = angleDeg;
    currentState = PivotState.PROVIDED;
    setPositionRad(angleDeg);
  }

  public void stop() {
    currentGoal = null;
    io.stop();
  }

  public void setPositionRad(double angle) {
    io.setPosition(angle);
  }

  public void setPivotGoalWithState() {
    switch (currentState) {
      case STOW -> setGoal(PivotGoal.STOW);
      case PROVIDED -> setGoal(PivotGoal.PROVIDED);
    }
  }

  @AutoLogOutput(key = "Pivot/State")
  public PivotState getPivotState() {
    return currentState;
  }

  @AutoLogOutput(key = "Pivot/GoalDegrees")
  public double getSimGoalDeg() {
    return goalRotations;
  }

  public void setPivotState(PivotState state) {
    this.currentState = state;
  }

  // @AutoLogOutput(key = "Pivot/AtGoal")
  // public boolean atGoal() {
  //   return Math.abs(goalAngleRad - getAngleDeg()) < PivotConstants.kPositionToleranceRad;
  // }

  @AutoLogOutput(key = "Pivot/AngleDeg")
  public double getAngleDeg() {
    return inputs.positionDegrees;
  }

  @AutoLogOutput(key = "Pivot/GoalDeg")
  public double getGoalDeg() {
    return goalRotations;
  }
}
