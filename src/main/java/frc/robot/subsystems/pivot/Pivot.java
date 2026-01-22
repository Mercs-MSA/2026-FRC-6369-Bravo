package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.ShooterMathProvider;
import frc.robot.subsystems.drive.Drive.Drive;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Pivot extends SubsystemBase {

  public enum PivotGoal {
    STOW(() -> 0.0),
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

  private double homingStartTime = 0.0;
  private static final double kHomingTimeoutSec = 2.0;

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private PivotGoal currentGoal = null;
  private double goalAngleRad = 0.0;

  private final LinearFilter homingCurrentFilter =
      LinearFilter.movingAverage(PivotConstants.kLinearFilterSampleCount);

  private final LoggedNetworkBoolean simHomeTrigger =
      new LoggedNetworkBoolean("Pivot/SimHasHomed", false);

  private boolean isHoming = false;
  private boolean hasHomed = false;

  private final Debouncer stowDebouncer = new Debouncer(0.05, DebounceType.kBoth);

  private boolean hasStowed = false;

  private final Drive drive;
  private Translation2d targetPoint = new Translation2d();

  private final ShooterMathProvider math;

  public Pivot(PivotIO io, Drive drive, ShooterMathProvider math) {
    this.io = io;
    this.drive = drive;
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

    if (!isHoming && currentGoal != null) {
      if (currentState != PivotState.PROVIDED) {
        goalAngleRad = currentGoal.getGoalRadians();
      }
    
    if (currentGoal == PivotGoal.PROVIDED) {
      goalAngleRad = math.shooterHoodAngle;
    }

      if (currentGoal != PivotGoal.STOW) {
        setPositionRad(goalAngleRad);
        hasStowed = false;
      } else {
        if (hasStowed) {
          io.setVoltage(0.0);
        } else {
          setPositionRad(goalAngleRad);
          hasStowed = stowDebouncer.calculate(atGoal());
        }
      }
    } else if (isHoming) {
      handleHoming();
    }
  }

  public void setGoal(PivotGoal goal) {
    this.currentGoal = goal;
  }

  @AutoLogOutput(key = "Pivot/targetPoint")
  public Translation2d getTargetPoint() {
    return targetPoint;
  }

  public void setAngle(double angleDeg) {
    goalAngleRad = angleDeg;
    currentState = PivotState.PROVIDED;
    setPositionRad(angleDeg);
  }

  public void stop() {
    currentGoal = null;
    io.stop();
  }

  public void setPositionRad(double angle) {
    angle =
        MathUtil.clamp(
            angle, PivotConstants.kMinPositionRad, PivotConstants.kMaxPositionRad);
    io.setPositionMM(Units.radiansToRotations(angle));
  }

  public void home() {
    isHoming = true;
    hasHomed = false;
    homingStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    setGoal(null);
  }

  private void handleHoming() {
    double elapsed = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - homingStartTime;
    if (elapsed > kHomingTimeoutSec) {
      io.stop();
      isHoming = false;
      hasHomed = false;
      DriverStation.reportWarning("Pivot homing timed out", false);
      return;
    }

    double avgCurrent = homingCurrentFilter.calculate(inputs.statorCurrentAmps);

    if (RobotBase.isReal()) {
      if (avgCurrent > PivotConstants.kAmpFilterThreshold) {
        io.resetPosition();
        stop();
        isHoming = false;
        hasHomed = true;
      } else {
        io.setVoltage(-2.0);
      }
    } else {
      if (simHomeTrigger.get()) {
        io.resetPosition();
        stop();
        isHoming = false;
        hasHomed = true;
      } else {
        io.setVoltage(-2.0);
      }
    }
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
    return goalAngleRad;
  }

  public void setPivotState(PivotState state) {
    this.currentState = state;
  }

  @AutoLogOutput(key = "Pivot/AtGoal")
  public boolean atGoal() {
    return Math.abs(goalAngleRad - getAngleDeg()) < PivotConstants.kPositionToleranceRad;
  }

  @AutoLogOutput(key = "Pivot/AngleDeg")
  public double getAngleDeg() {
    return inputs.positionDegrees;
  }

  @AutoLogOutput(key = "Pivot/GoalDeg")
  public double getGoalDeg() {
    return goalAngleRad;
  }
}
