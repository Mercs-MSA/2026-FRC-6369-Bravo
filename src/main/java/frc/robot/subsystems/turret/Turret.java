package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive.Drive;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  public enum TurretGoalState {
    HOME,
    FIXED,
    POSE,
  }

  public TurretGoalState currentState = TurretGoalState.HOME;
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private Pose2d goalPose = null;
  private double goalRadians = 0.0;

  private double driveRotationOffset = 0.0;
  private double previousDriveRotation = 0.0;

  private final Drive drive;

  public Turret(TurretIO io, Drive drive) {
    this.io = io;
    this.drive = drive;

    goalPose = new Pose2d();
    goalRadians = 0.0;
    driveRotationOffset = 0.0;
    previousDriveRotation = drive.getPose().getRotation().getRadians();

    io.setGains(
        TurretConstants.kTurretGains.p(),
        TurretConstants.kTurretGains.i(),
        TurretConstants.kTurretGains.d(),
        TurretConstants.kTurretGains.s(),
        TurretConstants.kTurretGains.g(),
        TurretConstants.kTurretGains.v(),
        TurretConstants.kTurretGains.a());

    io.setMotionMagicConstraints(
        TurretConstants.kTurretGains.maxVelocityDegPerSec(),
        TurretConstants.kTurretGains.maxAccelerationDegPerSec2());

  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Turret/Inputs", inputs);

    if (DriverStation.isDisabled()) {
      stop();
      return;
    }

    double driveRotation = drive.getPose().getRotation().getRadians();
    if (driveRotation - previousDriveRotation > Math.PI) {
      driveRotationOffset -= Math.PI * 2;
    } else if (driveRotation - previousDriveRotation < -Math.PI) {
      driveRotationOffset += Math.PI * 2;
    }
    previousDriveRotation = drive.getPose().getRotation().getRadians();

    io.setPosition(getTargettingAngle());
  }

  private Pose2d calculateTurretOffset(Pose2d drivePose) {
    var r = Math.sin(TurretConstants.kTurretOffsetY);
    return new Pose2d(drivePose.getX() + r * Math.cos(drivePose.getRotation().getRadians()), drivePose.getY() + r * Math.sin(drivePose.getRotation().getRadians()), drivePose.getRotation());
  }

  private double getTargettingAngle() {
    var state = currentState;

    if (state == TurretGoalState.HOME) {
      return TurretConstants.kHomeRadians;
    } else if (state == TurretGoalState.FIXED) {
      return goalRadians;
    } else {
      Pose2d turretPose = calculateTurretOffset(drive.getPose()); 
      return Math.atan2(goalPose.getY() - turretPose.getY(), goalPose.getX() - turretPose.getX()) - (drive.getPose().getRotation().getRadians() + driveRotationOffset);
    }
  }

  public void home() {
    this.goalRadians = TurretConstants.kHomeRadians;
    this.currentState = TurretGoalState.HOME;
  }

  public void setGoalRadians(double goal) {
    this.goalRadians = goal;
    this.currentState = TurretGoalState.FIXED;
  }

  public void setGoalPose(Pose2d pose) {
    this.goalPose = pose;
    this.currentState = TurretGoalState.POSE;
  }

  @AutoLogOutput(key = "Turret/TargetIndicator")
  public Pose2d getTargetIndicator() {
    return new Pose2d(calculateTurretOffset(drive.getPose()).getTranslation(), new Rotation2d(drive.getPose().getRotation().getRadians() + (getTargettingAngle())));
  }

  @AutoLogOutput(key = "Turret/TargetPoint")
  public Pose2d getTargetPoint() {
    return goalPose;
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Turret/State")
  public TurretGoalState getState() {
    return currentState;
  }

  @AutoLogOutput(key = "Turret/Goal")
  public double getSimGoal() {
    return getTargettingAngle();
  }

  public void setPivotState(TurretGoalState state) {
    this.currentState = state;
  }

  @AutoLogOutput(key = "Turret/AtGoal")
  public boolean atGoal() {
    return Math.abs(goalRadians - getRotations()) < TurretConstants.kToleranceRotations;
  }

  @AutoLogOutput(key = "Turret/Rotations")
  public double getRotations() {
    return inputs.positionRadians;
  }

  @AutoLogOutput(key = "Turret/GoalRotations")
  public double getGoalRadians() {
    return goalRadians;
  }
}
