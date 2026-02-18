// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** List of position setpoints for the Intake in meters */
  public enum IntakeGoal {
    kOut(() -> -1.68),
    kStow(() -> 0.0);
    /** Custom setpoint that can be modified over network tables; Usefu for debugging */
    private DoubleSupplier goal;

    IntakeGoal(DoubleSupplier goalMeters) {
      this.goal = goalMeters;
    }

    public double getGoalRadians() {
      return this.goal.getAsDouble();
    }
  }

  public enum IntakeFlywheelGoal {
    kStop(() -> 0),
    kRunning(() -> 8);

    private DoubleSupplier goalRps;

    IntakeFlywheelGoal(DoubleSupplier goalRps) {
      this.goalRps = goalRps;
    }

    public double getGoalRps() {
      return this.goalRps.getAsDouble();
    }
  }

  private final IntakeIO kIntake;
  private final IntakeIOInputsAutoLogged kInputsIntake = new IntakeIOInputsAutoLogged();

  private final IntakeFlywheelIO kIntakeFlywheel;
  private final IntakeFlywheelIOInputsAutoLogged kInputsIntakeFlywheel = new IntakeFlywheelIOInputsAutoLogged();

  private IntakeGoal currentIntakeGoal = IntakeGoal.kStow;
  private IntakeFlywheelGoal currentFlywheelGoal = IntakeFlywheelGoal.kStop;

  public Intake(IntakeIO io, IntakeFlywheelIO flywheelIO) {
    kIntake = io;
    kIntakeFlywheel = flywheelIO;
  }

  @Override
  public void periodic() {
    kIntake.updateInputs(kInputsIntake);
    kIntakeFlywheel.updateInputs(kInputsIntakeFlywheel);
    Logger.recordOutput("Intake/PositionGoal", currentIntakeGoal);
    Logger.recordOutput("Intake/FlywheelGoal", currentFlywheelGoal);
    Logger.recordOutput("Intake/Actual", getPosition());
    Logger.recordOutput("Intake/AtGoal", positionAtGoal());
    Logger.processInputs("Intake/Inputs", kInputsIntake);


    if (currentIntakeGoal != null) {
      kIntake.setPosition(currentIntakeGoal.getGoalRadians());

      Logger.processInputs("Intake/Inputs", kInputsIntake);
    } else {
      Logger.recordOutput("Intake/PositionGoal", "NONE");
    }

    if (currentFlywheelGoal != null) {
      kIntakeFlywheel.setVelocity(currentFlywheelGoal.getGoalRps());

      Logger.processInputs("Intake/FlywheelInputs", kInputsIntake);
    } else {
      Logger.recordOutput("Intake/FlywheelGoal", "NONE");
    }
  }

  /**
   * Sets the position goal of the mechanism, logic runs in subsystem periodic method
   *
   * @param desiredGoal The desired position goal
   */
  public void setIntakeGoal(IntakeGoal desiredGoal) {
    currentIntakeGoal = desiredGoal;
  }

  public void setFlywheelGoal(IntakeFlywheelGoal desiredGoal) {
    currentFlywheelGoal = desiredGoal;
  }

  /** Stops the mechanism */
  public void stop() {
    currentIntakeGoal = null;
    kIntake.stop();
  }

  /** Reset the mechanism's encoder to 0 */
  public void resetPosition() {
    kIntake.resetPosition();
  }

  public IntakeGoal getGoal() {
    return currentIntakeGoal;
  }


  /**
   * Compute the error based off of our current position and current goal
   *
   * @return The computed error in radians
   */
  @AutoLogOutput(key = "Intake/Feedback/PositionError")
  public double getPositionError() {
    return currentIntakeGoal.getGoalRadians() - getPosition();
  }

  /**
   * Compute the error based off of our current speed and current goal
   *
   * @return The computed error in rps
   */
  @AutoLogOutput(key = "Intake/Feedback/FlywheelError")
  public double getFlywheelError() {
    return currentIntakeGoal.getGoalRadians() - getPosition();
  }

  /**
   * @return If the Intake is at its desired goal yet
   */
  @AutoLogOutput(key = "Intake/Feedback/PositionAtGoal")
  public boolean positionAtGoal() {
    return Math.abs(getPositionError()) < IntakeConstants.kPositionToleranceMeters;
  }

  /**
   * @return If the Intake flywheels are at its desired goal yet
   */
  @AutoLogOutput(key = "Intake/Feedback/FlywheelsAtGoal")
  public boolean flywheelAtGoal() {
    return Math.abs(getFlywheelError()) < IntakeConstants.kFlywheelSpeedToleranceRps;
  }

  /**
   * @return The position of the linear mechanism in radians
   */
  public double getPosition() {
    return kIntake.getPosition();
  }
}
