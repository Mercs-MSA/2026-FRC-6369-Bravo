package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.math.ShooterMathProvider;
import frc.robot.subsystems.drive.Drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelState;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.Index.IndexState;
import frc.robot.subsystems.intake.Intake.IntakeFlywheelGoal;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.pivot.Pivot.PivotState;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.Spindexer.SpindexerState;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretGoalState;
import frc.robot.subsystems.intake.Intake;

public class TeleopStates {
    public enum TeleopMode {
      IDLE,
      SHOOT_WARMUP,
      SHOOT_ACTIVE,
      INTAKE,
      HOME
    }

    @AutoLogOutput(key = "State/TeleopState")
    public TeleopMode currentTeleopMode;

    public final Drive drive;
    public final Intake intake;
    public final Flywheel shooterFlywheels;
    public final Pivot shooterHood;
    public final Turret shooterTurret;
    public final Spindexer spindexer;
    public final Index index;

    public TeleopStates(Drive drive, Intake intake, Flywheel shooterFlywheels, Pivot shooterHood, Turret shooterTurret, Spindexer spindexer, Index index) {
        this.drive = drive;
        this.intake = intake;
        this.shooterFlywheels = shooterFlywheels;
        this.shooterHood = shooterHood;
        this.shooterTurret = shooterTurret;
        this.spindexer = spindexer;
        this.index = index;
        this.currentTeleopMode = TeleopMode.IDLE;
    }

    public void warmupShootMode() {
      intake.setIntakeGoal(IntakeGoal.kOut);
      intake.setFlywheelGoal(IntakeFlywheelGoal.kStop);
      shooterFlywheels.setFlywheelState(FlywheelState.PROVIDED);
      shooterTurret.setTurretState(TurretGoalState.PROVIDED);
      shooterHood.setGoal(PivotGoal.PROVIDED);
      shooterHood.setPivotState(PivotState.PROVIDED);
      this.currentTeleopMode = TeleopMode.SHOOT_WARMUP;
    }

    public void intakeMode() {
      intake.setIntakeGoal(IntakeGoal.kOut);
      intake.setFlywheelGoal(IntakeFlywheelGoal.kRunning);
      shooterFlywheels.setFlywheelState(FlywheelState.STOP);
      index.setIndexState(IndexState.STOP);
      spindexer.setIndexState(SpindexerState.STOP);
      shooterTurret.setTurretState(TurretGoalState.PROVIDED);
      shooterHood.setGoal(PivotGoal.PROVIDED);
      shooterHood.setPivotState(PivotState.PROVIDED);
      this.currentTeleopMode = TeleopMode.INTAKE;
      
    }
    
    public void idleMode() {
      intake.setIntakeGoal(IntakeGoal.kOut);
      intake.setFlywheelGoal(IntakeFlywheelGoal.kStop);
      shooterFlywheels.setFlywheelState(FlywheelState.STOP);
      shooterTurret.setTurretState(TurretGoalState.PROVIDED);
      shooterHood.setGoal(PivotGoal.PROVIDED);
      shooterHood.setPivotState(PivotState.PROVIDED);
      index.setIndexState(IndexState.STOP);
      spindexer.setIndexState(SpindexerState.STOP);
      this.currentTeleopMode = TeleopMode.IDLE;
    }

    public void homeMode() {
      intake.setIntakeGoal(IntakeGoal.kStow);
      intake.setFlywheelGoal(IntakeFlywheelGoal.kStop);
      shooterFlywheels.setFlywheelState(FlywheelState.STOP);
      shooterTurret.setTurretState(TurretGoalState.HOME);
      index.setIndexState(IndexState.STOP);
      spindexer.setIndexState(SpindexerState.STOP);
      shooterHood.setGoal(PivotGoal.STOW);
      shooterHood.setPivotState(PivotState.STOW);
      this.currentTeleopMode = TeleopMode.IDLE;
    }
}
