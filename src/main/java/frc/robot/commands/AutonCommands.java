package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelState;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.Index.IndexState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.pivot.Pivot.PivotState;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.Spindexer.SpindexerState;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretGoalState;

public class AutonCommands {

//    NamedCommands.registerCommands(new HashMap<String, Command>(){
//       {
// put("Start", Commands.runOnce(() -> {
//           System.out.println("start");
//           shooterFlywheels.setFlywheelState(FlywheelState.STOP);
//           shooterHood.setPivotState(PivotState.PROVIDED);
//           shooterHood.setGoal(PivotGoal.PROVIDED);
//           shooterTurret.setTurretState(TurretGoalState.PROVIDED);
//           index.setIndexState(IndexState.STOP);
//           spindexer.setIndexState(SpindexerState.STOP);
//         }, shooterFlywheels, shooterHood, shooterTurret, index, spindexer));
//         put("StartFlywheels", Commands.runOnce(() -> {
//           shooterFlywheels.setFlywheelState(FlywheelState.PROVIDED);
//         }, shooterFlywheels, shooterHood, shooterTurret, index, spindexer));
//         put("BeginIndex", Commands.runOnce(() -> {
//           index.setIndexState(IndexState.PROVIDED);
//           spindexer.setIndexState(SpindexerState.RUNNING);
//         }, index, spindexer));
//       }
//     });        

    public static Command startCommand(Flywheel shooterFlywheels, Pivot shooterHood, Turret shooterTurret, Index index, Spindexer spindexer) {
      return Commands.runOnce(() -> {
        System.out.println("start");
        shooterFlywheels.setFlywheelState(FlywheelState.STOP);
        shooterHood.setPivotState(PivotState.PROVIDED);
        shooterHood.setGoal(PivotGoal.PROVIDED);
        shooterTurret.setTurretState(TurretGoalState.PROVIDED);
        index.setIndexState(IndexState.STOP);
        spindexer.setIndexState(SpindexerState.STOP);
      }, shooterFlywheels, shooterHood, shooterTurret, index, spindexer);
    }
    public static Command startFlywheelsCommand(Flywheel shooterFlywheels, Pivot shooterHood, Turret shooterTurret, Index index, Spindexer spindexer) {
      return Commands.runOnce(() -> {
        shooterFlywheels.setFlywheelState(FlywheelState.PROVIDED);
      }, shooterFlywheels, shooterHood, shooterTurret, index, spindexer);
    }
    public static Command beginIndexCommand(Flywheel shooterFlywheels, Pivot shooterHood, Turret shooterTurret, Index index, Spindexer spindexer) {
      return Commands.runOnce(() -> {
        index.setIndexState(IndexState.PROVIDED);
        spindexer.setIndexState(SpindexerState.RUNNING);
      }, index, spindexer);

}}
   