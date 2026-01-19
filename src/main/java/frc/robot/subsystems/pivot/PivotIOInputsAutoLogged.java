package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PivotIOInputsAutoLogged extends PivotIO.PivotIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IsMotorConnected", isMotorConnected);
    table.put("PositionDegrees", positionDegrees);
    table.put("velocityDegreesPerSec", velocityDegreesPerSec);
    table.put("appliedVolts", appliedVolts);
    table.put("SupplyCurrentAmps", supplyCurrentAmps);
    table.put("StatorCurrentAmps", statorCurrentAmps);
    table.put("TemperatureCelsius", temperatureCelsius);
  }

  @Override
  public void fromLog(LogTable table) {
    isMotorConnected = table.get("IsMotorConnected", isMotorConnected);
    positionDegrees = table.get("PositionDegrees", positionDegrees);
    velocityDegreesPerSec = table.get("velocityDegreesPerSec", velocityDegreesPerSec);
    appliedVolts = table.get("appliedVolts", appliedVolts);
    supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
    statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
    temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
  }

  public PivotIOInputsAutoLogged clone() {
    PivotIOInputsAutoLogged copy = new PivotIOInputsAutoLogged();
    copy.isMotorConnected = this.isMotorConnected;
    copy.positionDegrees = this.positionDegrees;
    copy.velocityDegreesPerSec = this.velocityDegreesPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.supplyCurrentAmps = this.supplyCurrentAmps;
    copy.statorCurrentAmps = this.statorCurrentAmps;
    copy.temperatureCelsius = this.temperatureCelsius;
    return copy;
  }
}
