package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TurretIOInputsAutoLogged extends TurretIO.TurretIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IsMotorConnected", isMotorConnected);
    table.put("PositionDegrees", positionRadians);
    table.put("velocityDegreesPerSec", velocityDegreesPerSec);
    table.put("appliedVolts", appliedVolts);
    table.put("SupplyCurrentAmps", supplyCurrentAmps);
    table.put("StatorCurrentAmps", statorCurrentAmps);
    table.put("TemperatureCelsius", temperatureCelsius);
  }

  @Override
  public void fromLog(LogTable table) {
    isMotorConnected = table.get("IsMotorConnected", isMotorConnected);
    positionRadians = table.get("PositionDegrees", positionRadians);
    velocityDegreesPerSec = table.get("velocityDegreesPerSec", velocityDegreesPerSec);
    appliedVolts = table.get("appliedVolts", appliedVolts);
    supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
    statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
    temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
  }

  public TurretIOInputsAutoLogged clone() {
    TurretIOInputsAutoLogged copy = new TurretIOInputsAutoLogged();
    copy.isMotorConnected = this.isMotorConnected;
    copy.positionRadians = this.positionRadians;
    copy.velocityDegreesPerSec = this.velocityDegreesPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.supplyCurrentAmps = this.supplyCurrentAmps;
    copy.statorCurrentAmps = this.statorCurrentAmps;
    copy.temperatureCelsius = this.temperatureCelsius;
    return copy;
  }
}
