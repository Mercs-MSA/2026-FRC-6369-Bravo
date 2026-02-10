package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IsMotorConnected", isMotorConnected);
    table.put("PositionMeters", position);
    table.put("VelocityMetersPerSec", velocityRadianssPerSec);
    table.put("appliedVolts", appliedVolts);
    table.put("SupplyCurrentAmps", supplyCurrentAmps);
    table.put("StatorCurrentAmps", statorCurrentAmps);
    table.put("TemperatureCelsius", temperatureCelsius);
  }

  @Override
  public void fromLog(LogTable table) {
    isMotorConnected = table.get("IsMotorConnected", isMotorConnected);
    position = table.get("PositionMeters", position);
    velocityRadianssPerSec = table.get("VelocityMetersPerSec", velocityRadianssPerSec);
    appliedVolts = table.get("appliedVolts", appliedVolts);
    supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
    statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
    temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.isMotorConnected = this.isMotorConnected;
    copy.position = this.position;
    copy.velocityRadianssPerSec = this.velocityRadianssPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.supplyCurrentAmps = this.supplyCurrentAmps;
    copy.statorCurrentAmps = this.statorCurrentAmps;
    copy.temperatureCelsius = this.temperatureCelsius;
    return copy;
  }
}
