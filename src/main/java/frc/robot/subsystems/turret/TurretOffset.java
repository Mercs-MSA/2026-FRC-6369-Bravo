package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;

public class TurretOffset {
    static public Pose2d calculateTurretOffset(Pose2d drivePose) {
    double r = drivePose.getRotation().getRadians();
    double x = TurretConstants.kTurretOffsetX * Math.cos(r) - TurretConstants.kTurretOffsetY * Math.sin(r);
    double y = TurretConstants.kTurretOffsetX * Math.sin(r) + TurretConstants.kTurretOffsetY * Math.cos(r);
    return new Pose2d(drivePose.getX() + x, drivePose.getY() + y, drivePose.getRotation());
  }
}
