package frc.robot.math;
import java.nio.file.Files;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShooterMathProvider {
    public double shooterVelocityTarget;
    public double shooterHoodAngle;
    public double shooterTurretDelta;

    public static final Translation2d targetPosition = new Translation2d();

    public void update(ChassisSpeeds velocities, Pose2d turretPose) {

        // TODO: implement

        System.out.printf("Update shooter sim: VelX: %f, VelY: %f, VelR: %f\n", velocities.vxMetersPerSecond, velocities.vyMetersPerSecond, velocities.omegaRadiansPerSecond);
    }
}
