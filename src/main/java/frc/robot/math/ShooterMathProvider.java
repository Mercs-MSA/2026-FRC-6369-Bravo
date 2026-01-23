package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.math.SimulationResults;

public class ShooterMathProvider {
    public double shooterVelocityTarget;
    public double shooterHoodAngle;
    public double shooterTurretDelta;

    public static final Translation2d targetPosition = new Translation2d();

    public static final SimulationResults sim = new SimulationResults();

    public void update(ChassisSpeeds velocities, Pose2d turretPose) {
        final double target = 2.5; // TODO: replace placeholder, meters from target
        final double tanVel = 0.5; // TODO: replace placeholder, m/s tangential
        final double radVel = 1.0; // TODO: replace placeolder, m/s radial

        // TODO: implement
        System.out.printf("Update shooter sim: VelX: %f, VelY: %f, VelR: %f\n", velocities.vxMetersPerSecond, velocities.vyMetersPerSecond, velocities.omegaRadiansPerSecond);
    }
}
