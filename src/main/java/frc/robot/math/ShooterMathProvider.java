package frc.robot.math;

import java.util.Arrays;

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

    // Modified from https://stackoverflow.com/a/30245398
    // Posted by David Soroko, modified by community. See post 'Timeline' for change history
    // Retrieved 2026-01-22, License - CC BY-SA 4.0
    private static int searchInput(double value, double[] a) {
        if (value <= a[0]) { return 0; }
        if (value >= a[a.length - 1]) { return a.length - 1; }

        int result = Arrays.binarySearch(a, value);
        if (result >= 0) { return result; }

        int insertionPoint = -result - 1;
        return (a[insertionPoint] - value) < (value - a[insertionPoint - 1]) ?
                insertionPoint : insertionPoint - 1;
    }

    public void update(ChassisSpeeds velocities, Pose2d turretPose) {
        double target = searchInput(2.5, sim.targets); // TODO: replace placeholder, meters from target
        double tanVel = searchInput(0.1, sim.targets); // TODO: replace placeholder, m/s tangential
        double radVel = searchInput(0.1, sim.targets); // TODO: replace placeolder, m/s radial

        // TODO: implement
        System.out.printf("Update shooter sim: VelX: %f, VelY: %f, VelR: %f\n", velocities.vxMetersPerSecond, velocities.vyMetersPerSecond, velocities.omegaRadiansPerSecond);
    }
}
