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
    //public Translation2d targetTurretLocation;

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

    public void update(ChassisSpeeds velocities, Pose2d turretPose) 
    {  
        double robotRadialVelocity=velocities.vxMetersPerSecond; //replace with radial velocity in relation to target
        double robotTangentialVelocity=velocities.vyMetersPerSecond;//replace with tangential velocity in relation to target
        double targetXDistance=targetPosition.getX()-turretPose.getX(); // TODO: calculate actual target location
        int target = searchInput(targetXDistance, sim.targets); // TODO: replace placeholder, meters from target
        int tanVel = searchInput(robotTangentialVelocity, sim.tanVelocities); // TODO: replace placeholder, m/s tangential
        int radVel = searchInput(robotRadialVelocity, sim.radVelocities); // TODO: replace placeolder, m/s radial
        int calculationIndex = target * 1 + tanVel * sim.iterations + radVel * (sim.iterations * sim.iterations);
        shooterVelocityTarget = sim.calculations[calculationIndex][2];
        shooterHoodAngle = sim.calculations[calculationIndex][1];
        shooterTurretDelta = sim.calculations[calculationIndex][0];

        // TODO: implement
        System.out.printf("Update shooter sim: VelX: %f, VelY: %f, VelR: %f\n", velocities.vxMetersPerSecond, velocities.vyMetersPerSecond, velocities.omegaRadiansPerSecond);
    }
    public double getShooterVelocityTarget(ChassisSpeeds velocities, Pose2d turretPose) 
    {
        update(velocities, turretPose);
        return shooterVelocityTarget;
    }
    public double getShooterHoodAngle(ChassisSpeeds velocities, Pose2d turretPose) 
    {
        update(velocities, turretPose);
        return shooterHoodAngle;
    }
    public double getShooterTurretDelta(ChassisSpeeds velocities, Pose2d turretPose) 
    {
        update(velocities, turretPose);
        return shooterTurretDelta;
    }
}
