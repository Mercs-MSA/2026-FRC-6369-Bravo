package frc.robot.math;

import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.math.SimulationResults;
import frc.robot.subsystems.drive.Drive.Drive;

public class ShooterMathProvider {
    @AutoLogOutput
    public double shooterVelocityTarget;
    @AutoLogOutput
    public double shooterHoodAngle;
    @AutoLogOutput
    public double shooterTurretDelta;

    public static final Translation2d targetPosition = new Translation2d(4.640, 4.070);

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
        int target = searchInput(Math.sqrt(Math.pow(turretPose.getX() - targetPosition.getX(), 2) + Math.pow(turretPose.getY() - targetPosition.getY(), 2)), SimulationResults.targets); // TODO: replace placeholder, meters from target
        int tanVel = searchInput(-1*velocities.vyMetersPerSecond, SimulationResults.tanVelocities); // TODO: replace placeholder, m/s tangential
        int radVel = searchInput(-1*velocities.vxMetersPerSecond, SimulationResults.radVelocities); // TODO: replace placeolder, m/s radial
        int calculationIndex = target * 1 + tanVel * SimulationResults.iterations + radVel * (SimulationResults.iterations * SimulationResults.iterations);

        var calculation = sim.calculations[calculationIndex];
        shooterVelocityTarget = calculation[0];
        shooterHoodAngle = calculation[1];
        shooterTurretDelta = calculation[2];
        // System.out.println(Arrays.toString(sim.calculations[calculationIndex]));
    }
}
