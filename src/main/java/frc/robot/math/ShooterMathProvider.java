package frc.robot.math;

import java.io.IOException;
import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.BinaryLoader;

public class ShooterMathProvider {
    @AutoLogOutput
    public double shooterVelocityTarget;
    @AutoLogOutput
    public double shooterHoodAngle;
    @AutoLogOutput
    public double shooterTurretDelta;
    @AutoLogOutput
    public double dist;
    @AutoLogOutput
    public double runTime;

    
    // TODO: testing
    @AutoLogOutput
    public double closeHoodAngle;

    // inputs for Kalman filtering
    public static final double robotMassKg = 12.67;
    public static final double robotMOI = 2.000; // moment of inertia

    // position of hub opening on blue side
    public static final Translation2d targetPositionBlueSide = new Translation2d(4.640, 4.070);   // TODO: verify accuracy of position

    // indexing info returned from simulation generator.py
    // https://github.com/Mercs-MSA/2026-FRC-6369-ShooterSimulation
    public static final SimulationResults sim = new SimulationResults();

    // loader for bulk binary data produced by simulation
    public static final BinaryLoader loader = new BinaryLoader();

    /**
     * Upper/lower bound search for a value in an array
     * @param value Input value
     * @param a Array to search
     * @return Two index positions. First is the upper bound index, second the lower bound index. May return two identical values if the exact input is in the array.
     */
    private static int[] searchInput(double value, double[] a) {
        // Modified from https://stackoverflow.com/a/30245398
        // Posted by David Soroko, modified by community. See post 'Timeline' for change history
        // Retrieved 2026-01-22, License - CC BY-SA 4.0

        if (value <= a[0]) { return new int[]{0, 0}; } // value is first in array? - return the first position
        if (value >= a[a.length - 1]) { return new int[]{a.length - 1, a.length - 1}; } // value is last in array? - return the last position

        int result = Arrays.binarySearch(a, value); // search the array for the value, returns (-(insertion point) - 1) is value not found
        if (result >= 0) { return new int[]{result, result}; } // If value is found, return exact index

        int insertionPoint = -result - 1; // get the insertion point
        return new int[]{insertionPoint, insertionPoint - 1}; // upper/lower
    }

    /**
     * Get the flat index of results in the simulation records.
     * @param radVel Index of the radial velocity. Must be a valid index in SimulationResults
     * @param tanVel Index of the tangential velocity. Must be a valid index in SimulationResults
     * @param target Index of the target distance. Must be a valid index in SimulationResults
     * @return Position in the flat array. 
     */
    private int getCalcIndex(int radVel, int tanVel, int target) {
        return radVel * 1 + tanVel * SimulationResults.iterations + target * (SimulationResults.iterations * SimulationResults.iterations);
    }

    /**
     * Linear interpolation
     * @param x Input value
     * @param x1 Lower bound of x
     * @param x2 Upper bound of x
     * @param q00 Value at x1
     * @param q01 Value at x2
     * @return Interpolated value at x
     */
    private static double lerp(double x, double x1, double x2, double q00, double q01) {
        if (x1 == x2) {
            return q00;
        }
        if (x == x1) {
            return q00;
        }
        if (x == x2) {
            return q01;
        }
        return ((x2 - x) / (x2 - x1)) * q00 + ((x - x1) / (x2 - x1)) * q01;
    }

    /**
     * Trilinear interpolation
     * @param x Input x value
     * @param y Input y value
     * @param z Input z value
     * @param q000 Value at (x1, y1, z1)
     * @param q001 Value at (x1, y1, z2)
     * @param q010 Value at (x1, y2, z1)
     * @param q011 Value at (x1, y2, z2)
     * @param q100 Value at (x2, y1, z1)
     * @param q101 Value at (x2, y1, z2)
     * @param q110 Value at (x2, y2, z1)
     * @param q111 Value at (x2, y2, z2)
     * @param x1 Lower bound of x
     * @param x2 Upper bound of x
     * @param y1 Lower bound of y
     * @param y2 Upper bound of y
     * @param z1 Lower bound of z
     * @param z2 Upper bound of z
     * @return Interpolated value at (x, y, z)
     */
    private static double triLerp(double x, double y, double z, float q000, float q001, float q010, float q011, float q100, float q101, float q110, float q111, double x1, double x2, double y1, double y2, double z1, double z2) {
        double x00 = lerp(x, x1, x2, q000, q100);
        double x10 = lerp(x, x1, x2, q010, q110);
        double x01 = lerp(x, x1, x2, q001, q101);
        double x11 = lerp(x, x1, x2, q011, q111);
        double r0 = lerp(y, y1, y2, x00, x01);
        double r1 = lerp(y, y1, y2, x10, x11);

        return lerp(z, z1, z2, r0, r1);
    }

    public void update(ChassisSpeeds velocities, Pose2d turretPose) throws IOException {
        // start runtime stat
        var ta = Utils.getCurrentTimeSeconds();

        // Distance to target
        Translation2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? FlippingUtil.flipFieldPose(new Pose2d(targetPositionBlueSide, new Rotation2d())).getTranslation() : targetPositionBlueSide;
        dist = Math.sqrt(Math.pow(turretPose.getX() - target.getX(), 2) + Math.pow(turretPose.getY() - target.getY(), 2));

        // Drivebase velocities (equal to turret velocities)
        var tanVel = velocities.vyMetersPerSecond;
        var radVel = velocities.vxMetersPerSecond;

        // Search for upper/lower bound indices
        int[] targetExtremesIndex = searchInput(dist, SimulationResults.targets);
        int[] tanVelExtremesIndex = searchInput(tanVel, SimulationResults.tanVelocities);
        int[] radVelExtremesIndex = searchInput(radVel, SimulationResults.radVelocities);

        closeHoodAngle = loader.readRecord(getCalcIndex(radVelExtremesIndex[0], tanVelExtremesIndex[0], targetExtremesIndex[0]))[1];

        // Read points for interpolation
        var calculation_q000 = loader.readRecord(getCalcIndex(radVelExtremesIndex[0], tanVelExtremesIndex[0], targetExtremesIndex[0]));
        var calculation_q001 = loader.readRecord(getCalcIndex(radVelExtremesIndex[0], tanVelExtremesIndex[0], targetExtremesIndex[1]));
        var calculation_q010 = loader.readRecord(getCalcIndex(radVelExtremesIndex[0], tanVelExtremesIndex[1], targetExtremesIndex[0]));
        var calculation_q011 = loader.readRecord(getCalcIndex(radVelExtremesIndex[0], tanVelExtremesIndex[1], targetExtremesIndex[1]));
        var calculation_q100 = loader.readRecord(getCalcIndex(radVelExtremesIndex[1], tanVelExtremesIndex[0], targetExtremesIndex[0]));
        var calculation_q101 = loader.readRecord(getCalcIndex(radVelExtremesIndex[1], tanVelExtremesIndex[0], targetExtremesIndex[1]));
        var calculation_q110 = loader.readRecord(getCalcIndex(radVelExtremesIndex[1], tanVelExtremesIndex[1], targetExtremesIndex[0]));
        var calculation_q111 = loader.readRecord(getCalcIndex(radVelExtremesIndex[1], tanVelExtremesIndex[1], targetExtremesIndex[1]));

        // Interpolate between points, velocity
        shooterVelocityTarget = convertShooterVelocity(triLerp(
            radVel,
            tanVel,
            dist,
            calculation_q000[0],
            calculation_q001[0],
            calculation_q010[0],
            calculation_q011[0],
            calculation_q100[0],
            calculation_q101[0],
            calculation_q110[0],
            calculation_q111[0],
            SimulationResults.radVelocities[radVelExtremesIndex[0]],
            SimulationResults.radVelocities[radVelExtremesIndex[1]],
            SimulationResults.tanVelocities[tanVelExtremesIndex[0]],
            SimulationResults.tanVelocities[tanVelExtremesIndex[1]],
            SimulationResults.targets[targetExtremesIndex[0]],
            SimulationResults.targets[targetExtremesIndex[1]]
        )); 
        // Interpolate between points, hood angle
        shooterHoodAngle = convertHoodPosition(triLerp(
            radVel,
            tanVel,
            dist,
            calculation_q000[1],
            calculation_q001[1],
            calculation_q010[1],
            calculation_q011[1],
            calculation_q100[1],
            calculation_q101[1],
            calculation_q110[1],
            calculation_q111[1],
            SimulationResults.radVelocities[radVelExtremesIndex[0]],
            SimulationResults.radVelocities[radVelExtremesIndex[1]],
            SimulationResults.tanVelocities[tanVelExtremesIndex[0]],
            SimulationResults.tanVelocities[tanVelExtremesIndex[1]],
            SimulationResults.targets[targetExtremesIndex[0]],
            SimulationResults.targets[targetExtremesIndex[1]]
        )); 
        // Interpolate between points, turret delta
        shooterTurretDelta = triLerp(
            radVel,
            tanVel,
            dist,
            calculation_q000[2],
            calculation_q001[2],
            calculation_q010[2],
            calculation_q011[2],
            calculation_q100[2],
            calculation_q101[2],
            calculation_q110[2],
            calculation_q111[2],
            SimulationResults.radVelocities[radVelExtremesIndex[0]],
            SimulationResults.radVelocities[radVelExtremesIndex[1]],
            SimulationResults.tanVelocities[tanVelExtremesIndex[0]],
            SimulationResults.tanVelocities[tanVelExtremesIndex[1]],
            SimulationResults.targets[targetExtremesIndex[0]],
            SimulationResults.targets[targetExtremesIndex[1]]
        ); 

        // update runtime stat
        var tb = Utils.getCurrentTimeSeconds();
        runTime = tb-ta;
    }

    private static final double FUEL_RADIUS = 0.075; // meters
    private static final double FLYWHEEL_RADIUS = 0.051; // meters
    private static final double FLYWHEEL_EFFICIENCY = 1.05; // percent

    private double convertShooterVelocity(double simExitVelocity) {
        return simExitVelocity * (FLYWHEEL_RADIUS + FUEL_RADIUS) / (FLYWHEEL_EFFICIENCY * FLYWHEEL_RADIUS * FLYWHEEL_RADIUS * 2 * Math.PI);
    }

    private static final double HOOD_ZERO_POSITION = 0.32; // rad

    private double convertHoodPosition(double input) {
        // System.out.println(input);
        return (Math.PI / 2) - input - HOOD_ZERO_POSITION;
    }
}
