package frc.robot.math;

import java.io.IOException;
import java.util.Arrays;
import java.util.NavigableMap;
import java.util.TreeMap;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShooterMathProvider {
    @AutoLogOutput
    public double shooterVelocityTarget;
    @AutoLogOutput
    public double shooterHoodAngle;
    @AutoLogOutput
    public double shooterTurretDelta;
    @AutoLogOutput
    public boolean hoodStow;
    @AutoLogOutput
    public double dist;
    @AutoLogOutput
    public double runTime;
    
    // position of hub opening on blue side
    public static final Translation2d targetPositionBlueSide = new Translation2d(4.625, 4.034);   // TODO: verify accuracy of position

    // stow
    public static final Rectangle2d[] stowEnablePositions = new Rectangle2d[]{
        new Rectangle2d(new Translation2d(3.986, 8.147), new Translation2d(5.5, 6.849)), 
        new Rectangle2d(new Translation2d(3.986, 1.282), new Translation2d(5.5, 0)),
        new Rectangle2d(new Translation2d(16.54-3.986, 8.147), new Translation2d(16.54-5.5, 6.849)), 
        new Rectangle2d(new Translation2d(16.54-3.986, 1.282), new Translation2d(16.54-5.5, 0))
    };
    public static final Rectangle2d[] stowDisablePositions = new Rectangle2d[]{
        new Rectangle2d(new Translation2d(3.886, 8.147), new Translation2d(5.7, 6.849)), 
        new Rectangle2d(new Translation2d(3.886, 1.282), new Translation2d(5.7, 0)),
        new Rectangle2d(new Translation2d(16.54-3.886, 8.147), new Translation2d(16.54-5.7, 6.849)), 
        new Rectangle2d(new Translation2d(16.54-3.886, 1.282), new Translation2d(16.54-5.7, 0))
    };

    private final NavigableMap<Double, Double[]> shotMapRPS = new TreeMap<>();

    public ShooterMathProvider() {
        shotMapRPS.put(1.83, new Double[]{50.0, 0.00});
        shotMapRPS.put(3.09, new Double[]{50.0, 0.95});
        shotMapRPS.put(3.64, new Double[]{50.0, 1.45});
        shotMapRPS.put(5.32, new Double[]{59.0, 1.65});
    }

    /**
     * Upper/lower bound search for a value in an array
     * @param value Input value
     * @param a Array to search
     * @return Two index positions. First is the upper bound index, second the lower bound index. May return two identical values if the exact input is in the array.
     */
    public static int[] searchInput(double value, double[] a) {
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

    public void update(ChassisSpeeds velocities, Pose2d turretPose) throws IOException {
        // start runtime stat
        var ta = Utils.getCurrentTimeSeconds();

        // Distance to target
        Translation2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? FlippingUtil.flipFieldPose(new Pose2d(targetPositionBlueSide, new Rotation2d())).getTranslation() : targetPositionBlueSide;
        dist = Math.sqrt(Math.pow(turretPose.getX() - target.getX(), 2) + Math.pow(turretPose.getY() - target.getY(), 2));

        // Search for upper/lower bound indices
        

        shooterTurretDelta = 0.0;
        // Safely get lower/upper map entries with fallbacks to first/last entries when out-of-range
        var lowerEntry = shotMapRPS.floorEntry(dist);
        if (lowerEntry == null) {
            lowerEntry = shotMapRPS.firstEntry();
        }
        var upperEntry = shotMapRPS.ceilingEntry(dist);
        if (upperEntry == null) {
            upperEntry = shotMapRPS.lastEntry();
        }

        double lowerKey = lowerEntry.getKey();
        double upperKey = upperEntry.getKey();
        Double[] lowerVal = lowerEntry.getValue();
        Double[] upperVal = upperEntry.getValue();

        shooterVelocityTarget = lerp(dist, lowerKey, upperKey, lowerVal[0], upperVal[0]);
        shooterHoodAngle = lerp(dist, lowerKey, upperKey, lowerVal[1], upperVal[1]);

        boolean stowEnable = false;
        for (Rectangle2d rect : stowEnablePositions) {
            if (rect.contains(turretPose.getTranslation())) {
                stowEnable = true;
                break;
            }
        }
        if (hoodStow && !stowEnable) {
            for (Rectangle2d rect : stowDisablePositions) {
                if (rect.contains(turretPose.getTranslation())) {
                    stowEnable = false;
                    break;
                }
            }
        }

        if (stowEnable && !hoodStow) {
            hoodStow = true;
        } else if (!stowEnable && hoodStow) {
            hoodStow = false;
        }

        // update runtime stat
        var tb = Utils.getCurrentTimeSeconds();
        runTime = tb-ta;
    }
}
