package frc.robot.util;

/**
 * Simple 3-state Kalman filter for chassis speeds (vx, vy, omega).
 * - State x = [vx, vy, omega]^T
 * - Process model: x_k = x_{k-1} + w (random walk)
 * - Measurement: z = x + v
 *
 */
public class ChassisSpeedsFilter {
    private final double[] x = new double[3];         // state vector
    private final double[][] P = new double[3][3];    // state covariance
    private final double[][] Q = new double[3][3];    // process noise covariance
    private final double[][] R = new double[3][3];    // measurement noise covariance

    /**
     * Create Kalman filter with default noise values.
     * Default P diagonal = 1e-1, Q diagonal = 1e-3, R diagonal = 1e-2.
     */
    public ChassisSpeedsFilter() {
        setIdentity(P, 1e-1);
        setIdentity(Q, 1e-3);
        setIdentity(R, 1e-2);
        // state initialized to zero by default
    }

    /**
     * Create Kalman filter with specified process and measurement noise (diagonal).
     * @param qDiag process noise for each state (applied to diagonal)
     * @param rDiag measurement noise for each state (applied to diagonal)
     */
    public ChassisSpeedsFilter(double qDiag, double rDiag) {
        setIdentity(P, 1e-1);
        setIdentity(Q, qDiag);
        setIdentity(R, rDiag);
    }

    /**
     * Predict step only (random walk: x_k = x_{k-1}, P = P + Q)
     */
    public void predict() {
        // P = P + Q
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                P[i][j] += Q[i][j];
            }
        }
    }

    /**
     * Update step using a new measurement (vx, vy, omega).
     * Performs standard Kalman update with H = I.
     */
    public void update(double measVx, double measVy, double measOmega) {
        double[] z = new double[] { measVx, measVy, measOmega };

        // S = P + R
        double[][] S = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                S[i][j] = P[i][j] + R[i][j];
            }
        }

        double[][] invS = invert3x3(S);
        // K = P * invS
        double[][] K = multiply3x3(P, invS);

        // y = z - x
        double[] y = new double[3];
        for (int i = 0; i < 3; ++i) y[i] = z[i] - x[i];

        // x = x + K * y
        double[] Ky = multiply3x3Vec(K, y);
        for (int i = 0; i < 3; ++i) x[i] += Ky[i];

        // P = (I - K) * P
        double[][] I_minus_K = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                I_minus_K[i][j] = -K[i][j];
            }
            I_minus_K[i][i] += 1.0;
        }
        double[][] newP = multiply3x3(I_minus_K, P);
        // copy back
        for (int i = 0; i < 3; ++i)
            System.arraycopy(newP[i], 0, P[i], 0, 3);
    }

    /**
     * Convenience: perform predict then update with measurement.
     */
    public void predictAndUpdate(double measVx, double measVy, double measOmega) {
        predict();
        update(measVx, measVy, measOmega);
    }

    /**
     * Set process and measurement noise (diagonal).
     */
    public void setNoise(double qDiag, double rDiag) {
        setIdentity(Q, qDiag);
        setIdentity(R, rDiag);
    }

    /**
     * Return current state {vx, vy, omega}.
     */
    public double[] getState() {
        return new double[] { x[0], x[1], x[2] };
    }

    /**
     * Return current covariance as a flattened 9-element array row-major (optional).
     */
    public double[] getCovarianceFlattened() {
        return new double[] {
            P[0][0], P[0][1], P[0][2],
            P[1][0], P[1][1], P[1][2],
            P[2][0], P[2][1], P[2][2]
        };
    }

    // ----- Helper matrix routines (3x3 only) -----

    private static void setIdentity(double[][] m, double diag) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) m[i][j] = 0.0;
            m[i][i] = diag;
        }
    }

    private static double[][] multiply3x3(double[][] a, double[][] b) {
        double[][] c = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double s = 0.0;
                for (int k = 0; k < 3; ++k) s += a[i][k] * b[k][j];
                c[i][j] = s;
            }
        }
        return c;
    }

    private static double[] multiply3x3Vec(double[][] a, double[] v) {
        double[] r = new double[3];
        for (int i = 0; i < 3; ++i) {
            double s = 0.0;
            for (int j = 0; j < 3; ++j) s += a[i][j] * v[j];
            r[i] = s;
        }
        return r;
    }

    // Analytic inverse for 3x3 matrix. If near-singular, add tiny regularization to diagonal.
    private static double[][] invert3x3(double[][] m) {
        double a = m[0][0], b = m[0][1], c = m[0][2];
        double d = m[1][0], e = m[1][1], f = m[1][2];
        double g = m[2][0], h = m[2][1], i = m[2][2];

        double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);

        double eps = 1e-12;
        if (Math.abs(det) < eps) {
            // regularize by adding small value to diagonal and recompute
            double[][] reg = new double[3][3];
            for (int rIdx = 0; rIdx < 3; ++rIdx) {
                for (int cIdx = 0; cIdx < 3; ++cIdx) reg[rIdx][cIdx] = m[rIdx][cIdx];
                reg[rIdx][rIdx] += eps;
            }
            return invert3x3(reg); // second call should succeed
        }

        double invDet = 1.0 / det;

        double[][] inv = new double[3][3];
        inv[0][0] =  (e*i - f*h) * invDet;
        inv[0][1] = -(b*i - c*h) * invDet;
        inv[0][2] =  (b*f - c*e) * invDet;

        inv[1][0] = -(d*i - f*g) * invDet;
        inv[1][1] =  (a*i - c*g) * invDet;
        inv[1][2] = -(a*f - c*d) * invDet;

        inv[2][0] =  (d*h - e*g) * invDet;
        inv[2][1] = -(a*h - b*g) * invDet;
        inv[2][2] =  (a*e - b*d) * invDet;

        return inv;
    }
}