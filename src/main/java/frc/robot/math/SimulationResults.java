package frc.robot.math;

// TODO: example data only
public class SimulationResults {
    public int iterations = 4;

    public double[] targets = {
        0.035,
        1.9566666667,
        3.8783333333,
        5.8,
    };

    public double[] tanVelocities = {
        -1.0,
        0.3333333333,
        1.6666666667,
        3.0
    };

    public double[] radVelocities = {
        -3.0,
        -1.0,
        1.0,
        3.0
    };

    public double[][] calculations = {
        {0.3163833147, 1.2498780002, 6.5978948821}, // azimuth, elevation, velocityMag (target: 1.9566666667, tanVel: -1.0, radVel: -3.0)
    };
}