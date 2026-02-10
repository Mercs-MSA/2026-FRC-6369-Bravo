package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight-right";
  public static String camera1Name = "limelight-left";
  public static String camera2Name = "limelight-front";
  public static String camera3Name = "limelight-back";

  // Basic filtering thresholds
  public static double maxAmbiguity = 20;
  public static double maxZError = 999; // changed

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  public static final Vector<N3> kSingleStdDevs =
      (RobotBase.isReal())
          ? VecBuilder.fill(0.274375, 0.274375, 5.0)
          : VecBuilder.fill(0.23, 0.23, 5.0);
  public static final Vector<N3> kMultiStdDevs =
      (RobotBase.isReal())
          ? VecBuilder.fill(0.23188, 0.23188, 5.0)
          : VecBuilder.fill(0.23, 0.23, 5.0);

  // tag ambiguity
  public static final double kAmbiguityThreshold =
      (RobotBase.isReal()) ? 0.2 : 1.0; // techinally the max ambiguity should be here
  public static final boolean KUseSingleTagTransform = false;

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0, // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
