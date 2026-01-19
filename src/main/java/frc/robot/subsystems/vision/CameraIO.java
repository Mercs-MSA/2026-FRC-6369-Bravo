package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * CameraIO interface for vision cameras. Provides a standard structure for camera inputs so that
 * the Vision subsystem can process multiple cameras consistently.
 */
public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    /** Name of this camera (for logging & debugging). */
    public String camName = "";

    /** Whether the camera is connected and returning data. */
    public boolean connected = false;

    /** Whether the camera currently has a valid target. */
    public boolean hasTarget = false;

    /** Whether the camera’s data has been updated this cycle. */
    public boolean hasBeenUpdated = false;

    /** Latest robot pose estimated by this camera (in field space). */
    public Pose3d latestEstimatedRobotPose = new Pose3d();

    /** Timestamp for the latest observation. */
    public double latestTimestamp = 0.0;

    /** Array of per-tag transforms (camera → tag). */
    public Transform3d[] latestTagTransforms = new Transform3d[0];

    /** Ambiguity values for each detected tag. */
    public double[] latestTagAmbiguities = new double[0];

    /** AprilTag ID if a single tag was used. */
    public int singleTagAprilTagID = -1;

    /** Number of targets detected in the latest frame. */
    public int numberOfTargets = 0;

    /** Transform from camera to the primary AprilTag (when applicable). */
    public Transform3d cameraToApriltag = new Transform3d();

    /** Fixed transform from this camera to the robot reference frame. */
    public Transform3d cameraToRobot = new Transform3d();
  }

  /**
   * Update the inputs from this camera. This is called once per cycle by the Vision subsystem.
   *
   * @param inputs Input structure to populate
   */
  public default void updateInputs(CameraIOInputs inputs) {}
}
