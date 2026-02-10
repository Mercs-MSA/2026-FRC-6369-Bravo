package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.Drive.*;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem for handling AprilTag-based localization from multiple cameras. This class
 * processes per-camera observations (pose estimates, uncertainty, timestamps),
 */
public class Vision {
  private final CameraIO[] cameras;
  private final CameraIOInputsAutoLogged[] camerasData;

  private final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public Vision(CameraIO[] cameras) {
    Logger.recordOutput("Vision/UseSingleTagTransform", KUseSingleTagTransform);
    this.cameras = cameras;
    this.camerasData = new CameraIOInputsAutoLogged[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      camerasData[i] = new CameraIOInputsAutoLogged(); // new logging for each camera
    }
  }

  /** Updates all cameras each cycle. */
  public void periodic(Drive drivebase) {
    // update vision data from limeights
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(camerasData[i]);
      Logger.processInputs("Vision/" + camerasData[i].camName, camerasData[i]);
    }

    // apply vision measurements to drivebase with std devs
    VisionObservation[] observations = getVisionObservations();
    for (VisionObservation observation : observations) {
      if (observation.hasObserved())
        drivebase.addVisionMeasurement(
            observation.pose(), observation.timeStamp(), observation.stdDevs());

      Logger.recordOutput(observation.camName() + "/stdDevX", observation.stdDevs().get(0));
      Logger.recordOutput(observation.camName() + "/stdDevY", observation.stdDevs().get(1));
      Logger.recordOutput(observation.camName() + "/stdDevTheta", observation.stdDevs().get(2));
    }
  }

  /**
   * Builds a set of per-camera vision observations. Each observation includes whether data is
   * valid, the estimated pose, standard deviations (how much to trust the data), and a timestamp.
   */
  public VisionObservation[] getVisionObservations() {
    VisionObservation[] observations = new VisionObservation[cameras.length];

    for (int i = 0; i < camerasData.length; i++) {
      CameraIOInputsAutoLogged camData = camerasData[i];

      // Case: camera has a valid, updated target
      if (camData.hasTarget && camData.hasBeenUpdated) {
        observations[i] = processCameraData(camData);
      }
      // Case: no target or stale data
      else {
        observations[i] = invalidObservation(camData);
      }
    }
    return observations;
  }

  /** Processes a single camera’s data into a valid VisionObservation. */
  private VisionObservation processCameraData(CameraIOInputsAutoLogged camData) {
    double numberOfTargets = camData.numberOfTargets;
    double avgDistMeters = 0.0;

    // Calculate average distance from valid tags
    for (int r = 0; r < camData.latestTagTransforms.length; r++) {
      if (camData.latestTagTransforms[r] != null) {
        if (camData.latestTagAmbiguities[r] < kAmbiguityThreshold) {
          avgDistMeters += camData.latestTagTransforms[r].getTranslation().getNorm();
        } else {
          numberOfTargets -= 1; // discard ambiguous tags
        }
      }
    }

    // If no trustworthy tags remain return an invlid observation
    if (numberOfTargets <= 0) {
      return invalidObservation(camData);
    }

    avgDistMeters /= numberOfTargets;

    // Case: single far-away tag → not reliable
    if (numberOfTargets == 1
        && avgDistMeters
            > 3.5) { // tune this number depending on clarity of vision and the speed moving in the
      // game
      return invalidObservation(camData);
    }

    // Case: single close tag
    if (numberOfTargets == 1) {
      Pose2d singleTagPose = estimateSingleTagPose(camData);
      return new VisionObservation(
          true,
          singleTagPose,
          VisionConstants.kSingleStdDevs,
          camData.latestTimestamp,
          camData.camName);
    }

    // multiple tags
    return new VisionObservation(
        true,
        camData.latestEstimatedRobotPose.toPose2d(),
        VisionConstants.kMultiStdDevs,
        camData.latestTimestamp,
        camData.camName);
  }

  /** Builds an invalid observation */
  private VisionObservation invalidObservation(CameraIOInputsAutoLogged camData) {
    return new VisionObservation(
        false,
        new Pose2d(),
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE),
        camData.latestTimestamp,
        camData.camName);
  }

  /** Estimates robot pose from a single */
  private Pose2d estimateSingleTagPose(CameraIOInputsAutoLogged camData) {
    if (KUseSingleTagTransform) {
      return fieldLayout
          .getTagPose(camData.singleTagAprilTagID)
          .get()
          .toPose2d()
          .plus(
              new Transform2d(
                  camData.cameraToApriltag.getX(),
                  camData.cameraToApriltag.getY(),
                  camData.cameraToApriltag.getRotation().toRotation2d()))
          .plus(toTransform2d(camData.cameraToRobot.inverse()));
    } else {
      return camData.latestEstimatedRobotPose.toPose2d();
    }
  }

  private Transform2d toTransform2d(Transform3d transform) {
    return new Transform2d(
        transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
  }

  /** Log. */
  public void logVisionObservation(VisionObservation observation, String state) {
    Logger.recordOutput("Vision/Observation/" + observation.camName() + "/State", state);
    Logger.recordOutput(
        "Vision/Observation/" + observation.camName() + "/Timestamp", observation.timeStamp());
    Logger.recordOutput(
        "Vision/Observation/" + observation.camName() + "/Pose", observation.pose());
    Logger.recordOutput(
        "Vision/Observation/" + observation.camName() + "/hasObserved", observation.hasObserved());
    Logger.recordOutput(
        "Vision/Observation/" + observation.camName() + "/StdDevs", observation.stdDevs());
  }

  public record VisionObservation(
      boolean hasObserved, Pose2d pose, Vector<N3> stdDevs, double timeStamp, String camName) {}
}
