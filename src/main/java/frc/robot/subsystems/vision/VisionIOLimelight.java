package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware, adapted for the new Vision subsystem. */
public class VisionIOLimelight implements CameraIO {
  private final String name;
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    this.name = name;
    this.rotationSupplier = rotationSupplier;

    var table = NetworkTableInstance.getDefault().getTable(name);
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    inputs.camName = name;

    // Connection status: update if we’ve heard from the Limelight in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Default to no target this cycle
    inputs.hasTarget = false;
    inputs.hasBeenUpdated = false;
    inputs.numberOfTargets = 0;

    // Update orientation for MegaTag2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault().flush();

    // Process MegaTag1 observations
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length < 7) continue;

      inputs.hasTarget = true;
      inputs.hasBeenUpdated = true;
      inputs.numberOfTargets = (int) rawSample.value[7];
      inputs.latestTimestamp = rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3;

      // Robot pose estimate
      inputs.latestEstimatedRobotPose = parsePose(rawSample.value);

      // Ambiguity (only first tag if available)
      inputs.latestTagAmbiguities =
          new double[] {rawSample.value.length >= 18 ? rawSample.value[17] : 0.0};

      // Just store one transform per tag — Limelight doesn’t give explicit per-tag transforms here
      inputs.latestTagTransforms = new Transform3d[] {new Transform3d()};

      // Single tag ID if only one tag
      inputs.singleTagAprilTagID = (inputs.numberOfTargets == 1) ? (int) rawSample.value[11] : -1;
    }

    // Process MegaTag2 observations (ORB-SLAM fused)
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length < 7) continue;

      inputs.hasTarget = true;
      inputs.hasBeenUpdated = true;
      inputs.numberOfTargets = (int) rawSample.value[7];
      inputs.latestTimestamp = rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3;

      // Robot pose estimate
      inputs.latestEstimatedRobotPose = parsePose(rawSample.value);

      // Ambiguity = 0 for already-disambiguated multi-tag
      inputs.latestTagAmbiguities = new double[] {0.0};

      // Same placeholder — could be expanded if Limelight publishes per-tag transforms
      inputs.latestTagTransforms = new Transform3d[] {new Transform3d()};

      inputs.singleTagAprilTagID = (inputs.numberOfTargets == 1) ? (int) rawSample.value[11] : -1;
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
