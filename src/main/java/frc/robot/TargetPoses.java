package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TargetPoses {
  public static final Pose2d kReefCenter =
      new Pose2d(4.48249, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));
  public static final double kXNetLineMeters = 7.15;

  public static final Pose2d AL = new Pose2d(3.162, 4.222, Rotation2d.fromDegrees(0.0));

  public static double average(double a, double b) {
    return (a + b) / 2.0;
  }
}
