package frc.robot.subsystems.drive.Drive.Controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class GoalPoseChooser {
  public static enum FIELD_ELEMENT {
    element1,
    element2,
    element3,
    element4
  }

  public static enum TASK {
    TASK1,
    TASK2,
    TASK3,
    TASK4
  }

  // @AutoLogOutput(key="GoalPoseChooser/TASK")
  private static TASK task = TASK.TASK1;

  public static Pose2d getGoalPose(FIELD_ELEMENT element, Pose2d robotPose) {
    switch (element) {
      case element1:
        return robotPose;
      case element2:
        return robotPose;
      case element3:
        return robotPose;
      case element4:
        return getIntakePose(robotPose);
    }
    return new Pose2d();
  }

  public static Pose2d getIntakePose(Pose2d robotPose) {
    return getAllianceAdjustedTarget(robotPose);
  }

  private static Pose2d getAllianceAdjustedTarget(Pose2d targetPose) {
    boolean isRed =
        DriverStation.getAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Red)
            .orElse(false);

    if (isRed) {
      double fieldLength = Constants.kFieldLengthMeters;
      double fieldWidth = Constants.kFieldWidthMeters;

      return new Pose2d(
          fieldLength - targetPose.getX(),
          fieldWidth - targetPose.getY(),
          targetPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    } else {
      return targetPose;
    }
  }

  // public static Command setGoalCommand(Pose2d goalPose) {
  //     return Commands.runOnce(()-> customGoal = goalPose);
  // }

  public static Command setTaskCommand(TASK reefTASK) {
    return Commands.runOnce(() -> task = reefTASK);
  }

  public static void setTask(TASK reefTASK) {
    task = reefTASK;
  }

  // private static boolean inBetween(double min, double max, double val) {
  //     return (val > min) && (val < max);
  // }
}
