package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive.Drive;
import frc.robot.subsystems.drive.Drive.Drive.DriveState;
import java.util.Optional;

public class ChoreoAutonCommands {

  private final Drive drive;

  public ChoreoAutonCommands(Drive drive) {

    this.drive = drive;
  }

  public Command followChoreoPath(String pathName) {
    PathPlannerPath path = getTraj(pathName).get();
    path.getIdealTrajectory(Drive.PP_CONFIG);
    double totalTimeSeconds = path.getIdealTrajectory(Drive.PP_CONFIG).get().getTotalTimeSeconds();
    return drive
        .setDriveStateCommand(DriveState.AUTO)
        .andThen(drive.customFollowPathComamnd(path).withTimeout(totalTimeSeconds));
  }

  public Command resetToStartPose(String pathName) {
    PathPlannerPath path = getTraj(pathName).get();
    path.getIdealTrajectory(Drive.PP_CONFIG);
    Pose2d initialPose = path.getIdealTrajectory(Drive.PP_CONFIG).get().getInitialPose();
    return Commands.runOnce(
        () ->
            drive.setSuppliedPose(
                () -> {
                  if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                    return FlippingUtil.flipFieldPose(initialPose);
                  } else {
                    return initialPose;
                  }
                }));
  }

  public Optional<PathPlannerPath> getTraj(String pathName) {
    try {
      return Optional.of(PathPlannerPath.fromChoreoTrajectory(pathName));
    } catch (Exception e) {
      e.printStackTrace();
      return Optional.empty();
    }
  }
}
