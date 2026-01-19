package frc.robot.subsystems.drive.Drive.Controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive.Drive;
import java.util.function.DoubleSupplier;

public class PointAndDriveController {

  private static final double DEADBAND = 0.1;
  private static final double kP = 20.0;

  private final PIDController headingController = new PIDController(kP, 0.0, 0.0);

  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  public PointAndDriveController() {
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void acceptJoystickInputs(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
  }

  public ChassisSpeeds calculate(Drive drive, Translation2d targetPoint) {

    Pose2d robotPose = drive.getPose();

    boolean isRed = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);

    Translation2d adjustedTarget =
        isRed
            ? new Translation2d(
                Constants.kFieldLengthMeters - targetPoint.getX(),
                Constants.kFieldWidthMeters - targetPoint.getY())
            : targetPoint;

    Translation2d linearVelocity =
        TeleopController.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());

    double vx = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
    double vy = linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();

    double dx = adjustedTarget.getX() - robotPose.getX();
    double dy = adjustedTarget.getY() - robotPose.getY();
    double desiredHeading = Math.atan2(dy, dx);

    double omega =
        headingController.calculate(robotPose.getRotation().getRadians(), desiredHeading);

    Rotation2d fieldRotation =
        isRed ? drive.getRotation().plus(Rotation2d.fromRadians(Math.PI)) : drive.getRotation();

    return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, fieldRotation);
  }

  public void reset() {
    headingController.reset();
  }
}
