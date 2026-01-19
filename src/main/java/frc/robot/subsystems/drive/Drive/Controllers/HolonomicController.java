package frc.robot.subsystems.drive.Drive.Controllers;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.AutoLogOutput;

public class HolonomicController {

  private static final double TRANSLATION_P = 2.5;
  private static final double TRANSLATION_I = 0.0;
  private static final double TRANSLATION_D = 0.3;

  private static final double ROTATION_P = 3.0;
  private static final double ROTATION_I = 0.0;
  private static final double ROTATION_D = 0.0;

  private static final double OMEGA_MAX_VEL = Math.toRadians(180.0);
  private static final double OMEGA_MAX_ACCEL = Math.toRadians(180.0);

  private static final double KS = 0.1;
  private static final double KV = 1.0;
  private static final double KA = 0;

  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;
  private final SimpleMotorFeedforward feedforward;
  private final HolonomicDriveController controller;

  private Pose2d goalPose = new Pose2d();

  public HolonomicController() {
    xController = new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D);
    yController = new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D);

    thetaController =
        new ProfiledPIDController(
            ROTATION_P,
            ROTATION_I,
            ROTATION_D,
            new TrapezoidProfile.Constraints(OMEGA_MAX_VEL, OMEGA_MAX_ACCEL));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    controller = new HolonomicDriveController(xController, yController, thetaController);
    feedforward = new SimpleMotorFeedforward(KS, KV, KA);
  }

  public void reset(Pose2d startPose, ChassisSpeeds currentVelocity) {
    goalPose = startPose;
    thetaController.reset(
        startPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond * 0);
  }

  public ChassisSpeeds calculate(
      Pose2d goalPose, Pose2d currentPose, ChassisSpeeds currentVelocity) {
    this.goalPose = goalPose;

    Trajectory.State desiredState = new Trajectory.State(0.0, 0.0, 0.0, goalPose, 0.0);

    ChassisSpeeds pidSpeeds =
        controller.calculate(currentPose, desiredState, goalPose.getRotation());

    double vxFF = feedforward.calculate(pidSpeeds.vxMetersPerSecond, 0.0);
    double vyFF = feedforward.calculate(pidSpeeds.vyMetersPerSecond, 0.0);

    return new ChassisSpeeds(
        pidSpeeds.vxMetersPerSecond + vxFF,
        pidSpeeds.vyMetersPerSecond + vyFF,
        pidSpeeds.omegaRadiansPerSecond);
  }

  @AutoLogOutput(key = "Drive/HolonomicController/AtGoal")
  public boolean atGoal(Pose2d currentPose) {
    double posError = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    double rotError =
        Math.abs(currentPose.getRotation().minus(goalPose.getRotation()).getDegrees());
    return posError < 0.03 && rotError < 3.0;
  }

  @AutoLogOutput(key = "Drive/HolonomicController/PositionGoal")
  public Pose2d getPositionGoal() {
    return goalPose;
  }

  @AutoLogOutput(key = "Drive/HolonomicController/PositionSetpoint")
  public Pose2d getPositionSetpoint(Pose2d currentPose) {
    return currentPose;
  }

  @AutoLogOutput(key = "Drive/HolonomicController/VelocitySetpoint")
  public ChassisSpeeds getVelocitySetpoint(Pose2d currentPose, ChassisSpeeds currentVelocity) {
    return calculate(goalPose, currentPose, currentVelocity);
  }
}
