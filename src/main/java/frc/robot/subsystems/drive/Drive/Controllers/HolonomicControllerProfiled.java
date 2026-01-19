package frc.robot.subsystems.drive.Drive.Controllers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class HolonomicControllerProfiled {

  // --- Hardcoded PID constants ---
  private static final double X_P = 3.5;
  private static final double X_I = 0.0;
  private static final double X_D = 0.0;
  private static final double X_IZONE = 0.0;
  private static final double X_IRANGE = 0.0;
  private static final double X_MAX_V = 3.0;
  private static final double X_MAX_A = 3.0;

  private static final double Y_P = 3.5;
  private static final double Y_I = 0.0;
  private static final double Y_D = 0.0;
  private static final double Y_IZONE = 0.0;
  private static final double Y_IRANGE = 0.0;
  private static final double Y_MAX_V = 3.0;
  private static final double Y_MAX_A = 3.0;

  private static final double OMEGA_P = 3.0;
  private static final double OMEGA_I = 0.0;
  private static final double OMEGA_D = 0.0;
  private static final double OMEGA_IZONE = 0.0;
  private static final double OMEGA_IRANGE = 0.0;
  private static final double OMEGA_MAX_V = 180.0; // deg/sec
  private static final double OMEGA_MAX_A = 180.0; // deg/sec^2

  // --- Feedforward constants ---
  private static final double X_S = 0.0;
  private static final double X_V = 0.5;
  private static final double Y_S = 0.0;
  private static final double Y_V = 0.5;
  private static final double OMEGA_S = 0.0;
  private static final double OMEGA_V = 0.5;

  // --- Tolerances ---
  private static final double X_TOLERANCE = 0.03;
  private static final double Y_TOLERANCE = 0.03;
  private static final double OMEGA_TOLERANCE = 3.0; // degrees

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController omegaController;

  private final SimpleMotorFeedforward xFeedforward;
  private final SimpleMotorFeedforward yFeedforward;
  private final SimpleMotorFeedforward omegaFeedforward;

  public HolonomicControllerProfiled() {
    xController = new ProfiledPIDController(X_P, X_I, X_D, new Constraints(X_MAX_V, X_MAX_A));
    xController.setIntegratorRange(-X_IRANGE, X_IRANGE);
    xController.setIZone(X_IZONE);
    xController.setTolerance(X_TOLERANCE);
    yController = new ProfiledPIDController(Y_P, Y_I, Y_D, new Constraints(Y_MAX_V, Y_MAX_A));
    yController.setIntegratorRange(-Y_IRANGE, Y_IRANGE);
    yController.setIZone(Y_IZONE);
    yController.setTolerance(Y_TOLERANCE);
    omegaController =
        new ProfiledPIDController(
            OMEGA_P, OMEGA_I, OMEGA_D, new Constraints(OMEGA_MAX_V, OMEGA_MAX_A));
    omegaController.enableContinuousInput(-180.0, 180.0);
    omegaController.setIntegratorRange(-OMEGA_IRANGE, OMEGA_IRANGE);
    omegaController.setIZone(OMEGA_IZONE);
    omegaController.setTolerance(OMEGA_TOLERANCE);

    xFeedforward = new SimpleMotorFeedforward(X_S, X_V);
    yFeedforward = new SimpleMotorFeedforward(Y_S, Y_V);
    omegaFeedforward = new SimpleMotorFeedforward(OMEGA_S, OMEGA_V);
  }

  public void reset(Pose2d robotPose) {
    reset(robotPose, new ChassisSpeeds());
  }

  public void reset(Pose2d robotPose, ChassisSpeeds robotChassisSpeeds) {
    xController.reset(new State(robotPose.getX(), robotChassisSpeeds.vxMetersPerSecond * 0.5));
    yController.reset(new State(robotPose.getY(), robotChassisSpeeds.vyMetersPerSecond * 0.5));
    omegaController.reset(
        new State(
            robotPose.getRotation().getDegrees(),
            Math.toDegrees(robotChassisSpeeds.omegaRadiansPerSecond * 0.5)));
  }

  public ChassisSpeeds calculate(Pose2d goalPose, Pose2d currentPose) {
    return calculate(goalPose, new ChassisSpeeds(), currentPose);
  }

  public ChassisSpeeds calculate(Pose2d goalPose, ChassisSpeeds goalSpeed, Pose2d currentPose) {
    double xSpeed =
        xController.calculate(
                currentPose.getX(), new State(goalPose.getX(), goalSpeed.vxMetersPerSecond))
            + xFeedforward.calculate(xController.getSetpoint().velocity);
    double ySpeed =
        yController.calculate(
                currentPose.getY(), new State(goalPose.getY(), goalSpeed.vyMetersPerSecond))
            + yFeedforward.calculate(yController.getSetpoint().velocity);
    double omegaSpeed =
        Math.toRadians(
            omegaController.calculate(
                    currentPose.getRotation().getDegrees(),
                    new State(
                        goalPose.getRotation().getDegrees(),
                        Math.toDegrees(goalSpeed.omegaRadiansPerSecond)))
                + omegaFeedforward.calculate(omegaController.getSetpoint().velocity));

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, omegaSpeed, currentPose.getRotation());
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
  }

  public Pose2d getPositionGoal() {
    return new Pose2d(
        new Translation2d(xController.getGoal().position, yController.getGoal().position),
        Rotation2d.fromDegrees(omegaController.getGoal().position));
  }

  public Pose2d getPositionSetpoint() {
    return new Pose2d(
        new Translation2d(xController.getSetpoint().position, yController.getSetpoint().position),
        Rotation2d.fromDegrees(omegaController.getSetpoint().position));
  }

  public ChassisSpeeds getVelocitySetpoint() {
    return new ChassisSpeeds(
        xController.getSetpoint().velocity,
        yController.getSetpoint().velocity,
        Math.toRadians(omegaController.getSetpoint().velocity));
  }
}
