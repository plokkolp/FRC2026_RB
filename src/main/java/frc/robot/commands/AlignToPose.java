package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToPose extends Command {

  private final CommandSwerveDrivetrain drivetrain;

  private static final Pose2d kTargetBlue =
      new Pose2d(14.65, 4.51, Rotation2d.fromDegrees(0.0));

  private final PIDController xPid = new PIDController(2, 0.0, 0.0);
  private final PIDController yPid = new PIDController(2, 0.0, 0.0);
  private final PIDController tPid = new PIDController(4.0, 0.0, 0.15);

  private static final double kMaxV_FAST = 0.4;
  private static final double kMaxOmega_FAST = 0.8;


  private static final double kMaxV_SLOW = 0.3;
  private static final double kMaxOmega_SLOW = 0.60;

  private static final double kTolXY = 0.03;
  private static final double kTolDeg = 1.0;

  private static final double kBackDistance = 0.50;

  private static final double kWaitAfterFast = 0.5; //0.5 秒

  private Pose2d targetField;
  private Pose2d phaseBackStartPose;

  private double waitStartTs;

  private enum Phase { ALIGN_FAST, WAIT, ALIGN_SLOW, BACK }
  private Phase phase = Phase.ALIGN_FAST;

  private final SwerveRequest.ApplyRobotSpeeds req =
      new SwerveRequest.ApplyRobotSpeeds();

  public AlignToPose(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    tPid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {

    boolean isRed =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    targetField = isRed ? kTargetBlue : kTargetBlue;

    xPid.reset();
    yPid.reset();
    tPid.reset();

    xPid.setSetpoint(targetField.getX());
    yPid.setSetpoint(targetField.getY());
    tPid.setSetpoint(targetField.getRotation().getRadians());

    phase = Phase.ALIGN_FAST;
    phaseBackStartPose = null;

  }

  @Override
  public void execute() {

    Pose2d cur = drivetrain.getPose();

    if (phase == Phase.ALIGN_FAST) {

      runAlign(cur, kMaxV_FAST, kMaxOmega_FAST);

      if (atTarget(cur)) {
        drivetrain.setControl(req.withSpeeds(new ChassisSpeeds())); // 停住
        waitStartTs = Timer.getFPGATimestamp();
        phase = Phase.WAIT;
        
      }
      return;
    }

    // =========================
    // WAIT 0.5 秒
    // =========================
    if (phase == Phase.WAIT) {

      drivetrain.setControl(req.withSpeeds(new ChassisSpeeds()));

      if (Timer.getFPGATimestamp() - waitStartTs > kWaitAfterFast) {

        xPid.reset();
        yPid.reset();
        tPid.reset();

        xPid.setSetpoint(targetField.getX());
        yPid.setSetpoint(targetField.getY());
        tPid.setSetpoint(targetField.getRotation().getRadians());

        phase = Phase.ALIGN_SLOW;
      }
      return;
    }

    // =========================
    // SLOW ALIGN
    // =========================
    if (phase == Phase.ALIGN_SLOW) {

      runAlign(cur, kMaxV_SLOW, kMaxOmega_SLOW);

      if (atTarget(cur)) {
        phase = Phase.BACK;
        phaseBackStartPose = cur;
      }
      return;
    }

    if (phase == Phase.BACK) {

      double traveled =
          cur.getTranslation().getDistance(
              phaseBackStartPose.getTranslation());

      if (traveled < kBackDistance) {
        drivetrain.setControl(
            req.withSpeeds(new ChassisSpeeds(-0.4, 0.0, 0.0)));
      } else {
        drivetrain.setControl(
            req.withSpeeds(new ChassisSpeeds()));
      }
    }
  }

  private void runAlign(Pose2d cur, double maxV, double maxOmega) {

    double vxField = -xPid.calculate(cur.getX());
    double vyField = -yPid.calculate(cur.getY());
    double om = -tPid.calculate(cur.getRotation().getRadians());

    vxField = MathUtil.clamp(vxField, -maxV, maxV);
    vyField = MathUtil.clamp(vyField, -maxV, maxV);
    om = MathUtil.clamp(om, -maxOmega, maxOmega);

    Rotation2d heading = drivetrain.getTeleopHeading();

    ChassisSpeeds robotRel =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            vxField, vyField, om, heading);

    drivetrain.setControl(req.withSpeeds(robotRel));
  }

  private boolean atTarget(Pose2d cur) {

    double dx = targetField.getX() - cur.getX();
    double dy = targetField.getY() - cur.getY();
    double dxy = Math.hypot(dx, dy);

    double ddeg =
        Math.abs(targetField.getRotation()
            .minus(cur.getRotation()).getDegrees());

    return dxy < kTolXY && ddeg < kTolDeg;
  }

  @Override
  public boolean isFinished() {
    if (phase != Phase.BACK) return false;

    Pose2d cur = drivetrain.getPose();
    double traveled =
        cur.getTranslation().getDistance(
            phaseBackStartPose.getTranslation());

    return traveled >= kBackDistance;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(req.withSpeeds(new ChassisSpeeds()));
  }
}