package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignTheta extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final double targetDeg;

  private final PIDController pid = new PIDController(4.0, 0.0, 0.15);

  private static final double kMaxOmega = 1.0;     // rad/s
  private static final double kTolDeg = 2.0;
  private static final double kTimeoutSec = 2.0;

  private static final boolean kInvertOmega = false; // 如果轉反方向改成 true

  private final SwerveRequest.ApplyRobotSpeeds req =
      new SwerveRequest.ApplyRobotSpeeds();

  private double startTs;

  public AlignTheta(CommandSwerveDrivetrain drivetrain, double targetDeg) {
    this.drivetrain = drivetrain;
    this.targetDeg = targetDeg;
    addRequirements(drivetrain);

    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    startTs = Timer.getFPGATimestamp();
    pid.reset();
    pid.setSetpoint(Math.toRadians(targetDeg));

    SmartDashboard.putString("AlignTheta/State", "INIT");
    SmartDashboard.putNumber("AlignTheta/TargetDeg", targetDeg);
  }

  @Override
  public void execute() {
    double curRad = drivetrain.getPose().getRotation().getRadians();

    double omega = pid.calculate(curRad);

    if (kInvertOmega) omega = -omega;

    omega = MathUtil.clamp(omega, -kMaxOmega, kMaxOmega);

    ChassisSpeeds robotRel =
        new ChassisSpeeds(0.0, 0.0, omega);

    drivetrain.setControl(req.withSpeeds(robotRel));

    SmartDashboard.putNumber("AlignTheta/CurDeg",
        drivetrain.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("AlignTheta/OmegaCmd", omega);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(req.withSpeeds(new ChassisSpeeds()));
  }

  @Override
  public boolean isFinished() {
    double errDeg =
        Math.abs(targetDeg -
            drivetrain.getPose().getRotation().getDegrees());

    boolean done = errDeg < kTolDeg;
    boolean timeout = (Timer.getFPGATimestamp() - startTs) > kTimeoutSec;

    return done || timeout;
  }
}