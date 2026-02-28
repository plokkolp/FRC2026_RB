package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignX extends Command {

  public enum Mode { X, Y }

  private final CommandSwerveDrivetrain drivetrain;
  private final Mode mode;

  private final double target; // 目標 X 或 目標 Y（依 mode）
  private final PIDController pid = new PIDController(2.2, 0.0, 0.0);

  private static final double kMaxV = 0.5;
  private static final double kTol = 0.05;
  private static final double kTimeoutSec = 2.0;

  // 你已經量到 X 要反向，先做成參數
  private static final boolean kInvertX = true;
  private static final boolean kInvertY = false; // 先不要反，等你測完再決定

  private final SwerveRequest.ApplyRobotSpeeds req = new SwerveRequest.ApplyRobotSpeeds();
  private double startTs;

  public AlignX(CommandSwerveDrivetrain drivetrain, Mode mode, double target) {
    this.drivetrain = drivetrain;
    this.mode = mode;
    this.target = target;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    startTs = Timer.getFPGATimestamp();
    pid.reset();
    pid.setSetpoint(target);

    SmartDashboard.putString("Align1D/Mode", mode.name());
    SmartDashboard.putNumber("Align1D/Target", target);
  }

  @Override
  public void execute() {
    double cur = (mode == Mode.X) ? drivetrain.getPose().getX() : drivetrain.getPose().getY();

    double v = pid.calculate(cur);

    if (mode == Mode.X && kInvertX) v = -v;
    if (mode == Mode.Y && kInvertY) v = -v;

    v = MathUtil.clamp(v, -kMaxV, kMaxV);

    double vxField = (mode == Mode.X) ? v : 0.0;
    double vyField = (mode == Mode.Y) ? v : 0.0;

    ChassisSpeeds robotRel =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            vxField, vyField, 0.0, drivetrain.getFieldHeading());

    drivetrain.setControl(req.withSpeeds(robotRel));

    SmartDashboard.putNumber("Align1D/Cur", cur);
    SmartDashboard.putNumber("Align1D/Err", target - cur);
    SmartDashboard.putNumber("Align1D/Vcmd", v);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(req.withSpeeds(new ChassisSpeeds()));
  }

  @Override
  public boolean isFinished() {
    double cur = (mode == Mode.X) ? drivetrain.getPose().getX() : drivetrain.getPose().getY();
    boolean done = Math.abs(target - cur) < kTol;
    boolean timeout = (Timer.getFPGATimestamp() - startTs) > kTimeoutSec;
    return done || timeout;
  }
}