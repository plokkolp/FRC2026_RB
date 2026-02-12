package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterMaster extends Command {

  private final Shooter shooter;
  private final XboxController controller;


  // ===== Yaw 對位 =====
  private static final double kP = 0.01;
  private static final double kS = 0.0;
  private static final double kMaxOut = 0.25;
  private static final double kTolDeg = 1.0;
  private static final double kMaxValidTxDeg = 30.0;

  private static final double kAlphaDist = 0.35;     
  private static final double kHoldSec = 0.20;       
  private static final double kDtGuess = 0.02;       

  private double distFiltered = 0.0;
  private double lastValidDist = Double.NaN;
  private double timeSinceValid = 999.0;

  private static final double D0 = 1.66,  RPM0 = -2700.0, PITCH0 = -0.418+0.23;
  private static final double D1 = 2.126, RPM1 = -3100.0, PITCH1 = -0.56+0.23;
  private static final double D2 = 3.09,  RPM2 = -3600.0, PITCH2 = -0.99+0.23;

  public ShooterMaster(Shooter shooter,XboxController controller) {
    this.shooter = shooter;
    this.controller = controller;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    distFiltered = 0.0;
    lastValidDist = Double.NaN;
    timeSinceValid = 999.0;

    SmartDashboard.putBoolean("ShooterMaster/Active", true);
    SmartDashboard.putString("ShooterMaster/State", "INIT");
  }

  @Override
  public void execute() {

    boolean has = shooter.hasLLTarget();
    double dist = shooter.getlong(); 

    boolean distOk = has && Double.isFinite(dist) && dist > 0.1 && dist < 10.0;
    double trainspeed = controller.getRightTriggerAxis();
    
    if(trainspeed > 0.7){
      shooter.setTrainSpeed(-0.6);
      shooter.setSpeed(0.8);
    }else{
      shooter.setTrainSpeed(0);
      shooter.setSpeed(0);

    }

    if(trainspeed > 0.1){
    shooter.setShooterRPM(-3100);
    }else{
      shooter.stopShooter();
    }

    if (distOk) {
      distFiltered = (1.0 - kAlphaDist) * distFiltered + kAlphaDist * dist;
      lastValidDist = distFiltered;
      timeSinceValid = 0.0;
    } else {
      timeSinceValid += kDtGuess;
    }

    boolean canUseDist = Double.isFinite(lastValidDist) && timeSinceValid <= kHoldSec;

    double targetRPM = 0.0;
    double targetPitchRot = 0.0;

    if (canUseDist) {
      double dUse = lastValidDist;

      dUse = MathUtil.clamp(dUse, D0, D2);

      if (dUse <= D1) {
        double t = (dUse - D0) / (D1 - D0);
        targetRPM = lerp(RPM0, RPM1, t);
        targetPitchRot = lerp(PITCH0, PITCH1, t);
      } else {
        double t = (dUse - D1) / (D2 - D1);
        targetRPM = lerp(RPM1, RPM2, t);
        targetPitchRot = lerp(PITCH1, PITCH2, t);
      }
      shooter.setShooterRPM(targetRPM);
      shooter.setPitchPosition(targetPitchRot);
    } else {
    }

    SmartDashboard.putNumber("ShooterMaster/distRawM", dist);
    SmartDashboard.putNumber("ShooterMaster/distUseM", canUseDist ? lastValidDist : Double.NaN);
    SmartDashboard.putNumber("ShooterMaster/targetRPM", canUseDist ? targetRPM : 0.0);
    SmartDashboard.putNumber("ShooterMaster/targetPitchRot", canUseDist ? targetPitchRot : 0.0);
    SmartDashboard.putNumber("ShooterMaster/holdT", timeSinceValid);

    //Yaw
    if (!has) {
      shooter.setYawSpeed(0.0);
      SmartDashboard.putString("ShooterMaster/State", "NO_TARGET");
      return;
    }

    double tx = shooter.getLLTx();

    if (!Double.isFinite(tx) || Math.abs(tx) > kMaxValidTxDeg) {
      shooter.setYawSpeed(0.0);
      SmartDashboard.putString("ShooterMaster/State", "BAD_TX");
      SmartDashboard.putNumber("ShooterMaster/tx", tx);
      return;
    }

    if (Math.abs(tx) <= kTolDeg) {
      shooter.setYawSpeed(0.0);
      SmartDashboard.putString("ShooterMaster/State", "ON_TARGET");
      SmartDashboard.putNumber("ShooterMaster/tx", tx);
      SmartDashboard.putNumber("ShooterMaster/cmd", 0.0);
      return;
    }

    double cmd = -tx * kP;
    if (kS != 0.0) cmd += Math.copySign(kS, cmd);
    cmd = MathUtil.clamp(cmd, -kMaxOut, kMaxOut);

    shooter.setYawSpeed(cmd);

    SmartDashboard.putString("ShooterMaster/State", "TRACKING");
    SmartDashboard.putNumber("ShooterMaster/tx", tx);
    SmartDashboard.putNumber("ShooterMaster/cmd", cmd);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setYawSpeed(0.0);
    SmartDashboard.putBoolean("ShooterMaster/Active", false);
    SmartDashboard.putString("ShooterMaster/State", interrupted ? "INTERRUPTED" : "ENDED");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static double lerp(double a, double b, double t) {
    t = MathUtil.clamp(t, 0.0, 1.0);
    return a + (b - a) * t;
  }
}
