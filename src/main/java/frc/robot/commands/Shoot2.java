package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.ShooterLookup;

public class Shoot2 extends Command {

  private final Shooter shooter;
  private final XboxController controller;
  private final XboxController drive;

  private static final double kYawP = 0.01;
  private static final double kYawMaxOut = 0.3;
  private static final double kYawTolDeg = 1.0;
  private static final double kMaxValidTxDeg = 30.0;

  private static final double kTrainDuty = -0.7;
  private static final double kIntakeDuty = 0.85;
  private static final double kFireTrig = 0.3;

  private static final boolean kGateByRPM = false;
  private static final double kRpmTol = 120.0;
  private static final double kSpinupMinTime = 0.2;

  private double startTime;

  private double lastTargetRpm = 2200.0;
  private double lastTargetPitchRot = -0.45;

  private static final double kManualDeadband = 0.1;
  private static final double kManualScale = 0.06;   

  public Shoot2(Shooter shooter, XboxController controller,XboxController drive) {
    this.shooter = shooter;
    this.controller = controller;
    this.drive = drive;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {

    boolean lbHeld = controller.getLeftBumperButton();
    double rx = controller.getRightX(); 
    if (lbHeld) {

      double manualYawOut = 0.0;
      if (Math.abs(rx) > kManualDeadband) {
        manualYawOut = MathUtil.clamp(rx * kManualScale, -1.0, 1.0);
      }
      shooter.setYawSpeed(manualYawOut);

      SmartDashboard.putBoolean("Auto/manualOverride", true);
      SmartDashboard.putNumber("Auto/manual_rx", rx);
      SmartDashboard.putNumber("Auto/manualYawOut", manualYawOut);

    } else {

      SmartDashboard.putBoolean("Auto/manualOverride", false);

      boolean hasTarget = shooter.hasLLTarget();
      double dist = shooter.getlong();

      boolean distValid = Double.isFinite(dist) && dist > 0.05 && dist < 10.0;
      if (hasTarget && distValid) {
        ShooterLookup.Point sp = ShooterLookup.sample(dist);
        lastTargetRpm = sp.rpm;
        lastTargetPitchRot = sp.pitchRot;
      }

      shooter.setPitchPosition(lastTargetPitchRot);
      shooter.setShooterRPM(lastTargetRpm);

      if (hasTarget) {
        double tx = shooter.getLLTx();
        if (Double.isFinite(tx) && Math.abs(tx) <= kMaxValidTxDeg) {
          if (Math.abs(tx) > kYawTolDeg) {
            double yawCmd =  tx * kYawP;
            yawCmd = MathUtil.clamp(yawCmd, -kYawMaxOut, kYawMaxOut);
            shooter.setYawSpeed(yawCmd);
            SmartDashboard.putNumber("Auto/yawCmd", yawCmd);
          } else {
            shooter.setYawSpeed(0);
            SmartDashboard.putNumber("Auto/yawCmd", 0);
          }
        } else {
          shooter.setYawSpeed(0);
          SmartDashboard.putNumber("Auto/yawCmd", 0);
        }
        SmartDashboard.putNumber("Auto/tx", tx);
      } else {
        shooter.setYawSpeed(0);
        SmartDashboard.putNumber("Auto/tx", 999);
        SmartDashboard.putNumber("Auto/yawCmd", 0);
      }
    }


    boolean trigger = drive.getRightTriggerAxis() > 0.3;

    boolean rpmReady = Math.abs(shooter.getShooterRPM() - lastTargetRpm) <= kRpmTol;
    boolean timeReady = (Timer.getFPGATimestamp() - startTime) > kSpinupMinTime;

    boolean allowFeed = trigger;
    if (kGateByRPM) {
      allowFeed = trigger && rpmReady && timeReady;
    }

    if (allowFeed) {
      shooter.setTrainSpeed(kTrainDuty);
      shooter.setIntaketrainSpeed(kIntakeDuty);
    } else {
      shooter.setTrainSpeed(0);
      shooter.setIntaketrainSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setYawSpeed(0);
    shooter.setTrainSpeed(0);
    shooter.setIntaketrainSpeed(0);
    shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}