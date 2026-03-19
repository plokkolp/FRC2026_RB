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
  private static final double kYawMaxOut = 0.1;
  private static final double kYawTolDeg = 1.0;
  private static final double kMaxValidTxDeg = 30.0;

  private static final double kNoTagMinCmdThreshold = 0.05; 
  private static final double kNoTagMinCmd = 0.1;          

  private static final double kPostFilterHoldSec = 0.10;    

  private static final double kTrainDuty = -0.7;
  private static final double kIntakeDuty = -0.7;
  private static final double kFireTrig = 0.3;

  private static final boolean kGateByRPM = false;
  private static final double kRpmTol = 120.0;
  private static final double kSpinupMinTime = 0.2;

  private static final double kManualDeadband = 0.1;
  private static final double kManualScale = 0.06;

  private double startTime;

  private double lastTargetRpm = 2200.0;
  private double lastTargetPitchRot = -0.45;

 private boolean prevHasTarget = false;          
  private boolean noTagState = false;             
  private double noTagStartTime = -0.8;          

  private double lastSeenYawCmd = 0.0;            
  private double postFilterHoldCmd = 0.0;         
  private double postFilterHoldStartTime = -0.2;  

  public Shoot2(Shooter shooter, XboxController controller, XboxController drive) {
    this.shooter = shooter;
    this.controller = controller;
    this.drive = drive;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();

    prevHasTarget = shooter.hasLLTarget();
    noTagState = false;
    noTagStartTime = -1.0;
    lastSeenYawCmd = 0.0;
    postFilterHoldCmd = 0.0;
    postFilterHoldStartTime = -1.0;
  }

  @Override
  public void execute() {

    boolean manualHeld = controller.getYButton();
    double rx = controller.getRightX();

    if (manualHeld) {
      double manualYawOut = 0.0;
      if (Math.abs(rx) > kManualDeadband) {
        manualYawOut = MathUtil.clamp(rx * kManualScale, -1.0, 1.0);
      }

      shooter.setYawSpeed(manualYawOut);

      noTagState = false;
      noTagStartTime = -1.0;
      postFilterHoldStartTime = -1.0;


    } else {
      SmartDashboard.putBoolean("Auto/manualOverride", false);

      boolean hasTarget = shooter.hasLLTarget();
      double tx = shooter.getLLTx();  
      double dist = shooter.getlong();

      boolean distValid = Double.isFinite(dist) && dist > 0.05 && dist < 10.0;
      if (hasTarget && distValid) {
        ShooterLookup.Point sp = ShooterLookup.sample(dist);
        lastTargetRpm = sp.rpm;
        lastTargetPitchRot = sp.pitchRot;
      }

      shooter.setPitchPosition(lastTargetPitchRot);
      shooter.setShooterRPM(lastTargetRpm);

      double yawCmd = 0.0;
      double now = Timer.getFPGATimestamp();

      if (hasTarget) {

        noTagState = false;
        noTagStartTime = -1.0;
        postFilterHoldStartTime = -1.0;
        postFilterHoldCmd = 0.0;

        if (Double.isFinite(tx) && Math.abs(tx) <= kMaxValidTxDeg) {
          if (Math.abs(tx) > kYawTolDeg) {
            yawCmd = tx * kYawP;
            yawCmd = MathUtil.clamp(yawCmd, -kYawMaxOut, kYawMaxOut);

            if (Math.abs(yawCmd) > 1e-6) {
              lastSeenYawCmd = -yawCmd;
            }
          } else {
            yawCmd = 0.0;
          }
        } else {
          yawCmd = 0.0;
        }

      } else {
        if (!noTagState) {
          noTagState = true;
          noTagStartTime = now;
          postFilterHoldStartTime = -1.0;
          postFilterHoldCmd = 0.0;
        }

        double noTagElapsed = now - noTagStartTime;

        boolean txLooksUsable = Double.isFinite(tx)
            && Math.abs(tx) <= kMaxValidTxDeg
            && Math.abs(tx) > 1e-6;

        if (txLooksUsable) {
          yawCmd = tx * kYawP;

         if (Math.abs(yawCmd) <= kNoTagMinCmdThreshold) {
  if (Math.abs(lastSeenYawCmd) > 1e-6) {
    yawCmd = Math.signum(lastSeenYawCmd) * kNoTagMinCmd;
  } else {
    yawCmd = 0.0;
  }
}
          yawCmd = MathUtil.clamp(yawCmd, -kYawMaxOut, kYawMaxOut);

          if (Math.abs(yawCmd) > 1e-6) {
            postFilterHoldCmd = yawCmd;
          }
        } else {
          if (postFilterHoldStartTime < 0.0) {
            postFilterHoldStartTime = now;

            if (Math.abs(postFilterHoldCmd) <= 1e-6 && Math.abs(lastSeenYawCmd) > 1e-6) {
              postFilterHoldCmd = Math.signum(lastSeenYawCmd) * kNoTagMinCmd;
            }
          }

          double postElapsed = now - postFilterHoldStartTime;

          if (postElapsed <= kPostFilterHoldSec) {
            yawCmd = postFilterHoldCmd;
          } else {
            yawCmd = 0.0;
          }
        }

        if (noTagElapsed > 1.0 && Math.abs(postFilterHoldCmd) <= 1e-6) {
          yawCmd = 0.0;
        }
      }

      shooter.setYawSpeed(yawCmd);

    }

    boolean trigger = drive.getRightTriggerAxis() > kFireTrig;

    boolean rpmReady = Math.abs(shooter.getShooterRPM() - lastTargetRpm) <= kRpmTol;
    boolean timeReady = (Timer.getFPGATimestamp() - startTime) > kSpinupMinTime;

    boolean allowFeed = trigger;
    if (kGateByRPM) {
      allowFeed = trigger && rpmReady && timeReady;
    }

    if (allowFeed) {
      shooter.setTrainSpeed(kTrainDuty);
      shooter.setIntaketrainSpeed(kIntakeDuty);
    } else if (drive.getLeftTriggerAxis() > 0.4) {
      shooter.setTrainSpeed(-kTrainDuty);
      shooter.setIntaketrainSpeed(-kIntakeDuty);
    } else {
      shooter.setTrainSpeed(0);
      shooter.setIntaketrainSpeed(0);
    }

    prevHasTarget = shooter.hasLLTarget();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setYawSpeed(0);
    shooter.setTrainSpeed(0);
    shooter.setIntaketrainSpeed(0);
    shooter.stopPitch();
    shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}