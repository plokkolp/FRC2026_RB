// package frc.robot.commands;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter;

// public class ShooterEasy extends Command {

//   private final Shooter shooter;
//   private final XboxController controller;

//   private static final double kP = 0.0002;        
//   private static final double kS = 0.0004;        
//   private static final double kMaxYawSpeed = 0.25;
//   private static final double kDeadbandDeg = 0.5;

//   public ShooterEasy(Shooter shooter, XboxController controller) {
//     this.shooter = shooter;
//     this.controller = controller;
//     addRequirements(shooter);
//   }

//   @Override
//   public void initialize() {
//     SmartDashboard.putBoolean("ShooterEasy/Active", true);
//   }

//   @Override
//   public void execute() {
//     shooter.setShooterSpeed(-0.63);

//     if (controller.getRightTriggerAxis() > 0.5) {
//       shooter.setTrainSpeed(-0.5);
//     } else {
//       shooter.setTrainSpeed(0);
//     }

//     if (!shooter.hasLLTarget()) {
//       shooter.setYawSpeed(0.0);
//       SmartDashboard.putString("ShooterEasy/State", "NO_LL_TARGET");
//       return;
//     }

//     double errDeg = shooter.getBestGoalYawDeg();
//     if (!Double.isFinite(errDeg)) {
//       shooter.setYawSpeed(0.0);
//       SmartDashboard.putString("ShooterEasy/State", "NO_VALID_GOAL");
//       return;
//     }

//     if (Math.abs(errDeg) < kDeadbandDeg) {
//       shooter.setYawSpeed(0.0);
//       SmartDashboard.putString("ShooterEasy/State", "ALIGNED");
//       return;
//     }

//     double yawCmd = -errDeg * kP;
    

   
//     yawCmd += Math.copySign(kS, yawCmd);

//     yawCmd = Math.max(-kMaxYawSpeed, Math.min(kMaxYawSpeed, yawCmd));
//     shooter.setYawSpeed(yawCmd);

//     SmartDashboard.putString("ShooterEasy/State", "TRACKING");
//     SmartDashboard.putNumber("ShooterEasy/errDeg", errDeg);
//     SmartDashboard.putNumber("ShooterEasy/yawCmd", yawCmd);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     shooter.setYawSpeed(0.0);
//     shooter.setTrainSpeed(0.0);
//     SmartDashboard.putBoolean("ShooterEasy/Active", false);
//     SmartDashboard.putString("ShooterEasy/State", interrupted ? "INTERRUPTED" : "ENDED");
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterEasy extends Command {

  private final Shooter shooter;
  private final XboxController controller;

  private static final double kP = 0.014;         
  private static final double kTxDeadbandDeg = 0.4; 
  private static final double kMaxYawSpeed = 0.25;  


  private static final double kStickDeadband = 0.10; 
  private static final double kManualMaxOut = 0.10;  

  public ShooterEasy(Shooter shooter, XboxController controller) {
    this.shooter = shooter;
    this.controller = controller;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("ShooterEasy/Active", true);
  }

  @Override
  public void execute() {

    shooter.setShooterSpeed(-0.55);//-0.63
    if (controller.getRightTriggerAxis() > 0.5) {
      shooter.setSpeed(-0.2);
      shooter.setTrainSpeed(-0.5);
    } else {
      shooter.setSpeed(0);
      shooter.setTrainSpeed(0.0);
    }


    double stickX = -MathUtil.applyDeadband(controller.getRightX(), kStickDeadband);

    double yawCmd = 0.0;

    if (Math.abs(stickX) > 0.0) {

      yawCmd = MathUtil.clamp(stickX * kManualMaxOut, -kManualMaxOut, kManualMaxOut);
      SmartDashboard.putString("ShooterEasy/Mode", "MANUAL");
    } else {

      if (!shooter.hasLLTarget()) {
        yawCmd = 0.0;
        SmartDashboard.putString("ShooterEasy/Mode", "AUTO_NO_TAG");
      } else {
        double tx = shooter.getLLTx(); // degrees

        if (Math.abs(tx) < kTxDeadbandDeg) {
          yawCmd = 0.0;
          SmartDashboard.putString("ShooterEasy/Mode", "AUTO_ALIGNED");
        } else {
          yawCmd = -tx * kP;
          yawCmd = MathUtil.clamp(yawCmd, -kMaxYawSpeed, kMaxYawSpeed);
          SmartDashboard.putString("ShooterEasy/Mode", "AUTO_TRACK");
        }

        SmartDashboard.putNumber("ShooterEasy/tx", tx);
      }
    }

    shooter.setYawSpeed(yawCmd);

    // ===== Debug =====
    SmartDashboard.putNumber("ShooterEasy/stickX", stickX);
    SmartDashboard.putNumber("ShooterEasy/yawCmd", yawCmd);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setYawSpeed(0.0);
    shooter.setTrainSpeed(0.0);
    shooter.setShooterSpeed(0);//-0.63

    SmartDashboard.putBoolean("ShooterEasy/Active", false);
    SmartDashboard.putString("ShooterEasy/Mode", interrupted ? "INTERRUPTED" : "ENDED");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
