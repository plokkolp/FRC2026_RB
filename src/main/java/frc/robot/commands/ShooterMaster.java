// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter;

// public class ShooterMaster extends Command {

//   private final Shooter shooter;

//   private static final double kP = 0.01;
//   private static final double kS = 0.0;
//   private static final double kMaxOut = 0.35;
//   private static final double kTolDeg = 1.0;
//   private static final double kMaxValidTxDeg = 30.0;
 

//   public ShooterMaster(Shooter shooter) {
//     this.shooter = shooter;
//     addRequirements(shooter);
//   }

//   @Override
//   public void initialize() {
//     SmartDashboard.putBoolean("ShooterMaster/Active", true);
//     SmartDashboard.putString("ShooterMaster/State", "INIT");
//   }

//   @Override
//   public void execute() {
//     if (!shooter.hasLLTarget()) {
//       shooter.setYawSpeed(0.0);
//       SmartDashboard.putString("ShooterMaster/State", "NO_TARGET");
//       return;
//     }

//     double tx = shooter.getLLTx();

//     if (!Double.isFinite(tx) || Math.abs(tx) > kMaxValidTxDeg) {
//       shooter.setYawSpeed(0.0);
//       SmartDashboard.putString("ShooterMaster/State", "BAD_TX");
//       SmartDashboard.putNumber("ShooterMaster/tx", tx);
//       return;
//     }

//     if (Math.abs(tx) <= kTolDeg) {
//       shooter.setYawSpeed(0.0);
//       SmartDashboard.putString("ShooterMaster/State", "ON_TARGET");
//       SmartDashboard.putNumber("ShooterMaster/tx", tx);
//       SmartDashboard.putNumber("ShooterMaster/cmd", 0.0);
//       return;
//     }

//     double cmd = -tx * kP;
//     cmd += Math.copySign(kS, cmd);
//     cmd = MathUtil.clamp(cmd, -kMaxOut, kMaxOut);

//     shooter.setYawSpeed(cmd);

//     SmartDashboard.putString("ShooterMaster/State", "TRACKING");
//     SmartDashboard.putNumber("ShooterMaster/tx", tx);
//     SmartDashboard.putNumber("ShooterMaster/cmd", cmd);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     shooter.setYawSpeed(0.0);
//     SmartDashboard.putBoolean("ShooterMaster/Active", false);
//     SmartDashboard.putString("ShooterMaster/State", interrupted ? "INTERRUPTED" : "ENDED");
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }