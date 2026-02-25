// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter;

// public class ShooterWater extends Command {

//   private final Shooter shooter;

//   private static final double kP = 0.015;      
//   private static final double kD = 0.0025;      
//   private static final double kS = 0.00;       
//   private static final double kMaxOut = 0.30;   
//   private static final double kTolDeg = 0.5;    

//   private static final double kMaxValidTxDeg = 30.0; // 你原本的保護值
//   private static final double kMinDt = 1e-3;         // 避免 dt=0

//   private double lastTx = 0.0;
//   private double lastTimeSec = 0.0;

//   public ShooterWater(Shooter shooter) {
//     this.shooter = shooter;
//     addRequirements(shooter);
//   }

//   @Override
//   public void initialize() {
//     SmartDashboard.putBoolean("ShooterWater/Active", true);
//     SmartDashboard.putString("ShooterWater/State", "INIT");

//     lastTx = 0.0;
//     lastTimeSec = Timer.getFPGATimestamp();
//   }

//   @Override
//   public void execute() {

//     // 1) 沒目標就停
//     if (!shooter.hasLLTarget()) {
//       shooter.setYawSpeed(0.0);
//       SmartDashboard.putString("ShooterWater/State", "NO_TARGET");
//       return;
//     }

//     double tx = shooter.getLLTx();

//     if (!Double.isFinite(tx) || Math.abs(tx) > kMaxValidTxDeg) {
//       shooter.setYawSpeed(0.0);
//       SmartDashboard.putString("ShooterWater/State", "BAD_TX");
//       SmartDashboard.putNumber("ShooterWater/tx", tx);
//       return;
//     }

//     double now = Timer.getFPGATimestamp();
//     double dt = now - lastTimeSec;
//     if (!Double.isFinite(dt) || dt < kMinDt) dt = 0.02; // 保底 20ms

//     double txRate = (tx - lastTx) / dt;

//     if (Math.abs(tx) <= kTolDeg) {
//       shooter.setYawSpeed(0.0);
//       SmartDashboard.putString("ShooterWater/State", "ON_TARGET");

//       SmartDashboard.putNumber("ShooterWater/tx", tx);
//       SmartDashboard.putNumber("ShooterWater/txRate", txRate);
//       SmartDashboard.putNumber("ShooterWater/cmd", 0.0);

//       lastTx = tx;
//       lastTimeSec = now;
//       return;
//     }

//     double cmd = (-tx * kP) + (-txRate * kD);

//     if (Math.abs(cmd) > 1e-4) {
//       cmd += Math.copySign(kS, cmd);
//     }

//     cmd = MathUtil.clamp(cmd, -kMaxOut, kMaxOut);

//     shooter.setYawSpeed(cmd);

//     SmartDashboard.putString("ShooterWater/State", "TRACKING");
//     SmartDashboard.putNumber("ShooterWater/tx", tx);
//     SmartDashboard.putNumber("ShooterWater/txRate", txRate);
//     SmartDashboard.putNumber("ShooterWater/cmd", cmd);

//     // 10) 更新狀態
//     lastTx = tx;
//     lastTimeSec = now;
//   }

//   @Override
//   public void end(boolean interrupted) {
//     shooter.setYawSpeed(0.0);
//     SmartDashboard.putBoolean("ShooterWater/Active", false);
//     SmartDashboard.putString("ShooterWater/State", interrupted ? "INTERRUPTED" : "ENDED");
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
