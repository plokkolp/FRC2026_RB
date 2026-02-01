package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class ShooterEasy extends InstantCommand {

  private final Shooter shooter;

  private static final double kP = 0.008;   
  private static final double kMaxYawSpeed = 0.02; 
  private static final double kDeadbandDeg = 0.5;  

  public ShooterEasy(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter); 
  }

  @Override
  public void execute() {

    if (!shooter.hasLLTarget()) {
      shooter.setYawSpeed(0.0);
      return;
    }

    double tx = shooter.getLLTx(); 

    if (Math.abs(tx) < kDeadbandDeg) {
      shooter.setYawSpeed(0.0);
      return;
    }

    double yawCmd = -tx * kP;

    yawCmd = Math.max(-kMaxYawSpeed, Math.min(kMaxYawSpeed, yawCmd));

    shooter.setYawSpeed(yawCmd);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setYawSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}
