package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class ShooterEasy extends InstantCommand {

  private final Shooter shooter;
  private final XboxController controller; 


  private static final double kP = 0.008;   
  private static final double kMaxYawSpeed = 0.02; 
  private static final double kDeadbandDeg = 0.5;  

  public ShooterEasy(Shooter shooter,XboxController testJoy) {
    this.shooter = shooter;
    this.controller = testJoy;
    

    addRequirements(shooter); 
  }
  @Override
    public void initialize() {
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
