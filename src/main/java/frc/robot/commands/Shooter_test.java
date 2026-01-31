package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class Shooter_test extends InstantCommand {

  private final Shooter shooter;
  private double targetDeg;
  
  
    public Shooter_test(Shooter shooter,double targetDeg) {
      this.shooter = shooter;
      this.targetDeg = targetDeg;
      addRequirements(shooter);
    }
  
    @Override
    public void initialize() {
    shooter.setYawMotorPosRot(targetDeg);
  }
    @Override
  public void execute() {
    shooter.setYawMotorPosRot(targetDeg);
  }

   @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
