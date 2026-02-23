package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class MJ extends Command {
  private final Shooter shooter;

  public MJ(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    shooter.setIntaketrainSpeed(-0.4);
    shooter.setShooterSpeed(-0.56); //6
    shooter.setPitchPosition(-1.6);
    shooter.setTrainSpeed(-0.6);
      }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterSpeed(0); 
    shooter.setTrainSpeed(0);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}