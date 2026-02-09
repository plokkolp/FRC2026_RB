package frc.robot.commands;

import java.util.function.DoubleSupplier; 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterTest extends Command {
  private final Shooter shooter;
  private final DoubleSupplier minionSpeedSupplier; 


  public ShooterTest(Shooter shooter, DoubleSupplier minionSpeedSupplier) {
    this.shooter = shooter;
    this.minionSpeedSupplier = minionSpeedSupplier;

    addRequirements(shooter);
  }
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {

    double speed = minionSpeedSupplier.getAsDouble();
    shooter.setPitchSpeed(speed/4); 
      // shooter. setTrainSpeed(-0.65); 
  }

  @Override
  public void end(boolean interrupted) {
    
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}