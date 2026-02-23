package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterTest extends Command {
  private final Shooter shooter;
  private final DoubleSupplier minionSpeedSupplier; 
  private final DoubleSupplier train; 


  public ShooterTest(Shooter shooter, DoubleSupplier minionSpeedSupplier,DoubleSupplier train) {
    this.shooter = shooter;
    this.train = train;
    this.minionSpeedSupplier = minionSpeedSupplier;

    addRequirements(shooter);
  }
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    
    double Long = shooter.getlong();
    double speed = minionSpeedSupplier.getAsDouble();
    double trainspeed = train.getAsDouble();
    
    shooter.setPitchSpeed(speed/10); 
    
    if(trainspeed > 0.7){
      shooter.setTrainSpeed(-0.6);
      shooter.setIntaketrainSpeed(0.8);
    }else{
      shooter.setTrainSpeed(0);
      shooter.setIntaketrainSpeed(0);

    }

    

    SmartDashboard.putNumber("0212/long", shooter.getlong());
    SmartDashboard.putNumber("0212/RPM", shooter.getShooterRPM());
    SmartDashboard.putNumber("0212/pitchRot", shooter.getPitchPositionRot());


  }

  @Override
  public void end(boolean interrupted) {
    
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}