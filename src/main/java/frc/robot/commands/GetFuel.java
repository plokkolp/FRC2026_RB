package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class GetFuel extends Command {

  private final Intake intake;
  private final XboxController driver;

  private static final double POS_DOWN = 13.2;
  private static final double POS_UP   = 0.0;

  private static final double TOLERANCE = 0.5;

   private static boolean down = false, end = false, rolling = false,unrolling = false;
   
     public GetFuel(Intake intake,XboxController driver) {
       this.intake = intake;
       this.driver = driver;
       addRequirements(intake);
     }
   
     @Override
     public void initialize() {
        down = false; end = false; unrolling = false;
       double currentPos = intake.getAnglePositionRot(); 
   
       if (Math.abs(currentPos - POS_DOWN) < TOLERANCE) {
         intake.setAnglePositionRot(POS_UP);
         rolling = false;
    }
    else {
      intake.setAnglePositionRot(POS_DOWN);
      rolling = true;
     
    }
  }

@Override
  public void execute() {
   

    if(rolling){
        intake.setRolling(1);
    }else{
         intake.setRolling(0);
    }
  

  if(rolling && driver.getRightTriggerAxis()  > 0.1){
        rolling = false;
        unrolling = true;
                intake.setRolling(-1);

  }else if(unrolling = driver.getRightTriggerAxis()  < 0.1){
        rolling = true;
        unrolling = false;
  }

  }
   @Override
  public void end(boolean interrupted) {
    intake.setRolling(0);
  }
  
  @Override
  public boolean isFinished() {
    return end;
  }
}
