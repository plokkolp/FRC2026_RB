package frc.robot.constants;

public class ConsPose {
      public enum Pose {
    L1(1, 1, 3,0.18,true,true),    
    L2(10.3, 1, 5,0.5,true,true),    
    L3(48, 1, 5,0.5,true,true),    
    Get_Coral(0.4, 1, 2,0.5,false,true), 
    
    L2A(48.5, 58.8, 3,0.25,true,false),   
    L3A(51.2, 16.2, 5,0.25,true,false),   
    floadA(0, 63, 5,0.2,true,false),
    Get_A(0,48,3,0,false,false),   
    Get_RA(51,48,3,0.3,true,false),   


    Put_railing(51.5, 50, 4, -0.2,true,false), 
    Put_pro(0, 47, 0, -0.2,true,false); 


    public final double pos_intakeAngle, pos_intakeRollinr, range,speed;
    public final boolean liftfirst,isCoral;
    Pose(double pos_intakeAngle, double pos_intakeRollinr, double range,double speed,boolean liftfirst,boolean isCoral) {
      this.pos_intakeAngle = pos_intakeAngle;
      this.pos_intakeRollinr = pos_intakeRollinr;
      this.range = range;
      this.speed = speed;
      this.liftfirst = liftfirst;
      this.isCoral = isCoral;

    }
  }
}