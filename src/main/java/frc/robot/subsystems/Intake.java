package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ConsIntake;

public class Intake extends SubsystemBase {

  // ===================== Motors =====================
  private final TalonFX haveRolling  = new TalonFX(ConsIntake.HAVE_ROLLING_ID);
  // private final TalonFX trainRolling = new TalonFX(ConsIntake.TRAIN_ROLLING_ID);
  private final TalonFX haveAngle    = new TalonFX(ConsIntake.HAVE_ANGLE_ID);

  private final DutyCycleOut haveDuty  = new DutyCycleOut(0);
  private final DutyCycleOut trainDuty = new DutyCycleOut(0);
  private final DutyCycleOut angleDuty = new DutyCycleOut(0);

  private final MotionMagicVoltage angleMM = new MotionMagicVoltage(0);
  private final NeutralOut angleNeutral = new NeutralOut();


  public Intake() {
    haveRolling.getConfigurator().apply(ConsIntake.ROLLING_CONFIG);
    // trainRolling.getConfigurator().apply(ConsIntake.ROLLING_CONFIG);
    haveAngle.getConfigurator().apply(ConsIntake.ANGLE_CONFIG);
  }

  public void setRolling(double duty) {
    duty = MathUtil.clamp(duty, -1.0, 1.0);
    haveDuty.Output = duty;
    haveRolling.setControl(haveDuty);
  }

  public double getRollingRPS() {
    return haveRolling.getVelocity().getValueAsDouble();
  }

  // public void setTrainRolling(double duty) {
  //   duty = MathUtil.clamp(duty, -1.0, 1.0);
  //   trainDuty.Output = duty;
  //   // trainRolling.setControl(trainDuty);
  // }

  // public double getTrainRollingRPS() {
  //   return trainRolling.getVelocity().getValueAsDouble();
  // }


  public void setAnglePositionRot(double rot) {
    haveAngle.setControl(angleMM.withPosition(rot));
  }

  public void setAngleSpeed(double duty) {
    duty = MathUtil.clamp(duty, -1.0, 1.0);
    angleDuty.Output = duty;
    haveAngle.setControl(angleDuty);
  }

  public double getAnglePositionRot() {
    return haveAngle.getPosition().getValueAsDouble();
  }

  public double getAngleVelocityRPS() {
    return haveAngle.getVelocity().getValueAsDouble();
  }

  public void stopAngle() {
    haveAngle.stopMotor();
  }

  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/AnglePosRot", getAnglePositionRot());
    // SmartDashboard.putNumber("Intake/AngleVelRPS", getAngleVelocityRPS());

    // SmartDashboard.putNumber("Intake/HaveRollerRPS", getRollingRPS());
    // SmartDashboard.putNumber("Intake/TrainRollerRPS", getTrainRollingRPS());
  }
}
