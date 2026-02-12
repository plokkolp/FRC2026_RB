package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ConsShooter;

public class Shooter extends SubsystemBase {

  private final TalonFX leftShooter  = new TalonFX(ConsShooter.LEFT_SHOOTER_ID);
  private final TalonFX rightShooter = new TalonFX(ConsShooter.RIGHT_SHOOTER_ID);

  private final TalonFX train = new TalonFX(ConsShooter.TRAIN_ID);

  private final TalonFX angleMotor = new TalonFX(ConsShooter.ANGLE_MOTOR_ID);   // Yaw 
  private final CANcoder angleCancoder = new CANcoder(ConsShooter.ANGLE_CANCODER_ID);

  private final TalonFX minionMotor = new TalonFX(ConsShooter.MINION_MOTOR_ID); // Pitch

  private final LL4 ll4 = new LL4("limelight-shoot");

 private final SparkFlex motor = new SparkFlex(51, MotorType.kBrushless);


  private final DutyCycleOut shooterDuty = new DutyCycleOut(0);
  private final DutyCycleOut trainDuty   = new DutyCycleOut(0);
  private final DutyCycleOut yawDuty     = new DutyCycleOut(0);
  private final DutyCycleOut pitchDuty   = new DutyCycleOut(0);

  private final VelocityVoltage shooterVel = new VelocityVoltage(0).withSlot(0);

  private final PositionVoltage yawPos   = new PositionVoltage(0).withSlot(0);
  private final PositionVoltage pitchPos = new PositionVoltage(0).withSlot(0);

  public Shooter() {

    leftShooter.getConfigurator().apply(ConsShooter.LEFT_SHOOTER_CONFIG);
    rightShooter.getConfigurator().apply(ConsShooter.RIGHT_SHOOTER_CONFIG);

    train.getConfigurator().apply(ConsShooter.TRAIN_CONFIG);

    angleCancoder.getConfigurator().apply(ConsShooter.ANGLE_CANCODER_CONFIG);
    angleMotor.getConfigurator().apply(ConsShooter.ANGLE_MOTOR_CONFIG);

    minionMotor.getConfigurator().apply(ConsShooter.MINION_MOTOR_CONFIG);
  }


  public void setShooterSpeed(double duty) {
    duty = MathUtil.clamp(duty, -1.0, 1.0);
    shooterDuty.Output = duty;
    leftShooter.setControl(shooterDuty);
    rightShooter.setControl(shooterDuty);
  }

  public void setShooterRPM(double rpm) {
    double rps = rpm / 60.0;
    shooterVel.Velocity = rps;
    leftShooter.setControl(shooterVel);
    rightShooter.setControl(shooterVel);
  }

  public void stopShooter() {
    leftShooter.stopMotor();
    rightShooter.stopMotor();
  }

  public double getLeftShooterRPS() {
    return leftShooter.getVelocity().getValueAsDouble();
  }

  public double getRightShooterRPS() {
    return rightShooter.getVelocity().getValueAsDouble();
  }

  public double getLeftShooterRPM() {
    return getLeftShooterRPS() * 60.0;
  }

  public double getRightShooterRPM() {
    return getRightShooterRPS() * 60.0;
  }

  public double getShooterRPM() {
    return (getLeftShooterRPM() + getRightShooterRPM()) / 2.0;
  }

  public double getLeftShooterPositionRot() {
    return leftShooter.getPosition().getValueAsDouble();
  }

  public double getRightShooterPositionRot() {
    return rightShooter.getPosition().getValueAsDouble();
  }


  public void setTrainSpeed(double duty) {
    duty = MathUtil.clamp(duty, -1.0, 1.0);
    trainDuty.Output = duty;
    train.setControl(trainDuty);
  }

public boolean hasSpeakerTag(int tagId) {
  return ll4.hasTag(tagId);
}

public boolean hasSpeakerTagAny() {
  return ll4.hasSpeakerTag();
}

// public double getGoalYawDegForTag(int tagId) {
//   return ll4.getGoalYawDeg(tagId);
// }

// public double getGoalDistanceMForTag(int tagId) {
//   return ll4.getGoalDistanceMeters(tagId);
// }
public double getBestGoalYawDeg() {
  return ll4.getBestGoalYawDeg();
}


  public void stopTrain() {
    train.stopMotor();
  }

  public double getTrainPositionRot() {
    return train.getPosition().getValueAsDouble();
  }


  public void setYawSpeed(double duty) {
    duty = MathUtil.clamp(duty, -1.0, 1.0);
    yawDuty.Output = duty;
    angleMotor.setControl(yawDuty);
  }

  public double getYawAbsRot() {
    return angleCancoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getYawCanRot() {
    return angleCancoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getYawCanNoOffsetRot() {
    return angleCancoder.getPosition().getValueAsDouble();
  }

  public void setYawAbsRot(double absRot) {
    angleMotor.setControl(yawPos.withPosition(absRot));
  }

  public double getYawMotorPositionRot() {
    return angleMotor.getPosition().getValueAsDouble();
  }

  public double getYawClosedLoopError() {
    return angleMotor.getClosedLoopError().getValueAsDouble();
  }

  public void setYawMotorPosRot(double targetMotorPosRot) {
    angleMotor.setControl(yawPos.withPosition(targetMotorPosRot));
  }

  public double getShooterAngle0to360Rot() {
    double rot = getYawMotorPositionRot();
    rot = rot - Math.floor(rot);
    if (rot < 0.0) rot += 1.0;
    return rot;
  }

  public double getShooterAngleRotContinuous() {
    return getYawMotorPositionRot();
  }

  public double getShooterAngle0to360Deg() {
    return getShooterAngle0to360Rot() * 360.0;
  }

  public double getShooterAngleDegContinuous() {
    return getShooterAngleRotContinuous() * 360.0;
  }


  public void setPitchSpeed(double duty) {
    duty = MathUtil.clamp(duty, -1.0, 1.0);
    pitchDuty.Output = duty;
    minionMotor.setControl(pitchDuty);
  }

  public double getPitchPositionRot() {
    return minionMotor.getPosition().getValueAsDouble();
  }

  public void setPitchPosition(double positionRot) {
    minionMotor.setControl(pitchPos.withPosition(positionRot));
  }

  public void stopPitch() {
    minionMotor.stopMotor();
  }

  public void stopAll() {
    stopShooter();
    stopTrain();
    stopPitch();
  }


  public boolean hasLLTarget() {
    return ll4.hasTarget();
  }

  public double getLLTx() {
    return ll4.getTX(); //degrees
  }

  // public double getlong(){
  //   return ll4.getGoalDistanceMeters(26);
  // }

  public void setSpeed(double speed) {
    motor.set(speed);
    }

  public double getlong(){
    return ll4.getBestTagDistanceMeters();
  }

  
  @Override
  public void periodic() {

    // SmartDashboard.putNumber("Shooter/LeftRPM", getLeftShooterRPM());
    // SmartDashboard.putNumber("Shooter/RightRPM", getRightShooterRPM());
    // SmartDashboard.putNumber("Shooter/AvgRPM", getShooterRPM());

    // SmartDashboard.putNumber("Shooter/LeftPosRot", getLeftShooterPositionRot());
    // SmartDashboard.putNumber("Shooter/RightPosRot", getRightShooterPositionRot());

    // SmartDashboard.putNumber("Train/PosRot", getTrainPositionRot());

    // SmartDashboard.putNumber("Yaw/CAN_AbsRot", getYawCanRot());
    // SmartDashboard.putNumber("Yaw/CAN_NoOffsetRot", getYawCanNoOffsetRot());
    // SmartDashboard.putNumber("Yaw/MotorPosRot", getYawMotorPositionRot());
    // SmartDashboard.putNumber("Yaw/ClosedLoopError", getYawClosedLoopError());

    // SmartDashboard.putNumber("Pitch/MotorPosRot", getPitchPositionRot());

    // SmartDashboard.putNumber("Yaw/ShooterAngle0-360", getShooterAngle0to360Rot());        // 0~1
    // SmartDashboard.putNumber("Yaw/ShooterAngle", getShooterAngleRotContinuous());         // rot
    // SmartDashboard.putNumber("Yaw/ShooterAngleDeg0-360", getShooterAngle0to360Deg());     // 0~360 deg
    // SmartDashboard.putNumber("Yaw/ShooterAngleDeg", getShooterAngleDegContinuous());      // continuous deg
//  SmartDashboard.putNumber("LONG/LONG", getlong());
    // SmartDashboard.putBoolean("LL/HasTarget", hasLLTarget());
    // SmartDashboard.putNumber("LL/tx", getLLTx());
  }
}
