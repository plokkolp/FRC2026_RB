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
import edu.wpi.first.wpilibj.Timer; // <<< 新增
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

  private double txFilteredDeg = 0.0;     // 給命令用（穩定/快速）
  private boolean txEverValid = false;
  private double lastSeenTimeSec = 0.0;

  private static final double kTxAlpha = 0.5;        // 越大越快 (0~1)
  private static final double kHoldTimeoutSec = 0.20; // 沒看到目標最多保留多久
  private static final double kMaxAbsTxDeg = 35.0;    // 避免爆值

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

  public void setAllTrainSpeed(double speed){
    setTrainSpeed(speed);
    setIntaketrainSpeed(speed);
  }

  public boolean hasLLTarget() {
    return ll4.hasTarget();
  }

  public double getLLTxRaw() {
    return ll4.getTX(); // degrees
  }

  public double getLLTx() {
    return txFilteredDeg; // degrees (filtered + hold)
  }

  public void setIntaketrainSpeed(double speed) {
    motor.set(speed);
  }

  public double getlong() {
    return ll4.getBestTagDistanceMeters();
  }

  

  @Override
  public void periodic() {

    double now = Timer.getFPGATimestamp();

    boolean has = hasLLTarget();
    double rawTx = getLLTxRaw();

    if (has && Double.isFinite(rawTx) && Math.abs(rawTx) <= kMaxAbsTxDeg) {
      lastSeenTimeSec = now;
      txEverValid = true;

      //遠快追
      txFilteredDeg = (1.0 - kTxAlpha) * txFilteredDeg + kTxAlpha * rawTx;
    } else {
      //跳變
      if (txEverValid && (now - lastSeenTimeSec) <= kHoldTimeoutSec) {
        // keep txFilteredDeg
      } else {
        txFilteredDeg = 0.0; //超時歸零
      }
    }

    SmartDashboard.putNumber("LL/tx_raw", rawTx);
    SmartDashboard.putNumber("LL/tx_filtered", txFilteredDeg);
    SmartDashboard.putBoolean("LL/tx_hold_active",
        txEverValid && (now - lastSeenTimeSec) <= kHoldTimeoutSec);

  }
}
