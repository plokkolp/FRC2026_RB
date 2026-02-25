package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

public class LL4 extends SubsystemBase {

  private final String name;

  public LL4() {
    this("limelight-shoot");
  }

  public LL4(String name) {
    if (name == null || name.isEmpty()) {
      throw new IllegalArgumentException("Limelight name CANNOT be empty");
    }
    this.name = name;
  }


  public boolean hasTarget() {
    return LimelightHelpers.getTV(name);
  }

  public double getTX() {
    return LimelightHelpers.getTX(name);
  }

  public double getTY() {
    return LimelightHelpers.getTY(name);
  }

  public double getTA() {
    return LimelightHelpers.getTA(name);
  }

  public double getTXNC() {
    return LimelightHelpers.getTXNC(name);
  }

  public double getTYNC() {
    return LimelightHelpers.getTYNC(name);
  }

  public double getTagID() {
    return LimelightHelpers.getFiducialID(name);
  }

  public double getHeartbeat() {
    return LimelightHelpers.getHeartbeat(name);
  }

  public void setPipeline(int index) {
    LimelightHelpers.setPipelineIndex(name, index);
  }

  public void ledPipelineControl() { LimelightHelpers.setLEDMode_PipelineControl(name); }
  public void ledForceOn()         { LimelightHelpers.setLEDMode_ForceOn(name); }
  public void ledForceOff()        { LimelightHelpers.setLEDMode_ForceOff(name); }
  public void ledForceBlink()      { LimelightHelpers.setLEDMode_ForceBlink(name); }


  public RawFiducial[] getRawFiducials() {
    return LimelightHelpers.getRawFiducials(name);
  }

  public RawDetection[] getRawDetections() {
    return LimelightHelpers.getRawDetections(name);
  }

  public boolean hasTag(int tagId) {
    RawFiducial[] tags = getRawFiducials();
    if (tags == null) return false;
    for (RawFiducial t : tags) {
      if (t != null && t.id == tagId) return true;
    }
    return false;
  }

  public boolean hasSpeakerTag() {
    return hasTag(10) || hasTag(26);
  }

  public int getBestFiducialId() {
    var results = LimelightHelpers.getLatestResults(name);
    if (results == null || results.targets_Fiducials == null) return -1;

    LimelightHelpers.LimelightTarget_Fiducial best = null;
    for (var t : results.targets_Fiducials) {
      if (best == null || t.ta > best.ta) best = t;
    }
    return (best == null) ? -1 : (int) best.fiducialID;
  }

  public double getBestGoalYawDeg() {
    if (!hasTarget()) return Double.NaN;
    return getTX(); 
  }

  public boolean hasLLTarget() {
    return hasTarget();
  }

  //

  public PoseEstimate getPoseEstimateBlue_MegaTag1() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
  }

  public PoseEstimate getPoseEstimateBlue_MegaTag2() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  }

  public void setRobotOrientationDeg(
      double yawDeg,
      double yawRateDegPerSec,
      double pitchDeg,
      double pitchRateDegPerSec,
      double rollDeg,
      double rollRateDegPerSec
  ) {
    LimelightHelpers.SetRobotOrientation(
        name,
        yawDeg, yawRateDegPerSec,
        pitchDeg, pitchRateDegPerSec,
        rollDeg, rollRateDegPerSec
    );
  }

  public LimelightHelpers.IMUData getIMU() {
    return LimelightHelpers.getIMUData(name);
  }

  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(name, mode);
  }

  public void setIMUAssistAlpha(double alpha) {
    LimelightHelpers.SetIMUAssistAlpha(name, alpha);
  }
  public double getBestTagDistanceMeters() {
    var results = LimelightHelpers.getLatestResults(name);
    if (results == null || results.targets_Fiducials == null) return Double.NaN;
    LimelightHelpers.LimelightTarget_Fiducial best = null;
    for (var t : results.targets_Fiducials) {
      if (best == null || t.ta > best.ta) best = t;
    }
    if (best == null) return Double.NaN;
    var pose = best.getTargetPose_CameraSpace();
    if (pose == null) return Double.NaN;
    return pose.getTranslation().getNorm(); 
    
  }
 
//   public double getDistanceMeters() {
//   var results = LimelightHelpers.getLatestResults(name);
//   if (results == null || results.targets_Fiducials == null) return Double.NaN;

//   LimelightHelpers.LimelightTarget_Fiducial best = null;
//   for (var t : results.targets_Fiducials) {
//     if (t == null) continue;
//     if (best == null || t.ta > best.ta) best = t;
//   }
//   if (best == null) return Double.NaN;

//   Pose3d tagInCam = best.getTargetPose_CameraSpace();
//   if (tagInCam == null) return Double.NaN;

//   double forward = -0.60;
//   double right   =  0.00;
//   double up      = -0.70;

//   Transform3d tagToPOI = new Transform3d(
//       new Translation3d(forward, -right, up),
//       new Rotation3d()
//   );

//   Pose3d poiInCam = tagInCam.transformBy(tagToPOI);

//   return poiInCam.getTranslation().getNorm();
// }

public double getDistanceMeters() {
    var results = LimelightHelpers.getLatestResults(name);
    if (results == null || results.targets_Fiducials == null) return Double.NaN;

    LimelightHelpers.LimelightTarget_Fiducial best = null;
    for (var t : results.targets_Fiducials) {
        if (t == null) continue;
        if (best == null || t.ta > best.ta) best = t;
    }
    if (best == null) return Double.NaN;

    Pose3d tagInCam = best.getTargetPose_CameraSpace();
    if (tagInCam == null) return Double.NaN;

    
    double forward_z = -0.6;
    double right_x   =  0.0; 
    double up_y      = -0.7; 
    Transform3d tagToPOI = new Transform3d(
        new Translation3d(right_x, up_y, forward_z), 
        new Rotation3d() 
    );

    Pose3d poiInCam = tagInCam.transformBy(tagToPOI);

    return poiInCam.getTranslation().getNorm();
}
  

  @Override
  public void periodic() {
    // SmartDashboard.putString("LL4/Name", name);

    // SmartDashboard.putBoolean("LL4/HasTarget", hasTarget());
    // SmartDashboard.putNumber("LL4/tx", getTX());
    // SmartDashboard.putNumber("LL4/ty", getTY());
    // SmartDashboard.putNumber("LL4/ta", getTA());
    // SmartDashboard.putNumber("LL4/txnc", getTXNC());
    // SmartDashboard.putNumber("LL4/tync", getTYNC());
    // SmartDashboard.putNumber("LL4/tagID", getTagID());
    // SmartDashboard.putNumber("LL4/bestYawErrDeg(A)", getBestGoalYawDeg());
    // SmartDashboard.putNumber("Long/Tag", getBestTagDistanceMeters());
    // SmartDashboard.putNumber("Long/Hub", getDistanceMeters());

  }
}
