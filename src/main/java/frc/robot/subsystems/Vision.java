// package frc.robot.subsystems;

// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Vision extends SubsystemBase {

//     public enum VisionAction {
//         INITIALIZE,
//         FUSE,
//         IGNORE
//     }

//     public static class VisionUpdate {
//         public final VisionAction action;
//         public final Pose2d pose;
//         public final double timestamp;
//         public final double stdX, stdY, stdTheta;

//         private VisionUpdate(
//                 VisionAction action,
//                 Pose2d pose,
//                 double timestamp,
//                 double sx,
//                 double sy,
//                 double st) {
//             this.action = action;
//             this.pose = pose;
//             this.timestamp = timestamp;
//             this.stdX = sx;
//             this.stdY = sy;
//             this.stdTheta = st;
//         }

//         public static VisionUpdate initialize(Pose2d pose) {
//             return new VisionUpdate(VisionAction.INITIALIZE, pose, 0.0, 0, 0, 0);
//         }

//         public static VisionUpdate fuse(
//                 Pose2d pose, double ts, VisionStdDevs std) {
//             return new VisionUpdate(
//                     VisionAction.FUSE,
//                     pose,
//                     ts,
//                     std.stdX,
//                     std.stdY,
//                     std.stdThetaRad);
//         }

//         public static VisionUpdate ignore() {
//             return new VisionUpdate(VisionAction.IGNORE, null, 0.0, 0, 0, 0);
//         }
//     }

//     private final PhotonCamera rightCam = new PhotonCamera("Right");
//     private final PhotonCamera leftCam  = new PhotonCamera("Left");

//     private PhotonPoseEstimator rightEstimator;
//     private PhotonPoseEstimator leftEstimator;

//     private final CommandSwerveDrivetrain drivetrain;

//     private boolean hasInitializedPose = false;

//     public static final Transform3d kRightTransform =
//             new Transform3d(
//                     new Translation3d(0.265, -0.2, 0.27),
//                     new Rotation3d(0, Math.toRadians(-10), Math.toRadians(30)));

//     public static final Transform3d kLeftTransform =
//             new Transform3d(
//                     new Translation3d(0.0, 0.3, 0.5),
//                     new Rotation3d(0, Math.toRadians(20), 0));

//     public Vision(CommandSwerveDrivetrain drivetrain) {
//         this.drivetrain = drivetrain;

//         AprilTagFieldLayout fieldLayout = null;
//         try {

//             fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
//         } catch (Exception e) {

//             DriverStation.reportError("No Field", e.getStackTrace());
//         }


//         if (fieldLayout != null) {
//             rightEstimator = new PhotonPoseEstimator(
//                     fieldLayout,
//                     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                     kRightTransform);

//             leftEstimator = new PhotonPoseEstimator(
//                     fieldLayout,
//                     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                     kLeftTransform);

//             rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//             leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//         }
//     }

//     @Override
//     public void periodic() {

//         if (rightEstimator == null || leftEstimator == null) return;

//         Pose2d currentPose = drivetrain.getPose();
//         Pose3d ref = new Pose3d(currentPose);

//         rightEstimator.setReferencePose(ref);
//         leftEstimator.setReferencePose(ref);

//         handleCamera(rightCam, rightEstimator, currentPose);
//         handleCamera(leftCam, leftEstimator, currentPose);

//         if (SmartDashboard.getBoolean("Vision/ForceResetInit", false)) {
//             hasInitializedPose = false;
//             SmartDashboard.putBoolean("Vision/ForceResetInit", false);
//         }
//     }

//     private void handleCamera(
//             PhotonCamera cam,
//             PhotonPoseEstimator estimator,
//             Pose2d currentPose) {

//         PhotonPipelineResult result = cam.getLatestResult();
//         if (!result.hasTargets()) {
//             updateDashboard(cam, "No Targets");
//             return;
//         }

//         Optional<EstimatedRobotPose> opt = estimator.update(result);
//         if (opt.isEmpty()) return;

//         EstimatedRobotPose erp = opt.get();
//         Pose2d visionPose = erp.estimatedPose.toPose2d();

//         int tagCount = erp.targetsUsed.size();
//         double avgAmb = 0.0, avgDist = 0.0;

//         for (var t : erp.targetsUsed) {
//             avgAmb += t.getPoseAmbiguity();
//             avgDist += t.getBestCameraToTarget().getTranslation().getNorm();
//         }
//         avgAmb /= tagCount;
//         avgDist /= tagCount;

//         double poseDiff =
//                 currentPose.getTranslation()
//                         .getDistance(visionPose.getTranslation());

//         if (!hasInitializedPose) {
//             if (tagCount >= 2 && avgAmb < 0.1) {
//                 drivetrain.handleVisionUpdate(
//                         VisionUpdate.initialize(visionPose));
//                 hasInitializedPose = true;
//                 updateDashboard(cam, "INITIALIZED");
//             }
//             return;
//         }

//         if (poseDiff > 0.8) {
//             updateDashboard(cam, "REJECTED");
//             return;
//         }

//         VisionStdDevs std =
//                 VisionStdDevs.compute(tagCount, avgAmb, avgDist);

//         drivetrain.handleVisionUpdate(
//                 VisionUpdate.fuse(
//                         visionPose,
//                         result.getTimestampSeconds(),
//                         std));

//         updateDashboard(cam, "FUSED");
//     }

//     private void updateDashboard(PhotonCamera cam, String status) {
//         SmartDashboard.putString("Vision/" + cam.getName(), status);
//     }

//     public static class VisionStdDevs {
//         public final double stdX, stdY, stdThetaRad;

//         private VisionStdDevs(double sx, double sy, double st) {
//             stdX = sx;
//             stdY = sy;
//             stdThetaRad = st;
//         }

//         public static VisionStdDevs compute(
//                 int tagCount,
//                 double avgAmb,
//                 double avgDist) {

//             double sx = 0.05, sy = 0.05, st = Math.toRadians(2);

//             if (tagCount <= 1) {
//                 sx *= 5;
//                 sy *= 5;
//                 st *= 5;
//             }

//             double ambScale =
//                     1 + 2 * clamp(avgAmb, 0.0, 0.6) / 0.6;
//             double distScale =
//                     1 + 2 * clamp(avgDist, 0.5, 8.0) / 8.0;

//             return new VisionStdDevs(
//                 clamp(sx * ambScale * distScale, 0.03, 0.5),
//                 clamp(sy * ambScale * distScale, 0.03, 0.5),
//                 clamp(st * ambScale * distScale,
//                         Math.toRadians(1),
//                         Math.toRadians(15)));
//         }

//         private static double clamp(double v, double lo, double hi) {
//             return Math.max(lo, Math.min(hi, v));
//         }
//     }
// }