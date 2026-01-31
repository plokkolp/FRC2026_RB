package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathDrive {
    private static final PathConstraints kConstraints = new PathConstraints(
        3.0, 3.0, 4.0, 4.0 
    );

    public static Command driveToPose(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(
            targetPose,
            kConstraints,
            0.0 
        );
    }
}