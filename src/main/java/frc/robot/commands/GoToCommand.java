package frc.robot.commands;

import java.util.ArrayList;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

/*
 * Go to a specific point on a field.
 */
public class GoToCommand extends SequentialCommandGroup {
    private PathPlannerTrajectory trajectory;

    public GoToCommand(PathPoint targetPathPoint, DrivetrainSubsystem drivetrainSubsystem) {
        addCommands(new InstantCommand(() -> {
            var currentPose = drivetrainSubsystem.odometry.getPose();

            ArrayList<PathPoint> points = new ArrayList<>();
            points.add(new PathPoint(currentPose.getTranslation(), currentPose.getRotation()));
            points.add(targetPathPoint);

            this.trajectory = PathPlanner.generatePath(new PathConstraints(1, .1), points);
        }), drivetrainSubsystem.followTrajectorySuppliedCommand(() -> this.trajectory));
    }
}
