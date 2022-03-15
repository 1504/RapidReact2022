package frc.robot.Pathing;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class PathChooser {
    
    enum Paths {
        BASIC_SHOOT("Paths/Basic Shoot.wpilib.json"),
        CRINGE("");

        public final String dir;

        Paths(String _d) {
            dir = _d;
        }
    }

    private SendableChooser<Paths> mPathChooser;

    ShuffleboardTab a_tab = Shuffleboard.getTab("Auton");
    NetworkTableEntry autonPath;

    public PathChooser() {
        mPathChooser = new SendableChooser<>();
        mPathChooser.setDefaultOption("Basic", Paths.BASIC_SHOOT);
        mPathChooser.addOption("Cringe", Paths.CRINGE);

        a_tab.add("Auton Pathing", mPathChooser)
            .withPosition(0, 0)
            .withSize(3, 1);
            
    }

    public Paths getPath() {
        return mPathChooser.getSelected();
    }

    public Trajectory getTrajectory() {
        try {
            Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(mPathChooser.getSelected().dir);
            return TrajectoryUtil.fromPathweaverJson(trajPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + mPathChooser.getSelected().dir, ex.getStackTrace());
            return new Trajectory();
        }

    }

}
