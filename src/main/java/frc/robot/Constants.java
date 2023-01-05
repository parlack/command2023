package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {

    public static final class PUERTOSCAN {

        public static int PuertMotDer1yEncoder = 1;
        public static int PuertMotDer2 = 2;
        public static int PuertMotDer3 = 3;

        public static int PuertMotIzq1yEncoder = 4;
        public static int PuertMotIzq2 = 5;
        public static int PuertMotIzq3 = 6;



    }


    public static final class DriveConstants {

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.79733;
        public static final double kvVoltSecondsPerMeter = 3.0761;
        public static final double kaVoltSecondsSquaredPerMeter = 0.79086;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 2.0033;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }



}
