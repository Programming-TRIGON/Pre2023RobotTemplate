package frc.trigon.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class AprilTagsLocations {
    public static final AprilTag[] locations = {
            new AprilTag(42, 5, 4, 180)
    };

    public static AprilTag getTag(int id) {
        for (AprilTag tag : locations) {
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }

    public static class AprilTag {
        public final int id;
        public final Pose3d pose;

        public AprilTag(int id, Pose3d pose) {
            this.id = id;
            this.pose = pose;
        }

        public AprilTag(int id, double x, double y, double yaw) {
            this(id,  new Pose3d(new Translation3d(x, y, 0), new Rotation3d(0, 0, Units.degreesToRadians(yaw))));
        }
    }
}
