package frc.trigon.robot.components;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;

public class VisionManager {
    private static final String CAMERA_NAME = "gloworm";
    public PhotonCamera camera;
    public Pose3d cameraPose = new Pose3d(new Translation3d(0.195, 0, 0), new Rotation3d(0, 0, 0));

    private static final VisionManager instance = new VisionManager();

    private VisionManager() {
        camera = new PhotonCamera(CAMERA_NAME);
    }

    public static VisionManager getInstance() {
        return instance;
    }
}
