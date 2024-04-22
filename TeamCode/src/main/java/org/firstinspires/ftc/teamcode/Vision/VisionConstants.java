package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Util.Transform3d;

public class VisionConstants {
    public enum Camera {
        LEFT("cameraLeft", new Transform3d(
                new VectorF(3, 3, 1),
                new Quaternion()
        )),
        RIGHT("cameraRight", new Transform3d(
                new VectorF(3, 3, 1),
                new Quaternion()
        ));

        public final String name;
        public final Transform3d offset;
        Camera(String name, Transform3d offset) {
            this.name = name;
            this.offset = offset;
        }
    }
}
