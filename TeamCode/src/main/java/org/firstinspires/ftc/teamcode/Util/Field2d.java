package org.firstinspires.ftc.teamcode.Util;

import com.arcrobotics.ftclib.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class Field2d {
    private final List<FieldObject2d> m_objects = new ArrayList<>();

    public Field2d() {
        FieldObject2d obj = new FieldObject2d("Robot");
        obj.setPose(new Pose2d());
        m_objects.add(obj);
    }

    public synchronized void setRobotPose(Pose2d pose) {
        m_objects.get(0).setPose(pose);
    }

    public synchronized Pose2d getRobotPose() {
        return m_objects.get(0).getPose();
    }

    /**
     * Get or create a field object.
     *
     * @param name The field object's name.
     * @return Field object
     */
    public synchronized FieldObject2d getObject(String name) {
        for (FieldObject2d obj : m_objects) {
            if (obj.m_name.equals(name)) {
                return obj;
            }
        }
        FieldObject2d obj = new FieldObject2d(name);
        m_objects.add(obj);
        return obj;
    }

    /**
     * Get the robot object.
     *
     * @return Field object for robot
     */
    public synchronized FieldObject2d getRobotObject() {
        return m_objects.get(0);
    }
}
