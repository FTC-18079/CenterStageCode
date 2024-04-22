package org.firstinspires.ftc.teamcode.Util;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class FieldObject2d {
    String m_name;
    double[] m_entry;
    private final List<Pose2d> m_poses = new ArrayList<>();

    FieldObject2d(String name) {
        m_name = name;
    }

    public synchronized void setPose(Pose2d pose) {
        setPoses(pose);
    }

    public synchronized Pose2d getPose() {
        updateFromEntry();
        if (m_poses.isEmpty()) {
            return new Pose2d();
        }
        return m_poses.get(0);
    }

    public synchronized void setPoses(List<Pose2d> poses) {
        m_poses.clear();
        m_poses.addAll(poses);
        updateEntry();
    }

    /**
     * Set multiple poses from a list of Pose objects. The total number of poses is limited to 85.
     *
     * @param poses list of 2D poses
     */
    public synchronized void setPoses(Pose2d... poses) {
        m_poses.clear();
        Collections.addAll(m_poses, poses);
        updateEntry();
    }

    public synchronized List<Pose2d> getPoses() {
        updateFromEntry();
        return new ArrayList<>(m_poses);
    }

    synchronized void updateEntry() {
        if (m_entry == null) {
            return;
        }

        double[] arr = new double[m_poses.size() * 3];
        int ndx = 0;
        for (Pose2d pose : m_poses) {
            Translation2d translation = pose.getTranslation();
            arr[ndx + 0] = translation.getX();
            arr[ndx + 1] = translation.getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }

        m_entry = arr;
    }

    private synchronized void updateFromEntry() {
        if (m_entry == null) {
            return;
        }

        double[] arr = m_entry;
        if (arr != null) {
            if ((arr.length % 3) != 0) {
                return;
            }

            m_poses.clear();
            for (int i = 0; i < arr.length; i += 3) {
                m_poses.add(new Pose2d(arr[i], arr[i + 1], Rotation2d.fromDegrees(arr[i + 2])));
            }
        }
    }
}
