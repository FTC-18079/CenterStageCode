
package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepRun {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(725);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 40, Math.toRadians(201.6), Math.toRadians(180), 13.325)
                .setDimensions(420/25.4, 440/25.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63.339, Math.toRadians(90)))
                                .forward(22)
                                .waitSeconds(0.25)
                                .back(9)
                                .splineToSplineHeading(new Pose2d(46, -35, Math.toRadians(0)), Math.toRadians(20))
                                .waitSeconds(0.5)
                                .back(0.5)
                                .splineToConstantHeading(new Vector2d(59, -60), Math.toRadians(0))
                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\nicsi\\Documents\\centerstagefield.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img /*MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL*/ )
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}