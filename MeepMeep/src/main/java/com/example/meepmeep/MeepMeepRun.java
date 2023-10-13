
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
                .setConstraints(150, 90, Math.toRadians(360), Math.toRadians(200), 13.325)
                .setDimensions(16.535, 17.3228)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -61, Math.toRadians(90)))
                                .forward(26)
                                .waitSeconds(0.25)
                                .back(24)
                                .strafeTo(new Vector2d(24, -61))
                                .lineToSplineHeading(new Pose2d(38, -38, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .strafeTo(new Vector2d(38, -12))
                                .lineToSplineHeading(new Pose2d(-56, -12, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(38, -12, Math.toRadians(180)))
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