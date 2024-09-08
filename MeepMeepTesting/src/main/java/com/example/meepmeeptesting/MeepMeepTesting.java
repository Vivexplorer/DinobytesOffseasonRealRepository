package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -34, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(34,-10),Math.toRadians(180))
                        .waitSeconds(2)
                        .lineToX(40)
                .splineToConstantHeading(new Vector2d(38,-45),Math.toRadians(180))
                                .waitSeconds(2)
                                .splineTo(new Vector2d(53, -50), Math.toRadians(310))
                        .waitSeconds(2)

                .splineTo(new Vector2d(38, -55), Math.toRadians(180))
                        .waitSeconds(2)
                .splineTo(new Vector2d(53, -50), Math.toRadians(310))
                        .waitSeconds(2)
                .splineTo(new Vector2d(24, -55), Math.toRadians(270))
                                .waitSeconds(2)
                        .lineToY(-50)
                .splineTo(new Vector2d(53, -50), Math.toRadians(130))





                //.setTangent(Math.toRadians(90))
                //.lineToY(55)
                //.setTangent(Math.toRadians(0))
                //.lineToX(32)
                //.setTangent(Math.toRadians(0))
                //.splineTo(new Vector2d(44,28), Math.toRadians(0))
                        .build());

        Image img = null;
        try { img = ImageIO.read(new File("D:\\IMPORTANT PICTURES\\IntoTheDeepField.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                        .setBackgroundAlpha(0.95f)
                                .addEntity(myBot)
                                        .start();
//  <following code you were using previously>

    }
}