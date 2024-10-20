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
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-6, -60, Math.toRadians(180)))
                .strafeTo(new Vector2d(-6, -45))// Math.toRadians(90))
                .waitSeconds(2)




                .splineToConstantHeading(new Vector2d(-48,-34),Math.toRadians(180))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-60, -60))//, Math.toRadians(90))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-58, -34))//, Math.toRadians(90))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-60, -60))//, Math.toRadians(90))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-60, -23))//, Math.toRadians(90))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-60, -60))//, Math.toRadians(90))
                        .waitSeconds(1)
                .strafeTo(new Vector2d(-25, -10))//, Math.toRadians(90))
                .waitSeconds(1)

               /* .splineToConstantHeading(new Vector2d(-52,-60),Math.toRadians(180))
                .waitSeconds(1)
                .strafeTo(new Vector2d(50, -20))// Math.toRadians(90))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-52,-60),Math.toRadians(180))
                .waitSeconds(1)
                .strafeTo(new Vector2d(54, -64))// Math.toRadians(180))
                .waitSeconds(1) */

//                        .waitSeconds(2)
                /*      .lineToX(30)

     /* .splineTo(new Vector2d(38, -55), Math.toRadians(180))
              .waitSeconds(2)
      .splineTo(new Vector2d(53, -50), Math.toRadians(310))
              .waitSeconds(2)
      .splineTo(new Vector2d(24, -55), Math.toRadians(270))
                      .waitSeconds(2)
              .lineToY(-50)
      .splineTo(new Vector2d(53, -50), Math.toRadians(130))

 */



                //.setTangent(Math.toRadians(90))
                //.lineToY(55)
                //.setTangent(Math.toRadians(0))
                //.lineToX(32)
                //.setTangent(Math.toRadians(0))
                //.splineTo(new Vector2d(44,28), Math.toRadians(0))
                .build());

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\rgutti1\\Documents\\PERSONAL\\LAASYA\\ROBOTICS\\intothedeep.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
//  <following code you were using previously>

    }
}
