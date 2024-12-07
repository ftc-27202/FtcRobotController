package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel
                // track width (Jeff's track width is 16", Jeff's track length is 17")
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-41, -60, 0))
                        .strafeTo(new Vector2d(-43, -60))
                        .splineTo(new Vector2d(-28, -35), Math.toRadians(160))
                        .lineToSplineHeading(-30, Math.toRadians(160))
                        .lineToYSplineHeading(-30, Math.toRadians(160))


//                        .waitSeconds(1)
//                        .back(6)
//                        .lineToLinearHeading(new Pose2d(-50, 6, 0))
//                        .turn(Math.toRadians(180) - 1e-6)
//                        .strafeTo(new Vector2d(-38,-24))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}