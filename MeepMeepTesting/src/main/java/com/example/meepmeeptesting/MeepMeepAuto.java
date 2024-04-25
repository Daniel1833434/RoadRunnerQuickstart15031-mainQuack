package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

enum Auto {
    Auto1,
    Auto2,
    Auto3,
    Auto4,
    Auto5
}

public class MeepMeepAuto {
    public static final Auto auto  = Auto.Auto1;//Select which auto
    public static final double Delay = 0.5;
    public static final double MaxVel = 60;
    public static final double MaxAccel = 60;
    public static final double MaxAngVel = Math.toRadians(180);
    public static final double MaxAngAccel = Math.toRadians(180);
    public static final double TrackWidth = 15;


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myBot = null;
        switch (auto){
            case Auto1:
                myBot = Auto1(meepMeep);
                break;
            case Auto2:
                myBot = Auto2(meepMeep);
                break;
            case Auto3:

                break;
            case Auto4:

                break;
            case Auto5:

                break;
        }
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
    private static RoadRunnerBotEntity Auto1(MeepMeep meepMeep){
        double startheading = 90;//should always be 90 or 270
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MaxVel, MaxAccel, MaxAngVel,MaxAngAccel, TrackWidth)
                .followTrajectorySequence(drive ->
                        //starting pose

                        drive.trajectorySequenceBuilder(new Pose2d(-40, -64, Math.toRadians(startheading)))//starting pose
                                //The Paths
                                .splineTo(new Vector2d(-55,-35),Math.toRadians(90+startheading))
                                .waitSeconds(Delay)
                                .forward(5)
                                .waitSeconds(Delay)
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(48,-35,Math.toRadians(270+startheading)))
                                .waitSeconds(Delay)
                                .strafeRight(25)
                                .forward(10)
                                //Building
                                .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity Auto2(MeepMeep meepMeep){
        double startheading = 270;//should always be 90 or 270
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MaxVel, MaxAccel, MaxAngVel,MaxAngAccel, TrackWidth)
                .followTrajectorySequence(drive ->
                        //starting pose
                        drive.trajectorySequenceBuilder(new Pose2d(20, 60, Math.toRadians(startheading)))//starting point
                                //The Paths
                                .splineTo(new Vector2d(-60,35),Math.toRadians(270+startheading))
                                .waitSeconds(Delay)
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(48,35,Math.toRadians(90+startheading)))
                                .waitSeconds(Delay)
                                .strafeLeft(25)
                                .forward(10)
                                //Building
                                .build()
                );
        return myBot;
    }
}