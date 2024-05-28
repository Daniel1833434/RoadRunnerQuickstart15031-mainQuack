package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.nfc.Tag;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagDetections {
    AprilTagProcessor AprilTagQuack;
    AprilTagDetection TagQuack;
    public void InitAprilTagDetection(){
          AprilTagQuack = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortalQuack = new VisionPortal.Builder()
                .addProcessor(AprilTagQuack)
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam1"))//change to camera name
                .setCameraResolution(new Size(640,480))
                .build();


    }
    public void LoopAprilTagDetection(){
        if(AprilTagQuack.getDetections().size()>0){
            TagQuack = AprilTagQuack.getDetections().get(0);

        }
    }

}
