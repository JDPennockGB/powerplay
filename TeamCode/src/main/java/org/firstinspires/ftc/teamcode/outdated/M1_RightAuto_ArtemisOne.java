package org.firstinspires.ftc.teamcode.outdated;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionTest;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static java.lang.Math.toRadians;

@Disabled
@Autonomous
public class M1_RightAuto_ArtemisOne extends LinearOpMode {

    double liftTarget = 30;
    double pCoef = 0;
    double liftState = 0;

    DcMotor lm;
    private Servo LC, RC, LV, RV;
    OpenCvCamera webcam;
    VisionTest.SamplePipeline pipeline;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        lm = hardwareMap.get(DcMotor.class, "lm");

        //Servo hardware mapping
        LC = hardwareMap.get(Servo.class, "lc");
        RC = hardwareMap.get(Servo.class, "rc");
        LV = hardwareMap.get(Servo.class, "lv");
        RV = hardwareMap.get(Servo.class, "rv");

        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new VisionTest.SamplePipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        double ySet = 0;
        double xset = 0;

        Trajectory One = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(54 + xset, 0, toRadians(-10)))
                .build();
        Pose2d eOne = new Pose2d(54, 0, toRadians(-10));

        Trajectory Two = drive.trajectoryBuilder(eOne)
                .lineToLinearHeading(new Pose2d(50, 2, toRadians(-131)))
                .build();
        Pose2d eTwo = new Pose2d(50, 2, toRadians(-131));

        Trajectory Three = drive.trajectoryBuilder(eTwo)
                .lineToLinearHeading(new Pose2d(51 + xset, -15 + ySet, toRadians(-88)))
                .build();
        Pose2d eThree = new Pose2d(51 + xset, -15 + ySet, toRadians(-88));

        Trajectory Four = drive.trajectoryBuilder(eThree)
                .lineToLinearHeading(new Pose2d(51 + xset, 3 + ySet, toRadians(-132)))
                .build();
        Pose2d eFour = new Pose2d(51 + xset, 3 + ySet, toRadians(-132));

        Trajectory Five = drive.trajectoryBuilder(eFour)
                .lineToLinearHeading(new Pose2d(51 + xset, -17 + ySet, toRadians(-90)))
                .build();
        Pose2d eFive = new Pose2d(51 + xset, -17 + ySet, toRadians(-90));

        Trajectory Six = drive.trajectoryBuilder(eFive)
                .lineToLinearHeading(new Pose2d(51 + xset, 2 + ySet, toRadians(-132)))
                .build();
        Pose2d eSix = new Pose2d(51 + xset, 2 + ySet, toRadians(-132));

        Trajectory Seven = drive.trajectoryBuilder(eSix)
                .lineToLinearHeading(new Pose2d(51 + xset, -18 + ySet, toRadians(-90)))
                .build();
        Pose2d eSeven = new Pose2d(51 + xset, -18 + ySet, toRadians(-90));

        Trajectory Eight = drive.trajectoryBuilder(eSeven)
                .lineToLinearHeading(new Pose2d(50 + xset, 1 + ySet, toRadians(-134)))
                .build();
        Pose2d eEight = new Pose2d(50 + xset, 1 + ySet, toRadians(-134));

        Trajectory Nine = drive.trajectoryBuilder(eEight)
                .lineToLinearHeading(new Pose2d(51 + xset, -19 + ySet, toRadians(-88)))
                .build();
        Pose2d eNine = new Pose2d(51 + xset, -19 + ySet, toRadians(-88));

        Trajectory Ten = drive.trajectoryBuilder(eNine)
                .lineToLinearHeading(new Pose2d(49 + xset, -1 + ySet, toRadians(-138)))
                .build();
        Pose2d eTen = new Pose2d(49 + xset, -1 + ySet, toRadians(-138));

        Trajectory Eleven3 = drive.trajectoryBuilder(eTen)
                .lineToLinearHeading(new Pose2d(51, -23, toRadians(-90)))
                .build();
        Pose2d eEleven3 = new Pose2d(51, -23, toRadians(-90));

        Trajectory Eleven2 = drive.trajectoryBuilder(eTen)
                .lineToLinearHeading(new Pose2d(50, 0, toRadians(-179)))
                .build();
        Pose2d eEleven2 = new Pose2d(50, 0, toRadians(-179));

        Trajectory Eleven1 = drive.trajectoryBuilder(eTen)
                .lineToLinearHeading(new Pose2d(52, 22, toRadians(-179)))
                .build();
        Pose2d eEleven1 = new Pose2d(52, 22, toRadians(-179));

        Trajectory Twelve2 = drive.trajectoryBuilder(eEleven2)
                .lineToLinearHeading(new Pose2d(26, 0, toRadians(-179)))
                .build();
        Pose2d eTwelve2 = new Pose2d(26, 0, toRadians(-179));

        Trajectory Twelve1 = drive.trajectoryBuilder(eEleven1)
                .lineToLinearHeading(new Pose2d(28, 22, toRadians(-179)))
                .build();
        Pose2d eTwelve1 = new Pose2d(28, 22, toRadians(-179));


        while (!isStarted()){
            telemetry.addData("Location", pipeline.parkLocation);
            telemetry.update();
        }

        if (isStopRequested()) return;

        LC.setPosition(0.36);
        RC.setPosition(0.35);
        pCoef = 0.0;
        int parkingSpot = pipeline.parkLocation;
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        webcam.stopStreaming();

        drive.followTrajectory(One);

        sleep(50);
        LV.setPosition(0.85);
        RV.setPosition(0.15);
        sleep(250);

        drive.followTrajectory(Two);

        sleep(150);
        LC.setPosition(0.56);
        RC.setPosition(0.16);
        sleep(200);

        drive.followTrajectory(Three);

        LV.setPosition(0.465);
        RV.setPosition(0.535);
        sleep(100);
        LC.setPosition(0.36);
        RC.setPosition(0.35);
        sleep(300);
        lm.setTargetPosition(1000);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setPower(Math.abs(0.8));
        sleep(300);

        drive.followTrajectory(Four);

        sleep(200);
        LC.setPosition(0.56);
        RC.setPosition(0.16);
        sleep(200);
        LV.setPosition(0.15);
        RV.setPosition(0.85);
        lm.setTargetPosition(15);
        lm.setPower(Math.abs(0.8));
       // sleep(200);

        drive.followTrajectory(Five);

        LV.setPosition(0.36);
        RV.setPosition(0.64);
        sleep(200);
        LC.setPosition(0.36);
        RC.setPosition(0.35);
        sleep(200);
        lm.setTargetPosition(1200);
        lm.setPower(Math.abs(1));
        sleep(300);

        drive.followTrajectory(Six);

        sleep(200);
        LC.setPosition(0.56);
        RC.setPosition(0.16);
        sleep(200);
        LV.setPosition(0.15);
        RV.setPosition(0.85);
        lm.setTargetPosition(15);
        lm.setPower(Math.abs(1));
        //sleep(200);

        drive.followTrajectory(Seven);

        LV.setPosition(0.29);
        RV.setPosition(0.71);
        sleep(200);
        LC.setPosition(0.36);
        RC.setPosition(0.35);
        sleep(200);
        lm.setTargetPosition(1400);
        lm.setPower(Math.abs(1));
        sleep(300);

        drive.followTrajectory(Eight);

        sleep(200);
        LC.setPosition(0.56);
        RC.setPosition(0.16);
        sleep(200);
        LV.setPosition(0.15);
        RV.setPosition(0.85);
        lm.setTargetPosition(15);
        lm.setPower(Math.abs(1));

        drive.followTrajectory(Nine);

        LV.setPosition(0.21);
        RV.setPosition(0.79);
        sleep(200);
        LC.setPosition(0.34);
        RC.setPosition(0.37);
        sleep(300);
        lm.setTargetPosition(1500);
        lm.setPower(Math.abs(1));
        sleep(300);

        drive.followTrajectory(Ten);

        sleep(200);
        LC.setPosition(0.56);
        RC.setPosition(0.16);
        sleep(200);
        LV.setPosition(0.05);
        RV.setPosition(0.95);
        lm.setTargetPosition(0);
        lm.setPower(Math.abs(1));

        if (parkingSpot == 1){
            drive.followTrajectory(Eleven1);
            drive.followTrajectory(Twelve1);
        } if (parkingSpot == 2){
            drive.followTrajectory(Eleven2);
            drive.followTrajectory(Twelve2);
        } if (parkingSpot == 3){
            drive.followTrajectory(Eleven3);
        }
        sleep(7000);
    }
}


