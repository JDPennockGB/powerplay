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
public class M3_RightAuto extends LinearOpMode {

    //DcMotor lm;
    private Servo lc, rc, LV, RV;
    OpenCvCamera webcam;
    VisionTest.SamplePipeline pipeline;
    private DcMotor liftRight = null, liftLeft = null;
    double target = 42;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //lm = hardwareMap.get(DcMotor.class, "lm");

        //Servo hardware mapping
        lc = hardwareMap.get(Servo.class, "lc");
        rc = hardwareMap.get(Servo.class, "rc");
        LV = hardwareMap.get(Servo.class, "lv");
        RV = hardwareMap.get(Servo.class, "rv");

        liftRight = hardwareMap.get(DcMotor.class, "lrm");
        liftLeft = hardwareMap.get(DcMotor.class, "llm");

        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        Pose2d start = new Pose2d(-36, 62, toRadians(-90));

        Trajectory One = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-36, 11, toRadians(-90)))
                .build();
        Pose2d eOne = new Pose2d(-36, 11, toRadians(-90));

        Trajectory Two = drive.trajectoryBuilder(eOne)
                .lineToLinearHeading(new Pose2d(-25, 10, toRadians(-92)))
                .build();
        Pose2d eTwo = new Pose2d(-25, 10, toRadians(-92));

        Trajectory Three = drive.trajectoryBuilder(eTwo)
                .lineToLinearHeading(new Pose2d(-36, 13, toRadians(-92)))
                .build();
        Pose2d eThree = new Pose2d(-36, 13, toRadians(-92));

        Trajectory Four = drive.trajectoryBuilder(eThree)
                .lineToLinearHeading(new Pose2d(-34, 30, toRadians(-92)))
                .build();
        Pose2d eFour = new Pose2d(-34, 30, toRadians(-92));

        Trajectory Five = drive.trajectoryBuilder(eFour)
                .lineToLinearHeading(new Pose2d(-34, 34, toRadians(-265)))
                .build();
        Pose2d eFive = new Pose2d(-34, 34, toRadians(-265));

        Trajectory Park1 = drive.trajectoryBuilder(eFive)
                .lineToLinearHeading(new Pose2d(-15, 33, toRadians(-266)))
                .build();

        Trajectory Park3 = drive.trajectoryBuilder(eFive)
                .lineToLinearHeading(new Pose2d(-63, 33, toRadians(-266)))
                .build();


        while (!isStarted()){
            telemetry.addData("Location", pipeline.parkLocation);
            telemetry.update();

            lc.setPosition(0.4);
            rc.setPosition(0.6);
        }

        if (isStopRequested()) return;

        drive.setPoseEstimate(start);

        sleep(300);
        int parkingSpot = pipeline.parkLocation;
        webcam.stopStreaming();

        drive.followTrajectoryAsync(One);

        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            liftLeft.setPower((1800 - liftLeft.getCurrentPosition()) * 0.01);
            liftRight.setPower((1800 - liftLeft.getCurrentPosition()) * 0.01);
        }

        LV.setPosition(1);
        RV.setPosition(0);
        sleep(500);

        drive.followTrajectory(Two);

        sleep(500);

        lc.setPosition(0.28);
        rc.setPosition(0.72);

        drive.followTrajectory(Three);

        drive.followTrajectoryAsync(Four);

        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            liftLeft.setPower((50 - liftLeft.getCurrentPosition()) * 0.01);
            liftRight.setPower((50 - liftLeft.getCurrentPosition()) * 0.01);
        }

        LV.setPosition(0.05);
        RV.setPosition(0.95);

        sleep(500);

        drive.followTrajectory(Five);

        if (parkingSpot == 1){
            drive.followTrajectory(Park1);
            sleep(25000);
        } if (parkingSpot == 2){
            sleep(25000);
        } if (parkingSpot == 3){
            drive.followTrajectory(Park3);
            sleep(25000);
        }
    }
}


