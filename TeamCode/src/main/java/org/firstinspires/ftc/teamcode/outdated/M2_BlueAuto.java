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
public class M2_BlueAuto extends LinearOpMode {

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
                .lineToLinearHeading(new Pose2d(-36, 10, toRadians(-105)))
                .build();
        Pose2d eOne = new Pose2d(-36, 10, toRadians(-105));

        Trajectory Two = drive.trajectoryBuilder(eOne)
                .lineToLinearHeading(new Pose2d(-40, 10, toRadians(-235)))
                .build();
        Pose2d eTwo = new Pose2d(-40, 10, toRadians(-235));

        Trajectory Three = drive.trajectoryBuilder(eTwo)
                .lineToLinearHeading(new Pose2d(-53, 10, toRadians(-178)))
                .build();
        Pose2d eThree = new Pose2d(-53, 10, toRadians(-178));

        Trajectory Four = drive.trajectoryBuilder(eThree)
                .lineToLinearHeading(new Pose2d(-12, 10, toRadians(-178)))
                .build();
        Pose2d eFour = new Pose2d(-12, 10, toRadians(-178));

        Trajectory Five = drive.trajectoryBuilder(eFour)
                .lineToLinearHeading(new Pose2d(-20, 10, toRadians(-110)))
                .build();
        Pose2d eFive = new Pose2d(-20, 10, toRadians(-110));

        Trajectory Six = drive.trajectoryBuilder(eFive)
                .lineToLinearHeading(new Pose2d(-16, 11, toRadians(-178)))
                .build();
        Pose2d eSix = new Pose2d(-16, 11, toRadians(-178));

        Trajectory Seven = drive.trajectoryBuilder(eSix)
                .lineToLinearHeading(new Pose2d(-61, 11, toRadians(-178)))
                .build();
        Pose2d eSeven = new Pose2d(-61, 11, toRadians(-178));

        Trajectory Eight = drive.trajectoryBuilder(eSeven)
                .lineToLinearHeading(new Pose2d(-12, 11, toRadians(-178)))
                .build();
        Pose2d eEight = new Pose2d(-12, 11, toRadians(-178));

        Trajectory Nine = drive.trajectoryBuilder(eEight)
                .lineToLinearHeading(new Pose2d(-19, 10.5, toRadians(-110)))
                .build();
        Pose2d eNine = new Pose2d(-19, 10.5, toRadians(-110));

        Trajectory Ten = drive.trajectoryBuilder(eNine)
                .lineToLinearHeading(new Pose2d(-16, 11, toRadians(-178)))
                .build();
        Pose2d eTen = new Pose2d(-16, 11, toRadians(-178));

        Trajectory Park1 = drive.trajectoryBuilder(eTen)
                .lineToLinearHeading(new Pose2d(-10.5, 15, toRadians(-270)))
                .build();

        Trajectory Park2 = drive.trajectoryBuilder(eTen)
                .lineToLinearHeading(new Pose2d(-35, 11, toRadians(-180)))
                .build();

        Trajectory Park3 = drive.trajectoryBuilder(eTen)
                .lineToLinearHeading(new Pose2d(-59, 12, toRadians(-180)))
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
            liftLeft.setPower((target - liftLeft.getCurrentPosition()) * 0.01);
            liftRight.setPower((target - liftLeft.getCurrentPosition()) * 0.01);
        }

        LV.setPosition(0.85);
        RV.setPosition(0.15);
        sleep(250);

        drive.followTrajectory(Two);

        sleep(200);
        lc.setPosition(0.28);
        rc.setPosition(0.72);
        sleep(250);

        drive.followTrajectory(Three);

        LV.setPosition(0.46);
        RV.setPosition(0.54);
        sleep(250);
        lc.setPosition(0.4);
        rc.setPosition(0.6);
        sleep(300);

        drive.followTrajectoryAsync(Four);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            liftLeft.setPower((1920 - liftLeft.getCurrentPosition()) * 0.01);
            liftRight.setPower((1920 - liftLeft.getCurrentPosition()) * 0.01);
        }
        LV.setPosition(1);
        RV.setPosition(0);

        drive.followTrajectory(Five);

        sleep(100);
        lc.setPosition(0.28);
        rc.setPosition(0.72);
        sleep(250);

        drive.followTrajectory(Six);

        LV.setPosition(0.05);
        RV.setPosition(0.95);

        drive.followTrajectoryAsync(Seven);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            liftLeft.setPower((450 - liftLeft.getCurrentPosition()) * 0.01);
            liftRight.setPower((450 - liftLeft.getCurrentPosition()) * 0.01);
        }

        lc.setPosition(0.4);
        rc.setPosition(0.6);
        sleep(200);

        drive.followTrajectoryAsync(Eight);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            liftLeft.setPower((1920 - liftLeft.getCurrentPosition()) * 0.01);
            liftRight.setPower((1920 - liftLeft.getCurrentPosition()) * 0.01);
        }
        LV.setPosition(1);
        RV.setPosition(0);

        drive.followTrajectory(Nine);

        sleep(100);
        lc.setPosition(0.28);
        rc.setPosition(0.72);
        sleep(250);

        drive.followTrajectory(Ten);

        LV.setPosition(0.05);
        RV.setPosition(0.95);



        drive.followTrajectoryAsync(Seven);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            liftLeft.setPower((350 - liftLeft.getCurrentPosition()) * 0.01);
            liftRight.setPower((350 - liftLeft.getCurrentPosition()) * 0.01);
        }

        lc.setPosition(0.4);
        rc.setPosition(0.6);
        sleep(300);

        drive.followTrajectoryAsync(Eight);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            liftLeft.setPower((1920 - liftLeft.getCurrentPosition()) * 0.01);
            liftRight.setPower((1920 - liftLeft.getCurrentPosition()) * 0.01);
        }
        LV.setPosition(1);
        RV.setPosition(0);

        drive.followTrajectory(Nine);

        sleep(100);
        lc.setPosition(0.28);
        rc.setPosition(0.72);
        sleep(250);

        drive.followTrajectory(Ten);

        LV.setPosition(0.05);
        RV.setPosition(0.95);

        if (parkingSpot == 1){
            drive.followTrajectoryAsync(Park1);
            while (opModeIsActive() && drive.isBusy()) {
                drive.update();
                liftLeft.setPower((target - liftLeft.getCurrentPosition()) * 0.01);
                liftRight.setPower((target - liftLeft.getCurrentPosition()) * 0.01);
            }
        } if (parkingSpot == 2){
            drive.followTrajectoryAsync(Park2);
            while (opModeIsActive() && drive.isBusy()) {
                drive.update();
                liftLeft.setPower((target - liftLeft.getCurrentPosition()) * 0.01);
                liftRight.setPower((target - liftLeft.getCurrentPosition()) * 0.01);
            }
        } if (parkingSpot == 3){
            drive.followTrajectoryAsync(Park3);
            while (opModeIsActive() && drive.isBusy()) {
                drive.update();
                liftLeft.setPower((target - liftLeft.getCurrentPosition()) * 0.01);
                liftRight.setPower((target - liftLeft.getCurrentPosition()) * 0.01);
            }
        }
        sleep(7000);
    }
}


