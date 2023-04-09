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

@Autonomous
@Disabled
public class M2_RightAuto_Apollo2 extends LinearOpMode {

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
                .lineToLinearHeading(new Pose2d(-38, 12, toRadians(-227)))
                .build();
        Pose2d eTwo = new Pose2d(-38, 12, toRadians(-230));

        Trajectory Three = drive.trajectoryBuilder(eTwo)
                .lineToLinearHeading(new Pose2d(-53, 11, toRadians(-178)))
                .build();
        Pose2d eThree = new Pose2d(-53, 11, toRadians(-178));

        Trajectory Four = drive.trajectoryBuilder(eThree)
                .lineToLinearHeading(new Pose2d(-38, 12, toRadians(-231)))
                .build();
        Pose2d eFour = new Pose2d(-38, 12, toRadians(-231));

        Trajectory Five = drive.trajectoryBuilder(eFour)
                .lineToLinearHeading(new Pose2d(-54, 12, toRadians(-178)))
                .build();
        Pose2d eFive = new Pose2d(-54, 12, toRadians(-178));

        Trajectory Six = drive.trajectoryBuilder(eFive)
                .lineToLinearHeading(new Pose2d(-38, 12, toRadians(-232)))
                .build();
        Pose2d eSix = new Pose2d(-38, 12, toRadians(-235));

        Trajectory Seven = drive.trajectoryBuilder(eSix)
                .lineToLinearHeading(new Pose2d(-50, 12, toRadians(-178)))
                .build();
        Pose2d eSeven = new Pose2d(-50, 12, toRadians(-178));

        Trajectory Eight = drive.trajectoryBuilder(eSeven)
                .lineToLinearHeading(new Pose2d(-61, 12, toRadians(-180)))
                .build();
        Pose2d eEight = new Pose2d(-61, 12, toRadians(-180));

        Trajectory Nine = drive.trajectoryBuilder(eEight)
                .lineToLinearHeading(new Pose2d(-42, 12, toRadians(-180)))
                .build();
        Pose2d eNine = new Pose2d(-42, 12, toRadians(-180));

        Trajectory Ten = drive.trajectoryBuilder(eNine)
                .lineToLinearHeading(new Pose2d(-38, 12, toRadians(-240)))
                .build();
        Pose2d eTen = new Pose2d(-38, 12, toRadians(-240));

        while (!isStarted()){
            telemetry.addData("Location", pipeline.parkLocation);
            telemetry.update();

            lc.setPosition(0.4);
            rc.setPosition(0.6);
        }

        if (isStopRequested()) return;

        drive.setPoseEstimate(start);

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
        LV.setPosition(0.85);
        RV.setPosition(0.15);

        drive.followTrajectory(Four);

        sleep(100);
        lc.setPosition(0.28);
        rc.setPosition(0.72);
        sleep(250);

        drive.followTrajectory(Three);

        LV.setPosition(0.42);
        RV.setPosition(0.58);
        sleep(250);
        lc.setPosition(0.4);
        rc.setPosition(0.6);
        sleep(200);
        LV.setPosition(0.85);
        RV.setPosition(0.15);

        drive.followTrajectory(Four);

        sleep(100);
        lc.setPosition(0.28);
        rc.setPosition(0.72);
        sleep(200);

        drive.followTrajectory(Five);

        LV.setPosition(0.32);
        RV.setPosition(0.68);
        sleep(350);
        lc.setPosition(0.4);
        rc.setPosition(0.6);
        sleep(300);
        LV.setPosition(0.85);
        RV.setPosition(0.15);

        drive.followTrajectory(Six);

        sleep(100);
        lc.setPosition(0.28);
        rc.setPosition(0.72);
        sleep(250);

        drive.followTrajectory(Five);

        LV.setPosition(0.31);
        RV.setPosition(0.69);
        sleep(350);
        lc.setPosition(0.39);
        rc.setPosition(0.61);
        sleep(300);
        LV.setPosition(0.85);
        RV.setPosition(0.15);

        drive.followTrajectory(Six);

        sleep(100);
        lc.setPosition(0.28);
        rc.setPosition(0.72);
        sleep(250);

        drive.followTrajectory(Seven);

        LV.setPosition(0.05);
        RV.setPosition(0.95);

        drive.followTrajectory(Eight);

        sleep(50);
        lc.setPosition(0.4);
        rc.setPosition(0.6);
        sleep(150);

        drive.followTrajectoryAsync(Nine);

        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            liftLeft.setPower((50 - liftLeft.getCurrentPosition()) * 0.01);
            liftRight.setPower((50 - liftLeft.getCurrentPosition()) * 0.01);
        }

        //sleep(250);
        LV.setPosition(0.85);
        RV.setPosition(0.15);
        sleep(250);

        drive.followTrajectory(Ten);

        sleep(100);
        lc.setPosition(0.28);
        rc.setPosition(0.72);
        sleep(250);

        sleep(2500);
    }
}


