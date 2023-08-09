package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class ArtemisV_Right_High_Worlds extends LinearOpMode {

    //DcMotor lm;
    private DcMotor lift1 = null, lift2 = null, turret = null;
    private Servo LV, RV, C, W;
    OpenCvCamera webcam;
    VisionTest.SamplePipeline pipeline;

    double liftTarget = 0;
    double turretTarget = 0;
    double liftKp = 0.15;


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //lm = hardwareMap.get(DcMotor.class, "lm");

        lift1 = hardwareMap.get(DcMotor.class, "l1");
        lift2 = hardwareMap.get(DcMotor.class, "l2");
        turret = hardwareMap.get(DcMotor.class, "turret");


        //Servo hardware mapping
        LV = hardwareMap.get(Servo.class, "lv");
        RV = hardwareMap.get(Servo.class, "rv");

        W = hardwareMap.get(Servo.class, "ld");
        C = hardwareMap.get(Servo.class, "claw");

        lift1.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotor.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new VisionTest.SamplePipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        Pose2d start = new Pose2d(-36, 63, toRadians(-90));
        Trajectory one = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-36, 31), Math.toRadians(-90))
                .addTemporalMarker(0, () -> LV.setPosition(0.5))
                .addTemporalMarker(0, () -> RV.setPosition(0.5))
                .addDisplacementMarker(35, () -> {
                    liftTarget = 680;
                    turretTarget = 580;
                    liftKp = 0.012;
                })
                .splineTo(new Vector2d(-31, 15), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-28, 9), Math.toRadians(0))
                .build();

        Trajectory two = drive.trajectoryBuilder(one.end())
                .lineToLinearHeading(new Pose2d(-58.5, 13, toRadians(0)))
                .addDisplacementMarker(5, () -> {
                    liftTarget = 145;
                    turretTarget = 0;
                    liftKp = 0.0015;
                })
                .addDisplacementMarker(5, () -> LV.setPosition(0.1))
                .addDisplacementMarker(5, () -> RV.setPosition(0.9))
                .addDisplacementMarker(12, () -> W.setPosition(0))
                .build();

        Trajectory three = drive.trajectoryBuilder(two.end())
                .splineToConstantHeading(new Vector2d(-34, 16), Math.toRadians(0))
                .addDisplacementMarker(2, () -> {
                    liftTarget = 680;
                    turretTarget = 530;
                    liftKp = 0.012;
                })
                .addDisplacementMarker(2, () -> LV.setPosition(0.5))
                .addDisplacementMarker(2, () -> RV.setPosition(0.5))
                .addDisplacementMarker(2, () -> W.setPosition(0.68))
                .splineToConstantHeading(new Vector2d(-28, 9), Math.toRadians(0))
                .build();

        Trajectory four = drive.trajectoryBuilder(three.end())
                .lineToLinearHeading(new Pose2d(-58.5, 13, Math.toRadians(0)))
                .addDisplacementMarker(5, () -> {
                    liftTarget = 115;
                    turretTarget = 0;
                    liftKp = 0.0015;
                })
                .addDisplacementMarker(2, () -> LV.setPosition(0.1))
                .addDisplacementMarker(2, () -> RV.setPosition(0.9))
                .addDisplacementMarker(10, () -> W.setPosition(0))
                .build();

        Trajectory five = drive.trajectoryBuilder(four.end())
                .splineToConstantHeading(new Vector2d(-34, 16), Math.toRadians(0))
                .addDisplacementMarker(2, () -> {
                    liftTarget = 680;
                    turretTarget = 530;
                    liftKp = 0.012;
                })
                .addDisplacementMarker(2, () -> LV.setPosition(0.5))
                .addDisplacementMarker(2, () -> RV.setPosition(0.5))
                .addDisplacementMarker(2, () -> W.setPosition(0.68))
                .splineToConstantHeading(new Vector2d(-28, 9), Math.toRadians(0))
                .build();

        Trajectory six = drive.trajectoryBuilder(five.end())
                .lineToLinearHeading(new Pose2d(-58.5, 12, Math.toRadians(0)))
                .addDisplacementMarker(5, () -> {
                    liftTarget = 75;
                    turretTarget = 0;
                    liftKp = 0.0015;
                })
                .addDisplacementMarker(2, () -> LV.setPosition(0.1))
                .addDisplacementMarker(2, () -> RV.setPosition(0.9))
                .addDisplacementMarker(10, () -> W.setPosition(0))
                .build();

        Trajectory seven = drive.trajectoryBuilder(six.end())
                .splineToConstantHeading(new Vector2d(-34, 16), Math.toRadians(0))
                .addDisplacementMarker(2, () -> {
                    liftTarget = 680;
                    turretTarget = 580;
                    liftKp = 0.012;
                })
                .addDisplacementMarker(2, () -> LV.setPosition(0.5))
                .addDisplacementMarker(2, () -> RV.setPosition(0.5))
                .addDisplacementMarker(2, () -> W.setPosition(0.68))
                .splineToConstantHeading(new Vector2d(-28, 9), Math.toRadians(0))
                .build();

        Trajectory eight = drive.trajectoryBuilder(seven.end())
                .lineToLinearHeading(new Pose2d(-58.5, 12, Math.toRadians(0)))
                .addDisplacementMarker(5, () -> {
                    liftTarget = 45;
                    turretTarget = 0;
                    liftKp = 0.0015;
                })
                .addDisplacementMarker(5, () -> LV.setPosition(0.07))
                .addDisplacementMarker(5, () -> RV.setPosition(0.93))
                .addDisplacementMarker(10, () -> W.setPosition(0))
                .build();

        Trajectory nine = drive.trajectoryBuilder(eight.end())
                .splineToConstantHeading(new Vector2d(-32, 15), Math.toRadians(0))
                .addDisplacementMarker(2, () -> {
                    liftTarget = 680;
                    turretTarget = 540;
                    liftKp = 0.012;
                })
                .addDisplacementMarker(2, () -> LV.setPosition(0.5))
                .addDisplacementMarker(2, () -> RV.setPosition(0.5))
                .addDisplacementMarker(2, () -> W.setPosition(0.68))
                .splineToConstantHeading(new Vector2d(-28, 9), Math.toRadians(0))
                .build();

        Trajectory ten = drive.trajectoryBuilder(nine.end())
                .lineToLinearHeading(new Pose2d(-58.5, 12, Math.toRadians(0)))
                .addDisplacementMarker(5, () -> {
                    liftTarget = -100;
                    turretTarget = 0;
                    liftKp = 0.0015;
                })
                .addDisplacementMarker(5, () -> LV.setPosition(0.07))
                .addDisplacementMarker(5, () -> RV.setPosition(0.93))
                .addDisplacementMarker(10, () -> W.setPosition(0))
                .build();

        Trajectory eleven = drive.trajectoryBuilder(ten.end())
                .splineToConstantHeading(new Vector2d(-32, 14), Math.toRadians(0))
                .addDisplacementMarker(2, () -> {
                    liftTarget = 680;
                    turretTarget = 500;
                    liftKp = 0.012;
                })
                .addDisplacementMarker(2, () -> LV.setPosition(0.5))
                .addDisplacementMarker(2, () -> RV.setPosition(0.5))
                .addDisplacementMarker(2, () -> W.setPosition(0.68))
                .splineToConstantHeading(new Vector2d(-28, 9), Math.toRadians(0))
                .build();

        Trajectory twelve = drive.trajectoryBuilder(eleven.end())
                .lineToLinearHeading(new Pose2d(-35, 9, toRadians(0)))
                .addDisplacementMarker(6, () -> {
                    liftTarget = -150;
                    turretTarget = 0;
                    liftKp = 0.0015;
                })
                .build();

        Trajectory twelve3 = drive.trajectoryBuilder(twelve.end())
                .lineToLinearHeading(new Pose2d(-63, 17, toRadians(0)))
                .addDisplacementMarker(6, () -> {
                    liftTarget = -150;
                    turretTarget = 0;
                    liftKp = 0.002;
                })
                .build();

        Trajectory twelve2 = drive.trajectoryBuilder(twelve.end())
                .lineToLinearHeading(new Pose2d(-35, 28, toRadians(-90)))
                .addDisplacementMarker(6, () -> {
                    liftTarget = -150;
                    turretTarget = 0;
                    liftKp = 0.002;
                })
                .build();

        Trajectory twelve1 = drive.trajectoryBuilder(twelve.end())
                .lineToLinearHeading(new Pose2d(-11, 19, toRadians(-90)))
                .addDisplacementMarker(6, () -> {
                    liftTarget = -150;
                    turretTarget = 0;
                    liftKp = 0.002;
                })
                .build();

        while (!isStarted()) {

            //LV.setPosition(0.28);
            //RV.setPosition(0.72);
            LV.setPosition(0.25);
            RV.setPosition(0.75);
            W.setPosition(0.68);

            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("Location", pipeline.parkLocation);
            telemetry.update();
            if (gamepad1.dpad_left) {

                C.setPosition(0.45);

            } else {
                // 0.62
                C.setPosition(0.62  );

            }

        }

        if (isStopRequested()) return;

        drive.setPoseEstimate(start);

        sleep(200);
        int parkingSpot = pipeline.parkLocation;
        webcam.stopStreaming();

        drive.followTrajectoryAsync(one);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        LV.setPosition(0.62);
        RV.setPosition(0.38);
        W.setPosition(0.68);
        sleep(150);
        C.setPosition(0.45);


        drive.followTrajectoryAsync(two);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        C.setPosition(0.62);
        sleep(300);
        liftTarget = 600;
        lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        sleep(220);

        drive.followTrajectoryAsync(three);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        liftTarget = 450;
        lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -0.001);
        lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -0.001);
        LV.setPosition(0.62);
        RV.setPosition(0.38);
        W.setPosition(0.68);
        sleep(150);
        C.setPosition(0.4);

        drive.followTrajectoryAsync(four);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        C.setPosition(0.62);
        sleep(300);
        liftTarget = 600;
        lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        sleep(220);

        drive.followTrajectoryAsync(five);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        liftTarget = 450;
        lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -0.001);
        lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -0.001);
        LV.setPosition(0.62);
        RV.setPosition(0.38);
        W.setPosition(0.68);
        sleep(150);
        C.setPosition(0.4);

        drive.followTrajectoryAsync(six);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        C.setPosition(0.62);
        sleep(300);
        liftTarget = 550;
        lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        sleep(220);

        drive.followTrajectoryAsync(seven);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        liftTarget = 450;
        lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -0.001);
        lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -0.001);
        LV.setPosition(0.62);
        RV.setPosition(0.38);
        W.setPosition(0.68);
        sleep(150);
        C.setPosition(0.4);

        drive.followTrajectoryAsync(eight);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        C.setPosition(0.62);
        sleep(300);
        liftTarget = 500;
        lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        sleep(220);

        drive.followTrajectoryAsync(nine);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        liftTarget = 450;
        lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -0.001);
        lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -0.001);
        LV.setPosition(0.62);
        RV.setPosition(0.38);
        W.setPosition(0.68);
        sleep(150);
        C.setPosition(0.4);

        drive.followTrajectoryAsync(ten);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        C.setPosition(0.62);
        sleep(200);

        drive.followTrajectoryAsync(eleven);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        LV.setPosition(0.62);
        RV.setPosition(0.38);
        W.setPosition(0.68);
        sleep(150);
        C.setPosition(0.55);

        drive.followTrajectoryAsync(twelve);
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
        }

        LV.setPosition(0.32);
        RV.setPosition(0.68);
        W.setPosition(0);
        sleep(150);
        C.setPosition(0.3);

        if (parkingSpot == 1 || parkingSpot == 0) {
                drive.followTrajectoryAsync(twelve1);
                while (opModeIsActive() && drive.isBusy()) {
                    drive.update();
                    turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
                    lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
                    lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
                }
                sleep(5000);

        }

        if (parkingSpot == 2) {
                drive.followTrajectoryAsync(twelve2);
                while (opModeIsActive() && drive.isBusy()) {
                    drive.update();
                    turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
                    lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
                    lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
                }
                sleep(5000);

        }

        if (parkingSpot == 3) {
                drive.followTrajectoryAsync(twelve3);
                while (opModeIsActive() && drive.isBusy()) {
                    drive.update();
                    turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.008);
                    lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
                    lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
                }
                sleep(5000);

        }

    }
}