package org.firstinspires.ftc.teamcode.outdated;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class ArtemisIV_Regionals_TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor lift1 = null, lift2 = null, turret = null;
    private Servo LV, RV, C, LD, RD;
    private DcMotor LF = null, RF = null, LR = null, RR = null;


    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {

        lift1 = hardwareMap.get(DcMotor.class, "l1");
        lift2 = hardwareMap.get(DcMotor.class, "l2");
        turret = hardwareMap.get(DcMotor.class, "turret");


        //Servo hardware mapping
        LV = hardwareMap.get(Servo.class, "lv");
        RV = hardwareMap.get(Servo.class, "rv");
        RD = hardwareMap.get(Servo.class, "ld");
        LD = hardwareMap.get(Servo.class, "rd");
        C = hardwareMap.get(Servo.class, "claw");

        //Motor hardware mapping
        LF = hardwareMap.get(DcMotor.class, "lf");
        RF = hardwareMap.get(DcMotor.class, "rf");
        LR = hardwareMap.get(DcMotor.class, "lr");
        RR = hardwareMap.get(DcMotor.class, "rr");

        //Motor parameters
        LF.setDirection(DcMotor.Direction.REVERSE);
        LR.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.FORWARD);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Motor parameters
        lift1.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotor.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double liftTarget = 0;
        double turretTarget = 0;
        double liftKp = 0.15;

        //LV.setPosition(0.5);
        //RV.setPosition(0.5);

        waitForStart();

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive()) {


            //Control for lift level
            if (gamepad2.dpad_up) {
                //High Junction Level
                liftTarget = 820;
                liftKp = 0.012;
            }
            if (gamepad2.dpad_left) {
                //Medium Junction Level
                liftTarget = 500;
                liftKp = 0.012;
            }
            if (gamepad2.dpad_right) {
                //Low Junction Level
                liftTarget = 180;
                liftKp = 0.012;
            }
            if (gamepad2.dpad_down) {
                //Reset Lift
                liftTarget = -150;
                liftKp = 0.002;
                LV.setPosition(0.05);
                RV.setPosition(0.95);
                C.setPosition(0.45);
            }

            //Control for V4B pitch
            if (gamepad2.left_bumper) {
                // Raise V4B
                //LD.setPosition(0.75);
                LV.setPosition(0.85);
                RV.setPosition(0.15);
            }
            if (gamepad2.right_bumper) {
                //Extend V4B
                //LD.setPosition(0.75);
                LV.setPosition(0.6);
                RV.setPosition(0.4);
            }

            //Control for turret position
            if (gamepad2.a) {
                turretTarget = 0;
            }
            if (gamepad2.b) {
                turretTarget = -540;
            }
            if (gamepad2.x) {
                turretTarget = 540;
            }
            if (gamepad2.y) {
                turretTarget = 1335;
            }

            //Driver Deposit Controls
            if (gamepad1.a) {
                C.setPosition(0.73);
            }
            if (gamepad1.b) {
                C.setPosition(0.45);
            }
            if (gamepad1.x) {
                liftTarget = liftTarget - 15;
            }

            //Code for Drive1 to Reset Lift and Turret
            if (gamepad1.y) {
                liftTarget = liftTarget + 15;
            }
            if (gamepad1.left_bumper) {
                turretTarget = turretTarget - 15;
            }
            if (gamepad1.right_bumper) {
                turretTarget = turretTarget + 15;
            }

            if (gamepad1.left_trigger > 0.9) {
                LV.setPosition(0.22);
                RV.setPosition(0.78);
                LD.setPosition(1);
                C.setPosition(0.45);
            } else {
                LD.setPosition(0.7);
            }

            turret.setPower((turretTarget - turret.getCurrentPosition()) * 0.02);
            lift2.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);
            lift1.setPower((liftTarget - lift1.getCurrentPosition()) * -liftKp);

            //Controls for the drivetrain - Gamepad 1
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double theta = gamepad1.right_stick_x;

            if (gamepad1.right_trigger > 0.1) {
                LF.setPower(Range.clip(y + x + theta, -1, 1) / 2.25);
                RF.setPower(Range.clip(y - x - theta, -1, 1) / 2.25);
                LR.setPower(Range.clip(y - x + theta, -1, 1) / 2.25);
                RR.setPower(Range.clip(y + x - theta, -1, 1) / 2.25);
            } else {
                LF.setPower(Range.clip(y + x + theta, -1, 1) / 1.0);
                RF.setPower(Range.clip(y - x - theta, -1, 1) / 1.0);
                LR.setPower(Range.clip(y - x + theta, -1, 1) / 1.0);
                RR.setPower(Range.clip(y + x - theta, -1, 1) / 1.0);
            }
        }
    }
}