package org.firstinspires.ftc.teamcode.outdated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp
public class M3_TeleOp_Apollo2 extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor liftRight = null, liftLeft = null;
    private Servo LV, RV, rc, lc;
    private DcMotor LF = null, RF = null, LR = null, RR = null;


    @Override
    public void runOpMode() {

        liftRight = hardwareMap.get(DcMotor.class, "lrm");
        liftLeft = hardwareMap.get(DcMotor.class, "llm");

        //Servo hardware mapping
        LV = hardwareMap.get(Servo.class, "lv");
        RV = hardwareMap.get(Servo.class, "rv");
        lc = hardwareMap.get(Servo.class, "lc");
        rc = hardwareMap.get(Servo.class, "rc");

        //Motor hardware mapping
        LF  = hardwareMap.get(DcMotor.class, "lf");
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
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double LEVEL = 0;
        double target = 0;

        waitForStart();

        //liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        while (opModeIsActive()) {

            //liftLeft.setPower(gamepad1.right_stick_x);
            //liftRight.setPower(gamepad1.right_stick_x);

            telemetry.addData("leftpos", liftLeft.getCurrentPosition());
            telemetry.addData("rightpos", liftRight.getCurrentPosition());
            telemetry.update();

            if (gamepad2.dpad_up){
                LEVEL = 3;
                target = 1830;
                LV.setPosition(1);
                RV.setPosition(0);
            } if (gamepad2.dpad_left){
                LEVEL = 2;
                target = 750;
                LV.setPosition(1);
                RV.setPosition(0);
            } if (gamepad2.dpad_right){
                LEVEL = 1;
                target = 42;
                LV.setPosition(0.8);
                RV.setPosition(0.2);
            } if (gamepad2.dpad_down){
                LEVEL = 0;
                target = 20;
                LV.setPosition(0.03);
                RV.setPosition(0.97);
            }

            liftLeft.setPower((target - liftLeft.getCurrentPosition()) * 0.01);
            liftRight.setPower((target - liftLeft.getCurrentPosition()) * 0.01);

            if (gamepad1.a) {
                lc.setPosition(0.4);
                rc.setPosition(0.6);
            } if (gamepad1.b) {
                lc.setPosition(0.28);
                rc.setPosition(0.72);
            }

            //Controls for the drivetrain - Gamepad 1
            double y = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double theta = gamepad1.right_stick_x;

            if (gamepad2.right_bumper){
                LF.setPower(Range.clip(y + x + theta,-1, 1) / 2.25);
                RF.setPower(Range.clip(y - x - theta,-1, 1) / 2.25);
                LR.setPower(Range.clip(y - x + theta,-1, 1) / 2.25);
                RR.setPower(Range.clip(y + x - theta,-1, 1) / 2.25);
            } else {
                LF.setPower(Range.clip(y + x + theta, -1, 1) / 1.0);
                RF.setPower(Range.clip(y - x - theta, -1, 1) / 1.0);
                LR.setPower(Range.clip(y - x + theta, -1, 1) / 1.0);
                RR.setPower(Range.clip(y + x - theta, -1, 1) / 1.0);
            }
        }
    }
}
