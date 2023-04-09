package org.firstinspires.ftc.teamcode.custom_util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ServoTesterHelper extends LinearOpMode {
    private Servo LV, RV, C, W;

    @Override
    public void runOpMode() throws InterruptedException {
        LV = hardwareMap.get(Servo.class, "lv");
        RV = hardwareMap.get(Servo.class, "rv");
        W  = hardwareMap.get(Servo.class, "ld");
        C = hardwareMap.get(Servo.class, "claw");
        Servo[] servos = {LV, RV, W, C};
        waitForStart();
        double prevTime = System.currentTimeMillis();
        int current_servo_index = 0;
        while (opModeIsActive()) {
            double currentPower = gamepad1.left_stick_x;
            double currentTime = System.currentTimeMillis();
            if (currentTime - prevTime > 500) {
                if (gamepad1.dpad_right) {

                    current_servo_index += 1;
                    prevTime = currentTime;


                } else if (gamepad1.dpad_left) {
                    current_servo_index -= 1;
                    prevTime = currentTime;

                }
            }
            if (current_servo_index == -1) {
                current_servo_index = servos.length -1;
            } else if (current_servo_index == servos.length) {
                current_servo_index = 0;
            }

            servos[current_servo_index].setPosition(currentPower);
            telemetry.addLine("current index:" + current_servo_index);
            telemetry.addLine("current servo port number:" + servos[current_servo_index].getPortNumber());
            telemetry.addLine("current Servo:" + servos[current_servo_index].getDeviceName());
            telemetry.update();
        }

    }
}
