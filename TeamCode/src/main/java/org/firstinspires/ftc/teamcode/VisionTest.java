package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class VisionTest extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Location", pipeline.parkLocation);
            telemetry.addData("Zone One", pipeline.getAverageOne());
            telemetry.addData("Zone Two", pipeline.getAverageTwo());
            telemetry.update();
            sleep(50);
        }
    }

    public static class SamplePipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(0, 0, 255);

        public static int THRESHOLD = 114;
        public int parkLocation = 0;

        Point topLeftOne = new Point(169, 95);
        Point bottomRightOne = new Point(198, 150);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        private volatile int average1, average2;
        private volatile TYPE type = TYPE.EMPTY;

        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat input1) {
            inputToCb(input1);

            region1_Cb = Cb.submat(new Rect(topLeftOne, bottomRightOne));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            average1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(input, topLeftOne, bottomRightOne, BLUE, 2);

            if (average1 < THRESHOLD){
                type = TYPE.ELEMENT;
                parkLocation = 2;
            } if (average1 > THRESHOLD && average1 < 135){
                type = TYPE.ELEMENT;
                parkLocation = 1;
            } if (average1 > 135) {
                type = TYPE.ELEMENT;
                parkLocation = 3;
            }

            return input;
        }

        public TYPE getType() {
            return type;
        }

        public int getAverageOne() {
            return average1;
        }

        public int getAverageTwo() {
            return average2;
        }

        public enum TYPE {
            EMPTY, ELEMENT
        }
    }
}

