package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.Servo;
import org.opencv.core.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;

import java.util.List;
import java.util.ArrayList;

@Autonomous(name = "CENTERSTAGE AUTONOMOUS BLUE FINAL", group = "Robot")
public class autonomous_blue extends LinearOpMode {
    OpenCvCamera camera;
    autonomous_pipeline pipeline = new autonomous_pipeline();
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackRight = null;

    private Servo RightClawServo;
    private Servo LeftClawServo;
    private Servo rightliftServo;


    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    static final double FEET_PER_METER = 3.28084;

    private ElapsedTime runtime = new ElapsedTime();

    // lens intrinsics
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    @Override
    public void runOpMode() {

        //Motor mapping for the drive motors
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        rightliftServo = hardwareMap.get(Servo.class, "rightliftServo");
        LeftClawServo = hardwareMap.get(Servo.class, "LeftClawServo");
        RightClawServo = hardwareMap.get(Servo.class, "RightClawServo");
        //Reversing motors to drive forward.
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        //Resets the motor encoders.
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Starts the motors running using encoders.
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initializes preview and the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // starts streaming the camera at 800x448 resolution
                // automatically calls the processFrame function with the startStrraeming function
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("There was an error!", errorCode);
            }
        });

        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            Rect rectangle = pipeline.getDetectedRectangle();

            int left_boundary = 265;
            int middle_boundary = 530;

            telemetry.addData ("Average x-value: ", (rectangle.x + rectangle.width) / 2);

            if ( (rectangle.x + rectangle.width) / 2 <= left_boundary){ // game element is on the left
                // encoder code
                // april tag hard code
            } else if ((rectangle.x + rectangle.width) / 2 >= left_boundary && (rectangle.x + rectangle.width) / 2 <= middle_boundary){ // game element is in the middle
                // encoder code
                // april tag hard code
            } else { // game element is on the right
                // encoder code
                // april tag hard code
            }

        }
    }

    class autonomous_pipeline extends OpenCvPipeline {

        public Rect boundingRect = new Rect();

        // process frame function acts as an intrinsic constructor
        public Mat processFrame(Mat input) {

            Mat hsvImage = new Mat();
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);


            /* not if these are correct bound values
            feel free to change if they don't fit the game element
             */

            Scalar lowerBound = new Scalar(50, 100, 150);
            Scalar upperBound = new Scalar(0, 0, 255);

            // converts mat into HSV for image processing
            Mat mask = new Mat();
            Core.inRange(hsvImage, lowerBound, upperBound, mask);

            // contours for rectangle detection
            Mat hierarchy = new Mat();
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = -1;
            int maxAreaIdx = -1;

            for (int i = 0; i < contours.size(); i++) {

                // contextualized version of Kadane's Algorithm
                double area = Imgproc.contourArea(contours.get(i));
                if (area > maxArea) {
                    maxArea = area;
                    maxAreaIdx = i;
                }
            }

            if (maxAreaIdx != -1) {
                // draws a rectangle in the frame
                boundingRect = Imgproc.boundingRect(contours.get(maxAreaIdx));
                Imgproc.rectangle(input, new Point(boundingRect.x, boundingRect.y),
                        new Point(boundingRect.x + boundingRect.width, boundingRect.y + boundingRect.height),
                        new Scalar(0, 255, 0), 2);
            }

            return input;
        }

        public Rect getDetectedRectangle (){
            return boundingRect;
        }

    }

    public void encoderDrive(double speed,
                             double frontLeftInches, double backLeftInches, double frontRightInches, double backRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newBackLeftTarget = motorBackLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = motorFrontLeft.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackRightTarget = motorBackLeft.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorBackLeft.setTargetPosition(newBackLeftTarget);
            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorBackRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorFrontLeft.setPower(Math.abs(speed / 2));
            motorBackLeft.setPower(Math.abs(speed / 2));
            motorFrontRight.setPower(Math.abs(speed / 2));
            motorBackRight.setPower(Math.abs(speed / 2));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        motorFrontLeft.getCurrentPosition(), motorFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move.
        }
    }
}

