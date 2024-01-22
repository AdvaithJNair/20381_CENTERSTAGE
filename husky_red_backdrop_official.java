package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.Servo;


import java.util.List;
import java.util.ArrayList;
/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *]
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add thi OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "CENTERSTAGE RED BACKDROP", group = "Robot")
public class husky_lens_blue_backdrop extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackRight = null;
    private DcMotor liftRightMotor = null;

    private DcMotor liftLeftMotor = null;
    private Servo RightClawServo;
    private Servo LeftClawServo;
    private Servo rightliftServo;

    private int frontLeftPos;
    private int frontRightPos;
    private int backLeftPos;
    private int backRightPos;

    private Servo test;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    static final double FEET_PER_METER = 3.28084;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        liftRightMotor = hardwareMap.dcMotor.get("liftRightMotor");
        liftLeftMotor = hardwareMap.dcMotor.get("liftLeftMotor");

        rightliftServo = hardwareMap.get(Servo.class, "rightliftServo");
        LeftClawServo = hardwareMap.get(Servo.class, "LeftClawServo");
        RightClawServo = hardwareMap.get(Servo.class, "RightClawServo");
        test = hardwareMap.get(Servo.class, "test");

        //Reversing motors to drive forward.
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);

        liftLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //Resets the motor encoders.
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Starts the motors running using encoders.
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftPos = 0;
        backLeftPos = 0;
        frontRightPos = 0;
        backRightPos = 0;

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        rateLimit.expire();
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();

        RightClawServo.setPosition(0.8);
        rightliftServo.setPosition(0.4);
        test.setPosition(1);

        waitForStart();

        resetRuntime();
        while (getRuntime() < 1.3)
        {
            liftLeftMotor.setPower(0.15);
            liftRightMotor.setPower(0.15);
            telemetry.addData("time:", getRuntime());

        }

        liftLeftMotor.setPower(0.001);
        liftRightMotor.setPower(0.001);

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);

                /*
                encoderDrive(DRIVE_SPEED, 7, -7, -7, 7, 5.0);
                test.setPosition(0);
                encoderDrive(DRIVE_SPEED, -2, -2, -2, -2, 5.0);

                encoderDrive(DRIVE_SPEED, -8, 8, 8, -8, 5.0);

                encoderDrive(DRIVE_SPEED, -3.5, 3.5, 3.5, -3.5, 5.0);

                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // lift encoder function (see below)
                sleep(1000);

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                DRIVE_SPEED = 0.3;

                encoderDrive(DRIVE_SPEED, -4, 4, -4, 4, 5.0);

                encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 5.0);



                sleep(1000);
                encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 5.0);



                 */
            runtime.reset();
            if (blocks.length == 0) {
                encoderDrive(DRIVE_SPEED, -13, -13, -13, -13, 5.0);
                encoderDrive(DRIVE_SPEED, 7, -7, -7, 7, 5.0);

                sleep(300);

                encoderDrive(DRIVE_SPEED, -2, -2, -2, -2, 5.0);

                test.setPosition(0);

                sleep(1000);

                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 5.0);
                encoderDrive(DRIVE_SPEED, -7, 7, 7, -7, 5.0);
                encoderDrive(DRIVE_SPEED, -7, 7, 7, -7, 5.0);

                encoderDrive(DRIVE_SPEED, -14, -14, -14, -14, 5.0);
                RightClawServo.setPosition(0.5);

                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 5.0);
                sleep(1000);

                terminateOpModeNow();

            } else {
                for (int i = 0; i < blocks.length; i++) {
                    HuskyLens.Block block = blocks[i];
                    telemetry.addData("Block", block.toString());

                    // block.id 1 is red, block.id 2 is blue

                    // 320 is the the resolution width
                    double thirdWidth = 320 / 3.0;

                    if (block.x > thirdWidth && block.x < 2 * thirdWidth && (block.id == 1 || block.id == 2)) {
                        telemetry.addData("Position", "Middle");

                        encoderDrive(DRIVE_SPEED, -14, -14, -14, -14, 5.0);
                        test.setPosition(0);

                        sleep(1000);

                        encoderDrive(DRIVE_SPEED, 2.5, 2.5, 2.5, 2.5, 5.0);
                        encoderDrive(DRIVE_SPEED, -7, 7, 7, -7, 5.0);

                        sleep(1000);

                        encoderDrive(DRIVE_SPEED, -15, -15, -15, -15, 5.0);
                        RightClawServo.setPosition(0.5);
                        encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 5.0);

                        sleep(1000);

                        terminateOpModeNow();

                    } else if (block.x > 2 * thirdWidth && (block.id == 1 || block.id == 2)) {
                        telemetry.addData("Position", "Right");

                        // right encoder code

                        encoderDrive(DRIVE_SPEED, -13, -13, -13, -13, 5.0);
                        encoderDrive(DRIVE_SPEED, -7, 7, 7, -7, 5.0);

                        sleep (300);

                        encoderDrive(DRIVE_SPEED, -3, -3, -3, -3, 5.0);
                        test.setPosition(0);

                        encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 5.0);

                        encoderDrive(DRIVE_SPEED, 7, -7, -7, 7, 5.0);
                        encoderDrive (DRIVE_SPEED, 4, 4, 4, 4, 5.0);
                        encoderDrive(DRIVE_SPEED, -7, 7, 7, -7, 5.0);
                        encoderDrive(DRIVE_SPEED, -16, -16, -16, -16, 5.0);


                        RightClawServo.setPosition(0.5);
                        encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 5.0);
                        terminateOpModeNow();


                        /*
                        encoderDrive(DRIVE_SPEED, 12, 12, 12, 12, 5.0);
                        encoderDrive(DRIVE_SPEED, -6, 6, -6, 6, 5.0);
                        test.setPosition(0);
                        sleep(2000);
                        encoderDrive(DRIVE_SPEED, -2, -2, -2, -2, 5.0);
                        encoderDrive(DRIVE_SPEED, 19, -19, 19, -19, 5.0);
                        encoderDrive(DRIVE_SPEED, -3.5, 3.5, 3.5, -3.5, 5.0);
                        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        // lift encoder function (see below)
                        sleep(1000);

                        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                        DRIVE_SPEED = 0.3;

                        encoderDrive(DRIVE_SPEED, -4, 4, -4, 4, 5.0);

                        encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 5.0);

                        LeftClawServo.setPosition(1);

                        sleep(1000);
                        encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 5.0);

                         */

                        terminateOpModeNow();

                    }

                    /*
                    encoderDrive(DRIVE_SPEED, 12, 12, 12, 12, 5.0);
                    encoderDrive(DRIVE_SPEED, 6, -6, 6, -6, 5.0);
                    test.setPosition(0);
                    sleep(2000);

                    encoderDrive(DRIVE_SPEED, 12, -12, 12, -12, 5.0);
                    encoderDrive(DRIVE_SPEED, -5, -5, 5, 5, 5.0);

                    // lift encoder function (see below)
                    liftEncoderDrive(DRIVE_SPEED, 3, 3, 5.0);

                    telemetry.addData("Position", "Left");

                    */
                }
            }
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
            newFrontRightTarget = motorFrontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackRightTarget = motorBackRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
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

    public void liftEncoderDrive (double speed, double left_lift_inches, double right_lift_inches, double timeouts){

        int leftLiftTarget;
        int rightLiftTarget;


        if (opModeIsActive()) {
            leftLiftTarget = motorFrontLeft.getCurrentPosition() + (int) (left_lift_inches * COUNTS_PER_INCH);
            rightLiftTarget = motorBackLeft.getCurrentPosition() + (int) (right_lift_inches * COUNTS_PER_INCH);

            liftLeftMotor.setTargetPosition(leftLiftTarget);
            liftRightMotor.setTargetPosition(rightLiftTarget);

            liftLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            liftLeftMotor.setPower(0.5);
            liftRightMotor.setPower(0.5);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeouts) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", leftLiftTarget, rightLiftTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        liftLeftMotor.getCurrentPosition(), liftRightMotor.getCurrentPosition());
                telemetry.update();
            }

            liftLeftMotor.setPower(0);
            liftRightMotor.setPower(0);

            liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
}
