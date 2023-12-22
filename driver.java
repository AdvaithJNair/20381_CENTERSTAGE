package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="CENTERSTAGE DRIVER CONTROL FINAL", group="Linear Opmode")

public class driver extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declaration and mapping of lift motor.

        // Declaration and mapping of drive motors.
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");


        DcMotor liftRightMotor = hardwareMap.dcMotor.get("liftRightMotor");

        DcMotor liftLeftMotor = hardwareMap.dcMotor.get("liftLeftMotor");

        liftRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Declaration and mapping of the claw Servo
        Servo rightliftServo = hardwareMap.get(Servo.class, "rightliftServo");
        Servo leftliftServo = hardwareMap.get(Servo.class, "leftliftServo");

        Servo LeftClawServo = hardwareMap.get(Servo.class, "LeftClawServo");



        Servo RightClawServo = hardwareMap.get(Servo.class, "RightClawServo");

        Servo planeServo = hardwareMap.get(Servo.class, "planeServo");

        rightliftServo.setPosition(0.05);

        // Reverse the right side motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        liftRightMotor.setTargetPosition(0 );
        liftRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        double position = 0;
        double right1 = 0;
        double left1 = 0;

        boolean left = false;
        boolean right = false;

        while (opModeIsActive()) {

            //Drive Motor Code!
            //Not to be messed with under any circumstances.
            ElapsedTime runtime = new ElapsedTime();
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            // Stupid Code
            double y2 = gamepad2.left_stick_y;
            double x2 = -gamepad2.left_stick_x * 1.1;
            double rx2 = -gamepad2.right_stick_x;

            telemetry.addLine("left stick y" + y);
            telemetry.addLine("left stick x" + x);
            telemetry.addLine("right stick x" + rx);
            telemetry.addLine("servo pos: " + rightliftServo.getPosition());

            telemetry.addLine("lift right: " + liftRightMotor.getPower());
            telemetry.addLine("lift left: " + liftLeftMotor.getPower());

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double denominator2 = Math.max(Math.abs(y2) + Math.abs(x2) + Math.abs(rx2), 1);
            double frontLeftPower2 = (y2 + x2 + rx2) / denominator2;
            double backLeftPower2   = (y2 - x2 + rx2) / denominator2;
            double frontRightPower2 = (y2 - x2 - rx2) / denominator2;
            double backRightPower2 = (y2 + x2 - rx2) / denominator2;


            if(gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_y <= -0.1 || gamepad1.left_stick_x <= -0.1 || gamepad1.right_stick_y >= 0.2 || gamepad1.right_stick_x >= 0.2 || gamepad1.right_stick_y <= -0.2 || gamepad1.right_stick_x <= -0.2  )
            {
                motorFrontLeft.setPower(frontLeftPower * 1.0);
                motorBackLeft.setPower(backLeftPower * 1.0);
                motorFrontRight.setPower(frontRightPower * 1.0);
                motorBackRight.setPower(backRightPower * 1.0);
            }

            else if(gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_x >= 0.1 || gamepad2.left_stick_y <= -0.1 || gamepad2.left_stick_x <= -0.1 || gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_x >= 0.1 || gamepad2.right_stick_y <= -0.1 || gamepad2.right_stick_x <= -0.1  )
            {
                motorFrontLeft.setPower(frontLeftPower2 * 0.4);
                motorBackLeft.setPower(backLeftPower2 * 0.4);
                motorFrontRight.setPower(frontRightPower2 * 0.4);
                motorBackRight.setPower(backRightPower2 * 0.4);
            }

            else
            {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
            //Lift Motor Code
            // Find starting height for lift and find power at that point
            // Find final height and power and map that to the claw orientation (position 0 < x < 1)

            double liftPowerDown = gamepad1.left_trigger;
            double liftPowerUp = gamepad1.right_trigger;

            double liftPowerDown2 = gamepad2.left_trigger;
            double liftPowerUp2 = gamepad2.right_trigger;

            if (liftPowerDown >= 0.1) {
                liftRightMotor.setPower(liftPowerDown * 0.50);
                liftLeftMotor.setPower(liftPowerDown * 0.50);
            }
            else if (liftPowerUp >= 0.1)
            {
                liftRightMotor.setPower(-liftPowerUp * 0.70);
                liftLeftMotor.setPower(-liftPowerUp * 0.70);
            }
            else
            {
                liftRightMotor.setPower(-0.07);
                liftLeftMotor.setPower(-0.07);
            }

            //Stupid Code

            if (liftPowerDown2 >= 0.1) {
                liftRightMotor.setPower(liftPowerDown2 * 0.50);
                liftLeftMotor.setPower(liftPowerDown2 * 0.5);
            }
            else if (liftPowerUp2 >= 0.1)
            {
                liftRightMotor.setPower(-liftPowerUp2 * 0.70);
                liftLeftMotor.setPower(-liftPowerUp2 * 0.70);
            } else {
                liftRightMotor.setPower(-0.07);
                liftLeftMotor.setPower(-0.07);
            }

            // double liftServoPowerUp  = aamepad2.right_trigger;
            // double liftServoPowerDown = // gamepad2.left_trigger;


            if (gamepad1.left_bumper == true && position > 0 || gamepad2.left_bumper == true && position > 0){
                position -= 0.1;
                sleep(100);
                rightliftServo.setPosition(position);
            }

            if (gamepad1.right_bumper == true && position < 1.00 || gamepad2.right_bumper == true && position < 1.00){
                position += 0.1;
                sleep(100);
                rightliftServo.setPosition(position);
            }

            /*

            if (liftServoPowerUp > 0 && position < 1) {
                position += 0.05;
                leftliftServo.setPosition(position);
            }

            if (liftServoPowerDown > 0 && position > 0){
                position -= 0.05;
                leftliftsetPosition(position);
            }



            */


            /*

            if(gamepad1.x == true && position < 1)
            {
                position = position + 0.1;
                sleep(1200);
                rightliftServo.setPosition(position);
                //leftliftServo.setPosition(position);
            }
            else if(gamepad1.y == true && position > 0)
            {
                position = position - 0.1;
                sleep(1200);
                rightliftServo.setPosition(position);
                //leftliftServo.setPosition(position);
            }

            */

            /*if(gamepad2.a){
                //Lift goes down.
                liftMotor.setPower(linearSlidePower);
            }else if(gamepad2.b){
                //Lift goes up.
                liftMotor.setPower(linearSlidePower);
            }else{
                liftMotor.setPower(0);
            }
            //else{
             //   liftMotor.setPower(0.1);
            //}*/

            //May have to add some ambient power to keep the claw up. Only if we have to though. It would be rather inconvenient otherwise

            //Claw Servo Code

            // double clawPowerDown = gamepad2.left_trigger;
            // double clawPowerUp = gamepad2.right_trigger;

            /*

            if (clawPowerDo
            }
             */

            //setPosition is between 0 and 1



            if(gamepad1.x && left1 == 1 || gamepad2.x && left1 == 1) {
                //open the left claw
                LeftClawServo.setPosition(0.15);
                left1 = 0;

                sleep(300);
            }
            else if (gamepad1.x && left1 == 0 || gamepad2.x && left1 == 0){
                LeftClawServo.setPosition(0);

                left1 = 1;
                sleep(300);
            }

            if(gamepad1.a && right1 == 1 || gamepad2.a && right1 == 1) {
                //open the left claw
                RightClawServo.setPosition(0.35);
                right1 = 0;

                sleep(300);

            }
            else if (gamepad1.a && right1 == 0 || gamepad2.a && right1 == 0){
                RightClawServo.setPosition(0.2);

                right1 = 1;
                sleep(300);
            }

            if(gamepad1.y || gamepad2.y) {
                planeServo.setPosition(0);
            }

            if (gamepad1.dpad_left || gamepad2.dpad_left){
                resetRuntime();
                while (getRuntime() <= 30)
                {
                    liftRightMotor.setPower(1);
                    liftLeftMotor.setPower(1);
                }
            }

            telemetry.addLine("Front Left Motor Power" + frontLeftPower);
            telemetry.addLine("Front Right Motor Power" + frontRightPower);
            telemetry.addLine("Back Left Motor Power" + backLeftPower);
            telemetry.addLine("Back Right Motor Power" + backRightPower);
            telemetry.update();

        }
    }
}