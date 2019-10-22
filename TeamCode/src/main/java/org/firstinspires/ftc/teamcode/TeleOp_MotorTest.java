package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// this class is used to test motor wiring
@TeleOp(name="TeleOp_MotorTest", group="TEAM - TeleOp")
@Disabled
public class TeleOp_MotorTest extends LinearOpMode
{
    // member level variables go here
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    // main entry point for this class
    @Override
    public void runOpMode()
    {
        telemetry.addData("Status:", "Initializing...");
        telemetry.update();

        // method level variables go here
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");

        ResetEncoders();

        // default to all motors forward
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status:", "Waiting for start...");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a)
            {
                ResetEncoders();
            }

            double reducedPowerAmount = 0.33;

            double leftFrontPower = gamepad1.left_stick_y * reducedPowerAmount;
            double rightFrontPower = gamepad1.left_stick_x * reducedPowerAmount;
            double leftRearPower = gamepad1.right_stick_y * reducedPowerAmount;
            double rightRearPower = gamepad1.right_stick_x * reducedPowerAmount;

            leftFront.setPower(leftFrontPower);     // left joystick up/y = clockwise from outside view
            leftRear.setPower(leftRearPower);       // right joystick up/y = clockwise from outside view
            rightFront.setPower(rightFrontPower);   // left joystick right/x = counter clockwise from outside view
            rightRear.setPower(rightRearPower);     // right joystick right/x = counter clockwise from outside view

            telemetry.addData("Power:", "left front: (%.2f), right front: (%.2f), left rear: (%.2f), right rear: (%.2f)",
                    leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
            telemetry.addData("Counts:", "left front: %7d", leftFront.getCurrentPosition());
            telemetry.addData("Counts:", "right front: %7d", rightFront.getCurrentPosition());
            telemetry.addData("Counts:", "left rear: %7d", leftRear.getCurrentPosition());
            telemetry.addData("Counts:", "right rear: %7d", rightRear.getCurrentPosition());

            telemetry.update();
        }
    }

    private void ResetEncoders() {

        // stop and reset all encoders (power = zero and encoder count = zero)
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set all motors to run with encoders for our test routine (running with encoders is slower, opposite is RUN_WITHOUT_ENCODER)
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
