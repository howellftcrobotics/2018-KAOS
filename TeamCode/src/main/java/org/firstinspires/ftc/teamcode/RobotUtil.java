package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



public class RobotUtil
{
    public static double TryGetServoPosition (Servo servo)
    {
        double result = 0.0;

        try
        {
            if (servo != null)
            {
                servo.getPosition();
            }
        }
        catch (Exception ex)
        {
            // log this
            //DbgLog.msg("RobotUtil:GetServoPosition() - Exception caught:" + ex.toString());
        }
        return result;
    }

    public static void TrySetServoPosition (Servo servo, double position)
    {
        // make sure the position passed in within the legal range
        double clippedPosition = Range.clip(position, Servo.MIN_POSITION, Servo.MAX_POSITION);

        try
        {
            if (servo != null)
            {
                servo.setPosition(clippedPosition);
            }
        }
        catch (Exception ex)
        {
            // log this
            //DbgLog.msg("RobotUtil:SetServoPosition() - Exception caught:" + ex.toString());
        }
    }

    public static double TryGetMotorPower (DcMotor motor)
    {
        double result = 0.0;

        try
        {
            if (motor != null)
            {
                result = motor.getPower ();
            }
        }
        catch (Exception ex)
        {
            // log this
            //.msg("RobotUtil:GetMotorPower() - Exception caught:" + ex.toString());
        }
        return result;
    }

    public static void TrySetMotorPower(DcMotor motor, float power)
    {
        try
        {
            if (motor != null)
            {
                motor.setPower(power);
            }
        }
        catch (Exception ex)
        {
            // log this
            //DbgLog.msg("RobotUtil:SetMotorPower() - Exception caught:" + ex.toString());
        }
    }

    public static void TryResetEncoder(DcMotor motor)
    {
        try
        {
            if (motor != null)
            {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        catch (Exception ex)
        {
            // log this
            //DbgLog.msg("RobotUtil:ResetEncoder() - Exception caught:" + ex.toString());
        }
    }

    public static void TrySetMotorDirection(DcMotor motor, DcMotor.Direction direction)
    {
        try
        {
            if (motor != null)
            {
                motor.setDirection(direction);
            }
        } catch (Exception ex)
        {
            // log this
            //DbgLog.msg("RobotUtil:SetMotorDirection() - Exception caught:" + ex.toString());
        }
    }

    public static void TrySetServoDirection(Servo servo, Servo.Direction direction)
    {
        try
        {
            if (servo != null)
            {
                servo.setDirection(direction);
            }
        } catch (Exception ex)
        {
            // log this
            //DbgLog.msg("RobotUtil:SetServoDirection() - Exception caught:" + ex.toString());
        }
    }

    public static int TryGetMotorCurrentPosition (DcMotor motor)
    {
        int result = 0;

        try
        {
            if (motor != null)
            {
                result = Math.abs(motor.getCurrentPosition());
            }
        }
        catch (Exception ex)
        {
            // log this
            //DbgLog.msg("RobotUtil:GetMotorCurrentPosition() - Exception caught:" + ex.toString());
        }
        return result;
    }

    public static int TryGetMotorTargetPosition (DcMotor motor)
    {
        int result = 0;

        try
        {
            if (motor != null)
            {
                result = Math.abs(motor.getTargetPosition());
            }
        }
        catch (Exception ex)
        {
            // log this
            //DbgLog.msg("RobotUtil:GetMotorTargetPosition() - Exception caught:" + ex.toString());
        }
        return result;
    }

    public static void TrySetMotorTargetPosition (DcMotor motor, int position)
    {
        try
        {
            if (motor != null)
            {
                motor.setTargetPosition(position);
            }
        }
        catch (Exception ex)
        {
            // log this
            //DbgLog.msg("RobotUtil:SetMotorTargetPosition() - Exception caught:" + ex.toString());
        }
    }

    public static void TrySetMotorRunMode(DcMotor motor, DcMotor.RunMode runMode,
                                          DcMotor.Direction direction, boolean resetEncoder, int initialTargetPosition, float InitialPower)
    {
        try
        {
            if (motor != null)
            {
                // check to see if the caller wants the encoders reset
                if (resetEncoder)
                {
                    TryResetEncoder(motor);
                }

                // set the specified motor run mode
                if (runMode != null)
                {
                    motor.setMode(runMode);

                    if (runMode == DcMotor.RunMode.RUN_TO_POSITION)
                    {
                        TrySetMotorTargetPosition(motor, initialTargetPosition);
                    }
                }

                // set the specified motor direction
                if (direction != null)
                {
                    TrySetMotorDirection(motor, direction);
                }

                // set the initial power
                TrySetMotorPower(motor, InitialPower);
            }
        }
        catch (Exception ex)
        {
            // log this
            //DbgLog.msg("RobotUtil:SetMotorRunMode() - Exception caught:" + ex.toString());
        }
    }

    public static void TrySetMotorRunMode(DcMotor motor, DcMotor.RunMode runMode,
                                          boolean resetEncoder, int initialTargetPosition, float InitialPower)
    {
        try
        {
            if (motor != null)
            {
                // check to see if the caller wants the encoders reset
                if (resetEncoder)
                {
                    TryResetEncoder(motor);
                }

                // set the specified motor run mode
                if (runMode != null)
                {
                    motor.setMode(runMode);

                    if (runMode == DcMotor.RunMode.RUN_TO_POSITION)
                    {
                        TrySetMotorTargetPosition(motor, initialTargetPosition);
                    }
                }

                // set the initial power
                TrySetMotorPower(motor, InitialPower);
            }
        }
        catch (Exception ex)
        {
            // log this
            //DbgLog.msg("RobotUtil:SetMotorRunMode() - Exception caught:" + ex.toString());
        }
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the m_robot more precisely at slower speeds.
     */
    public static double TryScaleJoystickInput(double dVal)
    {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // clip the right/left values so that the values never exceed +/- 1
        dVal = Range.clip(dVal, -1, 1);

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    public static double TryScaleJoystickInput_KILTS_Drive(double dVal)
    {
        //double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
        //        0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };


        double[] scaleArray = { 0.0, 0.04, 0.072, 0.8, 0.096, 0.12, 0.144, 0.192,
                0.24, 0.288, 0.344, 0.4, 0.48, 0.576, 0.68, 0.80, 0.80 };

        // clip the right/left values so that the values never exceed +/- 1
        dVal = Range.clip(dVal, -1, 1);

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    public static void TryControlMotorByEncoder(DcMotor motor, OpModeConfig opModeConfig, double speed,
                                                int encoderCounts, double timeout)
    {
        int newTarget;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the opmode is still active
        if (opModeConfig.OpMode.opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newTarget = motor.getCurrentPosition() + encoderCounts;
            motor.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            motor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the m_robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the m_robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeConfig.OpMode.opModeIsActive() && (elapsedTime.seconds() < timeout) && (motor.isBusy()))
            {
                // Display it for the driver.
                if (!opModeConfig.CompetitionMode)
                {
                    opModeConfig.Telemetry.addData("Path1", "Running to %7d", newTarget);
                    opModeConfig.Telemetry.addData("Path2", "Running at %7d", motor.getCurrentPosition());
                }
                opModeConfig.OpMode.idle();
            }

            // Stop all motion;
            motor.setPower(0);

            // Turn off RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public static void TryControlMotorByTime(DcMotor motor, OpModeConfig opModeConfig, double speed, double timeout)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure that the opmode is still active
        if (opModeConfig.OpMode.opModeIsActive())
        {
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            motor.setPower(speed);

            // keep looping while we are still active, and there is time left
            while (opModeConfig.OpMode.opModeIsActive() && (elapsedTime.seconds() < timeout))
            {
                // just wasting time
                opModeConfig.OpMode.idle();
            }

            // Stop all motion;
            motor.setPower(0);
        }
    }

//    private static double ScalePower2(double... powers)
//    {
//        double maxValue = DoubleStream.of(powers).max().getAsDouble();
//        if (maxValue > 1) {
//            return 1 / maxValue;
//        }
//        else{
//            return 1;
//        }
//    }

    public static double ScalePower(double p1, double p2, double p3, double p4)
    {
        //Returns a scaling factor to normalize the input powers p1-p4 to a maximum magnitude of 1

        double maxValue = Math.abs(p1);
        double scaleFactor = 1;

        //Search for the largest power magnitude
        if (Math.abs(p2) > maxValue)
        {
            maxValue = Math.abs(p2);
        }
        if (Math.abs(p3) > maxValue)
        {
            maxValue = Math.abs(p3);
        }
        if (Math.abs(p4) > maxValue)
        {
            maxValue = Math.abs(p4);
        }

        //If maxValue is larger than 1 return a scale factor to limit it to +1 or -1
        if (maxValue > 1 )
        {
            scaleFactor = 1 / maxValue;
        }

        return  scaleFactor;
    }
}
