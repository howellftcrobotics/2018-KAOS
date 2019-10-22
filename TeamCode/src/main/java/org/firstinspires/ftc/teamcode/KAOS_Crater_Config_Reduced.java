package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="3 - Crater: Drop Sample Extend", group="KAOS")
//@Disabled
public class KAOS_Crater_Config_Reduced extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        // setup the properties of this OpMode
        OpModeConfig opModeConfig = new OpModeConfig(
                OpModeConfig.ROBOT_TEAM.KAOS,
                OpModeConfig.ROBOT_OPERATING_MODE.AUTONOMOUS,
                OpModeConfig.ROBOT_STARTING_POSITION.CRATER_LAND_SAMPLE_EXTEND,
                telemetry, hardwareMap, this);
        opModeConfig.runOpMode();
    }
}
