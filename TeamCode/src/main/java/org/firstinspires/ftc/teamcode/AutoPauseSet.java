package org.firstinspires.ftc.teamcode;

public class AutoPauseSet
{
    public boolean a = false;
    public boolean b = false;
    public boolean x = false;
    public boolean y = false;

    public boolean dpad_up = false;
    public boolean dpad_down = false;
    public boolean dpad_left = false;
    public boolean dpad_right = false;

    public boolean useGyro = true;

    public AutoPauseSet()
    {
        Reset();
    }

    public void Reset()
    {
        a = false;
        b = false;
        x = false;
        y = false;
        dpad_down = false;
        dpad_left = false;
        dpad_right = false;
        dpad_up = false;
        useGyro = true;
    }

    public int GetPostionPauseB()
    {
        int result = 0;

        if (a) {
            result += 1;
        }
        if (b) {
            result += 1;
        }

        if (x) {
            result += 1;
        }
        if (y) {
            result += 1;
        }

        return result;
    }

    public int GetPostionPauseA()
    {
        int result = 0;

        if (dpad_up)
        {
            result += 1;
        }
        if (dpad_down)
        {
            result += 1;
        }
        if (dpad_right)
        {
            result += 1;
        }
        if (dpad_left)
        {
            result += 1;
        }

        return result;
    }
}
