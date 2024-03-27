package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class Lift {
    DcMotorEx motor;

    //24442 on the encoder = 6 wraps
    //and marked with black

    //if we zero the encoder at the beginning
    //the max ticks would be 15500ish

    //positive power moves chord out

    public Lift(DcMotorEx m) {
        this.motor = m;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getEncoderValue() {
        return this.motor.getCurrentPosition();
    }

    /**
     * only method that moves the lift
     * @param input positive values extends the string
     */
    public void move(double input){
        this.motor.setPower(input);
    }

    public void getCurrent(){
        TelemetryData.lift_current = this.motor.getCurrent(CurrentUnit.MILLIAMPS);
    }
}