package org.firstinspires.ftc.teamcode.Subsystems;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Claw;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Telescope;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Wrist;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class Arm {
    private Shoulder shoulder;
    private Telescope telescope;
    private Claw claw;
    private Lift lift;
    private Wrist wrist;
    private AHRS navx;
    private RevBlinkinLedDriver blinkin;
    private RevBlinkinLedDriver.BlinkinPattern mainColor;
    private RevBlinkinLedDriver.BlinkinPattern violet = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
    private int state = 0;
    private boolean autoPickup = false;
    private ElapsedTime colorTimer = new ElapsedTime();
    private boolean isRedHere;

    public Arm(Shoulder s, Telescope t, Claw c, Lift l, Wrist w, AHRS n, RevBlinkinLedDriver b, boolean fromAuto, boolean isRed) {
        this.shoulder = s;
        this.telescope = t;
        this.claw = c;
        this.lift = l;
        this.wrist = w;
        this.navx = n;
        this.blinkin = b;
        this.isRedHere = isRed;
        if (isRed) {
            this.mainColor = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        } else {
            this.mainColor = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        }
        this.blinkin.setPattern(this.mainColor);
    }

    public void goTo_Initialization(){
        this.wrist.setTarget(RC_Wrist.stowPos, RC_Wrist.stowTime);
        this.claw.closeBottom();
        this.claw.closeTop();
        this.shoulder.setPosition(100);
    }

    public void goTo_Pickup(boolean holding){
        if (this.state != 1) {
            if (!holding) {
                this.claw.openBottom();
                this.claw.openTop();
            }
            this.wrist.setTarget(RC_Wrist.pickupPos, RC_Wrist.pickupTime);
            this.state = -1;

        }
    }

    public void goTo_Stow(boolean holding) throws InterruptedException {
        if (this.state == -1) {
            this.claw.closeBottom();
            this.claw.closeTop();
            Thread.sleep(RC_Claw.pickupPause);
            this.wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);
        } else if (this.state == 1) {
            this.claw.partialBottom();
            Thread.sleep(RC_Claw.dropBottomPause);
            this.claw.partialTop();
            Thread.sleep(RC_Claw.dropTopPause);
            this.shoulder.setPosition(RC_Shoulder.stowPos);
            this.telescope.setPosition(RC_Telescope.stowPos);
            this.wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);
        } else if (this.state == 0) {
            this.shoulder.setPosition(RC_Shoulder.stowPos);
            this.telescope.setPosition(RC_Telescope.stowPos);
            this.wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);
        }
        this.state = 0;
    }

    public void goTo_dropOff(boolean holding){
        if (this.state == 0){
            if (TelemetryData.whatHeadingDo < -4.4 || TelemetryData.whatHeadingDo > 1.5){
                //we are facing the board
                this.shoulder.setPosition(RC_Shoulder.dropOffPosRev);
                this.telescope.setPosition(RC_Telescope.dropOffHighRev);
                this.wrist.setTarget(RC_Wrist.dropOffPosRev, RC_Wrist.dropOffTime);
            } else {
                //we are facing normally
                this.shoulder.setPosition(RC_Shoulder.dropOffPos);
                this.telescope.setPosition(RC_Telescope.dropOffHigh);
                this.wrist.setTarget(RC_Wrist.dropOffPos, RC_Wrist.dropOffTime);
            }
            this.state = 1;
        }
    }
}