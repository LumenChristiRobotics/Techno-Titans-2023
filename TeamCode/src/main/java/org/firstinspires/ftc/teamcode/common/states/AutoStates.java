package org.firstinspires.ftc.teamcode.common.states;

public class AutoStates {
    public enum LinearStates {
        SCORE_PURPLE_PIXEL,
        SCORE_YELLOW_PIXEL,
        PARK_IN_BACKSTAGE,
        EXTENDING,
        RETRACTING,
        RELEASE
    }

    public enum FallbackStates {
        MOVE_FALLBACK,
        DETECTION_FAIL
    }
}
