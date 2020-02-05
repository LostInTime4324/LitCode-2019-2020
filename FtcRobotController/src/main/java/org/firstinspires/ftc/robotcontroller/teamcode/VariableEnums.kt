package org.firstinspires.ftc.robotcontroller.teamcode


class EnumVariable {
    enum class AUTO_SIDE {
        B,
    }


    enum class GOLD_LOCATION {
        B,
    }
}

enum class NumberVariable(var number: Double = 0.0) {
    NUM_OF_SKY_STONES,
    NUM_OF_SKY_STONES_ON_WAFFLE,
    FIRST_DRIVE,
    SLIDE_SPEED,
    SLIDE_CORRECTION,
    FIRST_TURN,
    DRIVE_TO_STONE,
    TURN_TO_WAFFLE,
    FIRST_DISTANCE_TO_WAFFLE,
    SECOND_DISTANCE_TO_WAFFLE,
    THIRD_DISTANCE_TO_WAFFLE,
    BLUE_WAFFLE_ALIGNMENT_TURN,
    RED_WAFFLE_ALIGNMENT_TURN,
    SLIDE_DISTANCE_TO_WAFFLE,
    SLIDE_DISTANCE_TO_BUILD_ZONE,
    TURN_WAFFLE,
    TURN_STONE_PLACEMENT,
    TURN_PARK,
    DRIVE_TO_PARK,
    WAFFLE_SIDE_FIRST_TURN,
    WAFFLE_SIDE_SLIDE_DISTANCE,
    DAHOOKER_START_POSITION,
    DAHOOKER_END_POSITION,
    CLAW_START,
    CLAW_END


}

enum class BooleanVariable(var boolean: Boolean = false) {
    RED_SIDE,
    BLUE_SIDE,
    WAFFLE_OR_NOT,
    WAFFLE_SIDE,
    SKY_STONE_SIDE,
    PARK_AT_TOP,
    PARK_AT_BOTTOM
}