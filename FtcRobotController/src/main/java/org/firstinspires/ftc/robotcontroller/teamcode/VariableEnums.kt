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
    NUM_OF_STONES,
    NUM_OF_STONES_ON_WAFFLE,
    DAHOOKER_START_POSITION,
    DAHOOKER_END_POSITION,
    CLAW_START,
    CLAW_END


}

enum class BooleanVariable(var boolean: Boolean = false) {
    WAFFLE_OR_NOT,
    WAFFLE_SIDE,
    SKY_STONE_SIDE,
    PARK_AT_TOP,
    PARK_AT_BOTTOM,
    PARK_OR_NOT,
}