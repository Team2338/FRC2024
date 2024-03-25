package team.gif.lib;

public enum autoMode {
    NONE(0),
    MOBILITY(0),
    CIRCLE(0),
    TWO_CTR_C(0),
    TWO_SRC_7(0),
    TWO_SRC_8(0),
    TWO_SRC_S(0),
    THREE_AMP_A_4_5(0),
    THREE_W_8_7(0),
    THREE_W_7_8(0),
    THREE_W_7_6(0),
    FOUR_AMP_A_C_S(0),
    FOUR_CTR_C_S_A_4(0),
    FIVE_SRC_S_C_A_4(0),
    FIVE_CTR_C_S_A_4(0),
    FIVE_CTR_C_S_A_5(0),
    LINE_TEST(0),
    TWO_SCSPLIT_SIX(0),
    ;

    private int value;

    autoMode(int value) {
        this.value = value;
    }

    public int getValue() {
        return this.value;
    }
}
