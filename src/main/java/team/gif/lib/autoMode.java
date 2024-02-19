package team.gif.lib;

public enum autoMode {
    NONE(0),
    MOBILITY(0),
    CIRCLE(0),
    TWO_CTR_C(0),
    TWO_SRC_S(0),
    TWO_SRC_8(0),
    TWO_SRC_7(0)
    ;

    private int value;

    autoMode(int value) {
        this.value = value;
    }

    public int getValue() {
        return this.value;
    }
}
