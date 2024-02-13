package team.gif.lib;

public enum autoMode {
    NONE(0),
    MOBILITY(0),
    CIRCLE(0),
    CTR_C(0)
    ;

    private int value;

    autoMode(int value) {
        this.value = value;
    }

    public int getValue() {
        return this.value;
    }
}
