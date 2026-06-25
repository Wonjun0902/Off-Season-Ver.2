package frc.lib.helpers;

public class TagValues {

    public record TagTrust(double hubAccuracy, double climbAccuracy) {}

    public static TagTrust getTagTrust(int tagID) {
        return switch (tagID) {
            case 1 -> new TagTrust(0.5, 0.5);
            case 2 -> new TagTrust(0.5, 0.5);
            case 3 -> new TagTrust(0.5, 0.5);
            case 4 -> new TagTrust(0.5, 0.5);
            case 5 -> new TagTrust(0.5, 0.5);
            case 6 -> new TagTrust(0.5, 0.5);
            case 7 -> new TagTrust(0.5, 0.5);
            case 8 -> new TagTrust(0.5, 0.5);
            case 9 -> new TagTrust(0.5, 0.5);
            case 10 -> new TagTrust(0.5, 0.5);
            case 11 -> new TagTrust(0.5, 0.5);
            case 12 -> new TagTrust(0.5, 0.5);
            case 13 -> new TagTrust(0.5, 0.5);
            case 14 -> new TagTrust(0.5, 0.5);
            case 15 -> new TagTrust(0.5, 0.5);
            case 16 -> new TagTrust(0.5, 0.5);
            case 17 -> new TagTrust(0.5, 0.5);
            case 18 -> new TagTrust(0.5, 0.5);
            case 19 -> new TagTrust(0.5, 0.5);
            case 20 -> new TagTrust(0.5, 0.5);
            case 21 -> new TagTrust(0.5, 0.5);
            case 22 -> new TagTrust(0.5, 0.5);
            case 23 -> new TagTrust(0.5, 0.5);
            case 24 -> new TagTrust(0.5, 0.5);
            case 25 -> new TagTrust(0.5, 0.5);
            case 26 -> new TagTrust(0.5, 0.5);
            case 27 -> new TagTrust(0.5, 0.5);
            case 28 -> new TagTrust(0.5, 0.5);
            case 29 -> new TagTrust(0.5, 0.5);
            case 30 -> new TagTrust(0.5, 0.5);
            case 31 -> new TagTrust(0.5, 0.5);
            case 32 -> new TagTrust(0.5, 0.5);
            default -> new TagTrust(0.0, 0.0);
        };
    }
}