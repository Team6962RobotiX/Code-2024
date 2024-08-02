package frc.robot.subsystems.drive.alt.units;

import edu.wpi.first.units.UnaryFunction;
import edu.wpi.first.units.Unit;

public class Force extends Unit<Force> {
    private Force(double baseUnitEquivalent, String name, String symbol) {
        super(Force.class, baseUnitEquivalent, name, symbol);
    }

    private Force(UnaryFunction toBaseConverter, UnaryFunction fromBaseConverter, String name, String symbol) {
        super(Force.class, toBaseConverter, fromBaseConverter, name, symbol);
    }

    public static final Force Newtons = new Force(1.0, "Newtons", "N");
}
