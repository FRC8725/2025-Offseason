package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ScoreLevel;

public class Joysticks {
    private final XboxController driver = new XboxController(0);
    private final PS5Controller controller = new PS5Controller(1);

    // ---------- Suppliers ---------- //
    private final Supplier<Boolean> wantCoralAutoAlign = () -> this.superStructureInput().wantExtend;
    private final Supplier<Boolean> wantAlgaeAutoAlign = () -> this.superStructureInput().wantGetAlgae;
    private final Supplier<Boolean> wantBargeAlign = () -> this.superStructureInput().wantExtend;
    private final Supplier<Boolean> wantOffsetArmPositive = () -> this.controller.getLeftX() > 0.9 && this.controller.getL3Button();
    private final Supplier<Boolean> wantOffsetArmNegative = () -> this.controller.getLeftX() < -0.9 && this.controller.getL3Button();

    public enum AlignMode {
        None,
        Reef,   
        ThroughAlign,
        AlgaeAlign,
        BargeAlign
    }

    public class DriveInputs {
        public double leftY;
        public double leftX;
        public double rightX;
        public double deadZone;
        public AlignMode alignMode;
    }

    public DriveInputs getDriveInput() {
        DriveInputs input = new DriveInputs();
        input.leftX = this.driver.getLeftX();
        input.leftY = this.driver.getLeftY();
        input.rightX = this.driver.getRightX();
        input.deadZone = 0.05;
        
        if (this.wantBargeAlign.get()) input.alignMode = AlignMode.BargeAlign;
        else if (this.wantCoralAutoAlign.get() &&
            this.superStructureInput().wantedScoringLevel != ScoreLevel.Through) input.alignMode = AlignMode.Reef;
        else if (this.wantCoralAutoAlign.get() && 
            this.superStructureInput().wantedScoringLevel == ScoreLevel.Through) input.alignMode = AlignMode.ThroughAlign;
        else if (this.wantAlgaeAutoAlign.get() &&
            this.superStructureInput().wantGetAlgae) input.alignMode = AlignMode.AlgaeAlign;
        else input.alignMode = AlignMode.None;
        
        return input;
    }

    private ScoreLevel lastScoreLevel = ScoreLevel.Through;
    public SuperStructure.SuperstructureInputs getInput() {
        ScoreLevel level;
        int pov = controller.getPOV();

        switch (pov) {
            case 0 -> level = ScoreLevel.L4;
            case 270 -> level = ScoreLevel.L3;
            case 180 -> level = ScoreLevel.L2;
            case 90 -> level = ScoreLevel.Through;
            default -> level = this.lastScoreLevel;
        }
        this.lastScoreLevel = level;
        
        return new SuperStructure.SuperstructureInputs(
            this.controller.getL2Button(),
            this.driver.getRawAxis(4) > 0.5 || this.controller.getR3Button(),
            this.controller.getR2Button(),
            this.controller.getCrossButton(),
            level,
            this.controller.getR1Button(),
            this.controller.getTriangleButton(),
            this.controller.getOptionsButton(),
            this.controller.getSquareButton(),
            this.controller.getCircleButton(),
            this.controller.getL1Button());

        // input.wantExtend = this.controller.getL2Button();
        // input.wantScore = this.driver.getRawAxis(4) > 0.5 || this.controller.getR3Button();
        // System.out.println(input.wantScore);
        // input.wantGroundIntake = this.controller.getR2Button();
        // input.wantArmSourceIntake = this.controller.getCrossButton();
        // input.wantedScoringLevel = level;
        // input.wantGetAlgae = this.controller.getR1Button();
        // input.wantDescoreAlgae = this.controller.getTriangleButton();
        // input.wantResetSuperstructure = this.controller.getOptionsButton();
        // input.wantSourceIntake = this.controller.getSquareButton();
        // input.wantScoreProcessor = this.controller.getCircleButton();
        // input.wantAlgaeGroundIntake = this.controller.getL1Button();
        // return input;
    }
}
