package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ScoreLevel;

public class Joysticks {
    private final XboxController driver = new XboxController(0);
    private final PS5Controller controller = new PS5Controller(1);

    // ---------- Suppliers ---------- //
    private final Supplier<Boolean> wantCoralAutoAlign = () -> this.getInput().wantExtend;
    private final Supplier<Boolean> wantAlgaeAutoAlign = () -> this.getInput().wantGetAlgae &&
        Arm.getInstance().atSetpoint() &&
        Elevator.getInstance().atSetpoint();
    private final Supplier<Boolean> wantBargeAlign = () -> this.getInput().wantExtend &&
        (SuperStructure.getInstance().state == SuperStructure.State.AlgaeRest ||
            SuperStructure.getInstance().state == SuperStructure.State.PreBarge ||
            SuperStructure.getInstance().state == SuperStructure.State.ScoreBarge);

    public final Supplier<Boolean> wantOffsetArmPositive = () -> this.controller.getLeftX() > 0.9 && this.controller.getL3Button();
    public final Supplier<Boolean> wantOffsetArmNegative = () -> this.controller.getLeftX() < -0.9 && this.controller.getL3Button();

    public enum AlignMode {
        None,
        ReefAlign,   
        ThroughAlign,
        AlgaeAlign,
        BargeAlign
    }

    public static class DriveInputs {
        public double leftY;
        public double leftX;
        public double rightX;
        public double deadZone;
        public AlignMode alignMode;

        public boolean isNonZero() {
            return Math.abs(leftX) > deadZone ||
                Math.abs(leftY) > deadZone ||
                Math.abs(rightX) > deadZone;
        }

        public DriveInputs getRedFlipped() {
            DriveInputs flipped = new DriveInputs();
            flipped.leftX = -this.leftX;
            flipped.leftY = -this.leftY;
            flipped.rightX = -this.rightX;
            flipped.deadZone = deadZone;
            flipped.alignMode = alignMode;
            return flipped;
        }
    }

    public DriveInputs getDriveInput() {
        DriveInputs input = new DriveInputs();
        input.leftX = this.driver.getLeftX();
        input.leftY = this.driver.getLeftY();
        input.rightX = this.driver.getRightX();
        input.deadZone = 0.03;
        
        if (this.wantBargeAlign.get()) {
            input.alignMode = AlignMode.BargeAlign;
        } else if (this.wantCoralAutoAlign.get() && this.getInput().wantedScoringLevel != ScoreLevel.Through) {
            input.alignMode = AlignMode.ReefAlign;
        } else if (this.wantCoralAutoAlign.get() && this.getInput().wantedScoringLevel == ScoreLevel.Through) {
            input.alignMode = AlignMode.ThroughAlign;
        } else if (this.wantAlgaeAutoAlign.get() && this.getInput().wantGetAlgae) {
            input.alignMode = AlignMode.AlgaeAlign;
        } else {
            input.alignMode = AlignMode.None;
        }
        
        return input;
    }

    private ScoreLevel lastScoreLevel = ScoreLevel.L2;
    public SuperStructure.StructureInput getInput() {
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

        SuperStructure.StructureInput input = new SuperStructure.StructureInput();
        input.wantExtend = this.controller.getL2Button();
        input.wantScore = this.driver.getRawAxis(4) > 0.5 || this.controller.getR3Button();
        input.wantGroundIntake = this.controller.getR2Button();
        input.wantArmSourceIntake = this.controller.getCrossButton();
        input.wantedScoringLevel = level;
        input.wantGetAlgae = this.controller.getR1Button();
        input.wantDescoreAlgae = this.controller.getTriangleButton();
        input.wantResetSuperstructure = this.controller.getOptionsButton();
        input.wantSourceIntake = this.controller.getSquareButton();
        input.wantScoreProcessor = this.controller.getCircleButton();
        input.wantAlgaeGroundIntake = this.controller.getL1Button();
        return input;
    }
}
