namespace PipeHandler2;

class Robot
{
    private const double robRad = 0.5 * 0.152;
    private const double robF1A = Math.PI * 101.4 / 180;
    private const double robHx = 0.031;
    private const double robHy = 0.044;
    private const double robF0x = 0.041;
    private const double robF1x = 0.041;
    private const double robC1x = 0.015;
    private const double robC1y = 0.0745;
    private const double robF2y = 0.025;
    private const double robF2x = 0.0278;
    private const double robC2x = 0.018;
    private const double robC2y = 0.046;
    private const double robL1Min = 0.0593;
    private const double robL1Max = 0.1838;
    private const double robL0Min = 0.0623;
    private const double robL0Max = 0.2696;
    
    public class Coordinate
    {
        public double X, Y;
        public static Coord operator +(Coord A, Coord B) => new(){X = A.X + B.X, Y = A.Y + B.Y};
        public static Coord operator *(double alpha, Coord A) => new(){X = alpha * A.X, Y = alpha * A.Y};
        public static Coord operator -(Coord A, Coord B) => A + (-1) * B;
        public static double operator *(Coord A, Coord B) => A.X * B.X + A.Y * B.Y;
        public Coord(){}
        public Coord(int[] ij, Robot r) => (X, Y) = (ij[0] * r.StepX, ij[1] * r.StepY);
        public Coord(double[] XY) => (X, Y) = (XY[0], XY[1]);
        public double Norm
        {
            private set{}
            get => Math.Sqrt(X * X + Y * Y);
        }
        public override string ToString() => $"(X = {X}; Y = {Y})";
    }

    public class Hand
    {
        public Coord[] handLine{get; set;}
        public Coord HandPos{get; set;}
        public int[] IntHandPos{get; set;}
        public double handLen{get; set;}
    }

    public class RobotState: ICloneable
    {
        public Coordinate F2{get; set;}
        public Hand[] MoveableFingers{get; set;} //0 - Fing0, 1 - Fing1, 2 - FingH
        public Coordinate Center;
        public object Clone() => new RobotState(){F2 = F2, MoveableFingers = MoveableFingers, Center = Center};
        public RobotState Copy
        {
            private set{}
            get => (RobotState)Clone();
        }
    }

    private int Iterator = 0;
    public int CurrIterationIndex{get => Iterator; private set{}}

    public static Coord Ein(double alpha) => new(){X = Math.Cos(alpha), Y = Math.Sin(alpha)};
    public static Coord EinNeg(double alpha) => -1 * Ein(alpha);
    public static Coord Compl(Coord A) => new(){X = A.Y, Y = -A.X};
    public static Coord Aff(Coord E, double coeff_t, Coord XY) => XY + coeff_t * E;
    public static double AngNorm(double alpha) => alpha >= 0 ? alpha : 2 * Math.PI + alpha;
    public Coord IntCoordToDouble(int[] IndCoord) => new Coord(){X = IndCoord[0] * StepX, Y = IndCoord[1] * StepY};


}