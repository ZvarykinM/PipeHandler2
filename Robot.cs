using System;
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

    public double StepX, StepY, Radius;    
    public class Coordinate
    {
        public double X, Y;
        public static Coordinate operator +(Coordinate A, Coordinate B) => new(){X = A.X + B.X, Y = A.Y + B.Y};
        public static Coordinate operator *(double alpha, Coordinate A) => new(){X = alpha * A.X, Y = alpha * A.Y};
        public static Coordinate operator -(Coordinate A, Coordinate B) => A + (-1) * B;
        public static double operator *(Coordinate A, Coordinate B) => A.X * B.X + A.Y * B.Y;
        public Coordinate(){}
        public Coordinate(int[] ij, Robot r) => (X, Y) = (ij[0] * r.StepX, ij[1] * r.StepY);
        public Coordinate(double[] XY) => (X, Y) = (XY[0], XY[1]);
        public double Norm
        {
            private set{}
            get => Math.Sqrt(X * X + Y * Y);
        }
        public override string ToString() => $"(X = {X}; Y = {Y})";
    }

    public class Hand
    {
        public Coordinate[] handLine{get; set;}
        public Coordinate HandPos{get; set;}
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

    public static Coordinate Ein(double alpha) => new(){X = Math.Cos(alpha), Y = Math.Sin(alpha)};
    public static Coordinate EinNeg(double alpha) => -1 * Ein(alpha);
    public static Coordinate Compl(Coordinate A) => new(){X = A.Y, Y = -A.X};
    public static Coordinate Aff(Coordinate E, double coeff_t, Coordinate XY) => XY + coeff_t * E;
    public static double AngNorm(double alpha) => alpha >= 0 ? alpha : 2 * Math.PI + alpha;
    public Coordinate IntCoordToDouble(int[] IndCoord) => new Coordinate(){X = IndCoord[0] * StepX, Y = IndCoord[1] * StepY};

    public RobotState Inversion02(RobotState? SomeRobotState, int[] ij2, int[] ij0)
    {
        var XY2 = new Coordinate(){X = StepX * ij2[0], Y = StepY * ij2[1]};
        var XY0 = new Coordinate(){X = StepX * ij0[0], Y = StepY * ij0[1]};
        if(ij2[0] <= ij0[0])
        {
            var Dxy = XY0 - XY2;
            var theta = Math.Atan2(Dxy.Y, Dxy.X);
            var alpha = Math.Asin((robF0x - robF2x) / Dxy.Norm);
            if(theta is not double.NaN)
            {
                var e0 = Ein(theta - alpha);
                var l0 = e0 * Dxy - robF2y;
                var e1 = Compl(e0);
                var q2 = Aff(e1, robF2x, XY2);
                var c = Aff(e1, robC2x, Aff(e0, -robC2y, q2));
                var q0 = Aff(e0, robF2y, q2);
                var h0 = new Hand(){handLine = [e0, q0], HandPos = XY0, IntHandPos = ij2, handLen = l0};
                var NewRobotState = SomeRobotState is not null ? SomeRobotState.Copy : new RobotState();
                NewRobotState.F2 = XY2;
                NewRobotState.MoveableFingers[0] = h0;
                NewRobotState.Center = c;
                return NewRobotState;
            }
            return null;
        }
        else
        {
            var Dxy = XY2 - XY0;
            var theta = Math.Atan2(Dxy.Y, Dxy.X);
            var alpha = Math.Sin((robF0x - robF2x) / Dxy.Norm);
            if(theta is not double.NaN)
            {
                var e0 = Ein(theta - alpha);
                var l0 = e0 * Dxy - robF2y;
                var e1 = Compl(e0);
                var q2 = Aff(e1, -robF2x, XY2);
                var c = Aff(e1, -robC2x, Aff(e0, robC2y, q2));
                var q0 = Aff(e0, -robF2y, q2);
                var h0 = new Hand(){handLine = [e0, q0], HandPos = XY0, IntHandPos = ij0, handLen = -l0};
                var NewRobotState = SomeRobotState is not null ? SomeRobotState.Copy : new RobotState();
                NewRobotState.F2 = XY2;
                NewRobotState.MoveableFingers[0] = h0;
                NewRobotState.Center = c;
                return NewRobotState;
            }
            return null;
        }
    }

    public RobotState Inversion1(RobotState SomeRobotState, int[] ij1)
    {
        var XY1 = new Coordinate(){X = StepX * ij1[0], Y = StepY * ij1[1]};
        var c_xy = SomeRobotState.Center;
        var z = robF1x + robC1x;
        if(XY1.Y < c_xy.Y)
        {
            var D_xy = c_xy - XY1;
            var d = D_xy.Norm;
            var beta = Math.Acos(D_xy.X / d);
            var alpha = Math.Asin(z / d);
            if(alpha is not double.NaN && beta is not double.NaN)
            {
                var phi = alpha + beta;
                var e0 = Ein(phi);
                var e1 = Compl(e0);
                var q0 = Aff(e0, -robC1y, Aff(e1, -robC1x, c_xy));
                var l1 = D_xy * e0 - robC1y;
                var h = new Hand(){handLine = [e0, q0], HandPos = XY1, IntHandPos = ij1, handLen = -l1};
                var NewRobotState = SomeRobotState.Copy;
                NewRobotState.MoveableFingers[1] = h;
                return NewRobotState;
            }
            else return null;
        }
        else
        {
            var D_xy = XY1 - c_xy;
            var d = D_xy.Norm;
            var beta = Math.Acos(D_xy.X / d);
            var alpha = Math.Asin(z / d);
            if(alpha is not double.NaN && beta is not double.NaN)
            {
                var phi = alpha + beta;
                var e0 = Ein(phi);
                var e1 = Compl(e0);
                var q0 = Aff(e0, robC1y, Aff(e1, robC1x, c_xy));
                var l1 = D_xy * e0 - robC1y;
                var h = new Hand(){handLine = [e0, q0], HandPos = XY1, IntHandPos = ij1, handLen = l1};
                var NewRobotState = SomeRobotState.Copy;
                NewRobotState.MoveableFingers[1] = h;
                return NewRobotState;
            }
            else return null;
        }
    }

    public RobotState InversionH(RobotState SomeRobotState, int[] ij1)
    {
        var XY1 = new Coordinate(){X = StepX * ij1[0], Y = StepY * ij1[1]};
        var c_xy = SomeRobotState.Center;
        var z = robC1x + robHx;
        if(XY1.Y < c_xy.Y)
        {
            var D_xy = c_xy - XY1; //new Coord(){X = c_xy.X - XY1.X, Y = c_xy.Y - XY1.Y};
            var d = D_xy.Norm;
            var beta = Math.Acos(D_xy.X / d);
            var alpha = Math.Asin(z / d);
            if(alpha is not double.NaN && beta is not double.NaN)
            {
                var phi = alpha + beta;
                var e0 = Ein(phi);
                var e1 = Compl(e0);
                var q0 = Aff(e0, -robC1y, Aff(e1, -robC1x, c_xy));
                var l1 = D_xy * e0 - robC1y;
                var hh = new Hand(){handLine = [e0, q0], HandPos = XY1, IntHandPos = [Convert.ToInt32(XY1.X / StepX), Convert.ToInt32(XY1.Y / StepY)], handLen = -l1};
                var q1 = Aff(e1, -robF1x, Aff(e0, - l1 + robHy, q0));
                var h1 = new Hand(){handLine = [e0, q0], HandPos = q1, IntHandPos = [Convert.ToInt32(q1.X / StepX), Convert.ToInt32(q1.Y / StepY)], handLen = -l1 + robHy};
                var NewRobotState = SomeRobotState.Copy;
                NewRobotState.MoveableFingers[2] = hh;
                NewRobotState.MoveableFingers[1] = h1;
                return NewRobotState;
            }
            else return null;
        }
        else
        {
            var D_xy = XY1 - c_xy;
            var d = D_xy.Norm;
            var beta = Math.Acos(D_xy.X / d);   
            var alpha = Math.Asin(z / d);
            if(alpha is not double.NaN && beta is not double.NaN)

            {
                var phi = alpha + beta;
                var e0 = Ein(phi);
                var e1 = Compl(e0);
                var q0 = Aff(e0, robC1y, Aff(e1, robC1x, c_xy));
                var l1 = D_xy * e0 - robC1y;
                var hh = new Hand(){handLine = [e0, q0], HandPos = XY1, IntHandPos = [Convert.ToInt32(XY1.X / StepX), Convert.ToInt32(XY1.Y / StepY)], handLen = l1};
                var q1 = Aff(e1, robF1x, Aff(e0, l1 - robHy, q0));
                var h1 = new Hand(){handLine = [e0, q0], HandPos = q1, IntHandPos = [Convert.ToInt32(q1.X / StepX), Convert.ToInt32(q1.Y / StepY)], handLen = l1 - robHy};
                var NewRobotState = SomeRobotState.Copy;
                NewRobotState.MoveableFingers[3] = hh;
                NewRobotState.MoveableFingers[1] = h1;
                return NewRobotState;
            }
            else return null;
        }
    }

}