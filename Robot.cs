using System;
using System.Collections.Generic;
using System.Linq;
using Avalonia.Interactivity;
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

    private Grid GridContext;

    public Grid GetGridContext{get => GridContext; private set{}}

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
        public double handLen{get; set;}
    }

    public class RobotState: ICloneable
    {
        public Coordinate F2{get; set;}
        private Hand[] SetMoveableFingers = new Hand[3];
        public Hand[] MoveableFingers{get => SetMoveableFingers; set => SetMoveableFingers = value;} //0 - Fing0, 1 - Fing1, 2 - FingH
        public Coordinate Center;
        public object Clone() => new RobotState(){F2 = F2, MoveableFingers = MoveableFingers, Center = Center};
        public RobotState Copy
        {
            private set{}
            get => (RobotState)Clone();
        }
    }

    private RobotState[] SetOfRobotStates = new RobotState[3];
    public RobotState[] GetRobotStates{get => SetOfRobotStates;}
    private int Iterator = 0;
    public int CurrIterationIndex{get => Iterator; private set{}}

    public Robot(int[] F0, int[] F1, int[] F2, string PathToFile)
    {
        GridContext = new(PathToFile);
        StepX = GridContext.XStep;
        StepY = GridContext.YStep;
        Radius = GridContext.Radius;
        SetOfRobotStates = new RobotState[3];
        var f0 = new double[2]{F0[0] * StepX, F0[1] * StepY};
        var f1 = new double[2]{F1[0] * StepX, F1[1] * StepY};
        var f2 = new double[2]{F2[0] * StepX, F2[1] * StepY};
        var StartState02 = Inversion02(null, f2, f0);
        var StartState = Inversion1(StartState02, f1);
        SetOfRobotStates[1] = StartState;
    }

    public static Coordinate Ein(double alpha) => new(){X = Math.Cos(alpha), Y = Math.Sin(alpha)};
    public static Coordinate EinNeg(double alpha) => -1 * Ein(alpha);
    public static Coordinate Compl(Coordinate A) => new(){X = A.Y, Y = -A.X};
    public static Coordinate Aff(Coordinate E, double coeff_t, Coordinate XY) => XY + coeff_t * E;
    public static double AngNorm(double alpha) => alpha >= 0 ? alpha : 2 * Math.PI + alpha;
    public Coordinate IntCoordToDouble(int[] IndCoord) => new Coordinate(){X = IndCoord[0] * StepX, Y = IndCoord[1] * StepY};

    public RobotState? Inversion02(RobotState? SomeRobotState, double[] ij2, double[] ij0)
    {
        var XY2 = new Coordinate(ij2);
        var XY0 = new Coordinate(ij0);
        Coordinate Dxy, c;
        Hand h0;
        if(ij2[0] <= ij0[0])
        {
            Dxy = XY0 - XY2;
            var theta = Math.Atan2(Dxy.Y, Dxy.X);
            var alpha = Math.Asin((robF0x - robF2x) / Dxy.Norm);
            if(theta is not double.NaN)
            {
                var e0 = Ein(theta - alpha);
                var l0 = e0 * Dxy - robF2y;
                var e1 = Compl(e0);
                var q2 = Aff(e1, robF2x, XY2);
                c = Aff(e1, robC2x, Aff(e0, -robC2y, q2));
                var q0 = Aff(e0, robF2y, q2);
                h0 = new Hand(){handLine = [e0, q0], HandPos = XY0, handLen = l0};
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
            Dxy = XY2 - XY0;
            var theta = Math.Atan2(Dxy.Y, Dxy.X);
            var alpha = Math.Sin((robF0x - robF2x) / Dxy.Norm);
            if(theta is not double.NaN)
            {
                var e0 = Ein(theta - alpha);
                var l0 = e0 * Dxy - robF2y;
                var e1 = Compl(e0);
                var q2 = Aff(e1, -robF2x, XY2);
                c = Aff(e1, -robC2x, Aff(e0, robC2y, q2));
                var q0 = Aff(e0, -robF2y, q2);
                h0 = new Hand(){handLine = [e0, q0], HandPos = XY0, handLen = -l0};
                var NewRobotState = SomeRobotState is not null ? SomeRobotState.Copy : new RobotState();
                NewRobotState.F2 = XY2;
                NewRobotState.MoveableFingers[0] = h0;
                NewRobotState.Center = c;
                return NewRobotState;
            }
            return null;
        }
    }

    public RobotState? Inversion1(RobotState SomeRobotState, double[] ij1)
    {
        var XY1 = new Coordinate(ij1);
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
                var h = new Hand(){handLine = [e0, q0], HandPos = XY1, handLen = -l1};
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
                var h = new Hand(){handLine = [e0, q0], HandPos = XY1, handLen = l1};
                var NewRobotState = SomeRobotState.Copy;
                NewRobotState.MoveableFingers[1] = h;
                return NewRobotState;
            }
            else return null;
        }
    }

    public RobotState? InversionH(RobotState SomeRobotState, double[] ij1)
    {
        var XY1 = new Coordinate(ij1);
        var c_xy = SomeRobotState.Center;
        var z = robC1x + robHx;
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
                var hh = new Hand(){handLine = [e0, q0], HandPos = XY1, handLen = -l1};
                var q1 = Aff(e1, -robF1x, Aff(e0, - l1 + robHy, q0));
                var h1 = new Hand(){handLine = [e0, q0], HandPos = q1, handLen = -l1 + robHy};
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
                var hh = new Hand(){handLine = [e0, q0], HandPos = XY1, handLen = l1};
                var q1 = Aff(e1, robF1x, Aff(e0, l1 - robHy, q0));
                var h1 = new Hand(){handLine = [e0, q0], HandPos = q1, handLen = l1 - robHy};
                var NewRobotState = SomeRobotState.Copy;
                NewRobotState.MoveableFingers[2] = hh;
                NewRobotState.MoveableFingers[1] = h1;
                return NewRobotState;
            }
            else return null;
        }
    }

    public bool CheckHand(Hand H, double lmin, double lmax)
    {
        var l = Math.Abs(H.handLen) / 1900; //(hand_flag == false ? 1000 : 1); //??? ОТКУДА ПАРАМЕТРЫ НА L_MIN, L_MAX ???
        return l > lmin && l < lmax;
    }

    public bool CheckF0(RobotState SomeRobotState) => CheckHand(SomeRobotState.MoveableFingers[0], robL0Min, robL0Max);

    public bool CheckF1(RobotState SomeRobotState) => CheckHand(SomeRobotState.MoveableFingers[1], robL1Min, robL1Max);

    // public bool CheckF0(RobotState SomeRobotState) => CheckHand("F0", SomeRobotState);

    // public bool CheckF1(RobotState SomeRobotState) => CheckHand("F1", SomeRobotState);

    // public bool CheckHand(string FingIndex, RobotState SomeRobotState)
    // {
    //     double lmin = 0.0, lmax = 0.0, len = 0.0;
    //     if(FingIndex == "FH")
    //     {
    //         (lmin, lmax) = (robHy + robL1Min, robHy + robL1Max);
    //         len = SomeRobotState.MoveableFingers[2].handLen / 1900;
    //     }
    //     else if(FingIndex == "F0")
    //     {
    //         (lmin, lmax) = (robL0Min, robL0Max);
    //         len = SomeRobotState.MoveableFingers[0].handLen / 1900;
    //     }
    //     else if(FingIndex == "F1")
    //     {
    //         (lmin, lmax) = (robL1Min, robL1Max);
    //         len = SomeRobotState.MoveableFingers[1].handLen / 1900;
    //     }
    //     return len > lmin && len < lmax;
    // }

    public void IntermidiateInversion(double[] f2, double[] f0, double[] f1)
    {
        var NewRobotState02 = Inversion02(SetOfRobotStates[2], f2, f0);
        if(NewRobotState02 is not null)
        {
            SetOfRobotStates[0] = SetOfRobotStates[1];
            SetOfRobotStates[1] = SetOfRobotStates[2];
            var flagFing0 = CheckF0(NewRobotState02);
            if(flagFing0)
            {
                var NewRobotState1 = Inversion1(NewRobotState02, f1);
                var flagFing1 = CheckF1(NewRobotState1);
                if(flagFing1)
                    SetOfRobotStates[2] = NewRobotState1;
                else SetOfRobotStates[2] = null;
            }
        }
    }

    public void StartInversion(double[] f2, double[] f0, double[] f1)
    {
        var NewRobotState02 = Inversion02(SetOfRobotStates[1], f2, f0);
        if(NewRobotState02 is not null)
        {
            SetOfRobotStates[0] = SetOfRobotStates[1];
            SetOfRobotStates[1] = NewRobotState02;
            var flagFing0 = CheckF0(NewRobotState02);
            if(flagFing0)
            {
                var NewRobotState1 = Inversion1(NewRobotState02, f1);
                var flagFing1 = CheckF1(NewRobotState1);
                if(flagFing1)
                    SetOfRobotStates[2] = NewRobotState1;
                else SetOfRobotStates[2] = null;
            }
        }
    }

    public void Inversion(double[] f2, double[] f0, double[] f1)
    {
        if(Iterator == 0)
            StartInversion(f2, f0, f1);
        else IntermidiateInversion(f2, f0, f1);
        Iterator++;
    }

    public (double[], double[], double[]) NewPosForState(int[] f0_aff, int[] f1_aff, int[] f2_aff)
    {
        double[] Pos0 = [SetOfRobotStates[1].MoveableFingers[0].HandPos.X + f0_aff[0] * StepX, SetOfRobotStates[1].MoveableFingers[0].HandPos.Y + f0_aff[1] * StepY];
        double[] Pos1 = [SetOfRobotStates[1].MoveableFingers[1].HandPos.X + f1_aff[0] * StepX, SetOfRobotStates[1].MoveableFingers[1].HandPos.Y + f1_aff[1] * StepY];
        double[] Pos2 = [SetOfRobotStates[1].F2.X + f2_aff[0] * StepX, SetOfRobotStates[1].F2.Y + f2_aff[1] * StepY];
        return (Pos0, Pos1, Pos2);
    }

    public void NewState(int[] f0_aff, int[] f1_aff, int[] f2_aff)
    {
        var NewPos = NewPosForState(f0_aff, f1_aff, f2_aff);
        if(GridContext.CheckAccessity(NewPos.Item3, NewPos.Item1, NewPos.Item2))
            Inversion(NewPos.Item3, NewPos.Item1, NewPos.Item2);
    }

    private List<double[]> MakeNewAccessibleCoords(string FingerIndex)
    {
        var Res = new List<double[]>();
        Coordinate FingCoord;
        switch(FingerIndex)
        {
            case "F0":
                FingCoord = SetOfRobotStates[2].MoveableFingers[0].HandPos;
                break;
            case "F1":
                FingCoord = SetOfRobotStates[2].MoveableFingers[1].HandPos;
                break;
            case "F2":
                FingCoord = SetOfRobotStates[2].F2;
                break;
            default: throw new Exception("НЕИЗВЕСТНЫЙ ИДЕНТИФИКАТОР ПАЛЬЦА");
        }
        return FindAllPlanning(FingCoord.X, FingCoord.Y);
    }

    private List<double[]> FindAllPlanning(double x, double y)
    {
        // var x = SetOfRobotStates[2].MoveableFingers[1].HandPos.X;
        // var y = SetOfRobotStates[2].MoveableFingers[1].HandPos.Y;
        var PossibleIndexCoordArray = new List<double[]>
        {
            ([x, y]),
            ([x - StepX, y - 3 * StepY]),
            ([x + StepX, y - 3 * StepY]),
            ([x - StepX, y + 3 * StepY]),
            ([x + StepX, y + 3 * StepY]),
            ([x, y + 2 * StepY]),
            ([x, y - 2 * StepY]),
            ([x - StepX, y - StepY]),
            ([x - StepX, y + StepY]),
            ([x + StepX, y - StepY]),
            ([x + StepX, y + StepY])
        };
        return PossibleIndexCoordArray;
    }

    private List<double[]> FindAllPlanningForHose() => MakeNewAccessibleCoords("F1");

    public void FindAllAccessibleForHose()
    {
        var PlanForHose = FindAllPlanningForHose().FindAll(GridContext.CheckAccessity);
        PlanForHose.ForEach(coord => {if(InversionH(SetOfRobotStates[2], coord) is not null) GridContext.CoordsInfo[coord] = "Done";});
    }
}