using Avalonia.Controls;
using ScottPlot.Avalonia;
using Avalonia.Interactivity;
using System.Collections.Generic;
using KVP = System.Collections.Generic.KeyValuePair<string, ScottPlot.Color>;
using ScottPlot;
using Avalonia;
using System;
namespace PipeHandler2;

public partial class MainWindow : Window
{
    private Dictionary<string, Color> HoleStatesDescription = new(new List<KVP>([new KVP("Done", Colors.DarkGreen),
                                                                                 new KVP("Finger", Colors.Brown),
                                                                                 new KVP("Planned", Colors.Yellow),
                                                                                 new KVP("Goal", Colors.Orange),
                                                                                 new KVP("Inaccessible", Colors.Red),
                                                                                 new KVP("Accessible", Colors.Blue),
                                                                                 new KVP("Neutral", Colors.LightBlue)]));
    private TextBlock NameOfPipeDeskLabel;

    private CheckBox AutoRegimeFlagCheckBox;

    private TextBox InputJsonPath;

    private TextBlock StateInfoLabel;

    private AvaPlot gridPlot;

    private const string ConstPathToFile ="КуАЭС";

    private string[] States = ["Prev", "Curr", "Future"];

    private Robot PipeRobot;

    private void DrawHoles()
    {
        if(PipeRobot is not null)
        {
            foreach(var pair in PipeRobot.GetGridContext.CoordsInfo)
            {
                var C = gridPlot.Plot.Add.Circle(xCenter: pair.Key[0], yCenter: pair.Key[1], radius: PipeRobot.Radius);
                C.FillColor = HoleStatesDescription[pair.Value];
                C.LineColor = Colors.Black;
            }
            gridPlot.Plot.Axes.SetLimitsX(PipeRobot.GetGridContext.GetXBoards[0], PipeRobot.GetGridContext.GetXBoards[1]);
            gridPlot.Plot.Axes.SetLimitsY(PipeRobot.GetGridContext.GetYBoards[0], PipeRobot.GetGridContext.GetYBoards[1]);
            gridPlot.Refresh();
        }
    }
    
    private void Initialization(int[] F0, int[] F1, int[] F2, string Path = ConstPathToFile)
    {
        PipeRobot = new(F0, F1, F2, Path);
        DrawHoles();
        foreach(var state in States)
            DrawState(state);
        gridPlot.Refresh();
    }

    private int[] F0 = [6, 14];
    private int[] F1 = [6, 2];
    private int[] F2 = [7, 9];

    private List<ScottPlot.Plottables.Ellipse[]> StateFingersSchemas;

    private List<ScottPlot.Plottables.LinePlot[]> StateHandsSchemas;

    private void DrawFingers(Robot.RobotState SomeRobotState)
    {
        var FingerCircleSchemas = new ScottPlot.Plottables.Ellipse[3];
        FingerCircleSchemas[0] = gridPlot.Plot.Add.Circle(xCenter: SomeRobotState.MoveableFingers[0].HandPos.X, yCenter: SomeRobotState.MoveableFingers[0].HandPos.Y, radius: PipeRobot.Radius / 1.5);
        FingerCircleSchemas[1] = gridPlot.Plot.Add.Circle(xCenter: SomeRobotState.MoveableFingers[1].HandPos.X, yCenter: SomeRobotState.MoveableFingers[1].HandPos.Y, radius: PipeRobot.Radius / 1.5);
        FingerCircleSchemas[2] = gridPlot.Plot.Add.Circle(xCenter: SomeRobotState.F2.X, yCenter: SomeRobotState.F2.Y, radius: PipeRobot.Radius / 1.5);
        FingerCircleSchemas[0].FillColor = Colors.Red;
        FingerCircleSchemas[1].FillColor = Colors.Blue;
        FingerCircleSchemas[2].FillColor = Colors.Green;
    }

    private void DrawState(string State)
    {
        Robot.RobotState RS;
        ScottPlot.Plottables.LinePlot[] Hands;
        Color HandsCol;
        LinePattern HandsPattern;
        int LineWidth;
        switch(State)
        {
            case "Prev":
                RS = PipeRobot.GetRobotStates[0];
                Hands = StateHandsSchemas[0];
                HandsCol = Colors.Black;
                HandsPattern = LinePattern.Dashed;
                LineWidth = 3;
                break;
            case "Curr":
                RS = PipeRobot.GetRobotStates[1];
                Hands = StateHandsSchemas[1];
                HandsCol = Colors.Black;
                HandsPattern = LinePattern.Solid;
                LineWidth = 3;
                break;
            case "Future":
                RS = PipeRobot.GetRobotStates[2];
                Hands = StateHandsSchemas[2];
                HandsCol = Colors.DeepPink;
                HandsPattern = LinePattern.Solid;
                LineWidth = 2;
                break;
            default: throw new System.Exception("НЕИЗВЕСТНОЕ СОСТОЯНИЕ");
        }
        if(RS is not null)
        {
            DrawFingers(RS);
            Hands[0] = gridPlot.Plot.Add.Line(RS.MoveableFingers[0].HandPos.X, RS.MoveableFingers[0].HandPos.Y, RS.F2.X, RS.F2.Y);
            Hands[1] = gridPlot.Plot.Add.Line(RS.MoveableFingers[1].HandPos.X, RS.MoveableFingers[1].HandPos.Y, RS.F2.X, RS.F2.Y);
            foreach(var h in Hands)
            {
                h.LineColor = HandsCol;
                h.LinePattern = HandsPattern;
                h.LineWidth = LineWidth;
            }
        }
    }

    private void RemoveSomeState(string State)
    {
        ScottPlot.Plottables.Ellipse[] FingMarkers;
        ScottPlot.Plottables.LinePlot[] HandMarkers;
        switch(State)
        {
            case "Prev":
                FingMarkers = StateFingersSchemas[0];
                HandMarkers = StateHandsSchemas[0];
                break;
            case "Curr":
                FingMarkers = StateFingersSchemas[1];
                HandMarkers = StateHandsSchemas[1];
                break;
            case "Future":
                FingMarkers = StateFingersSchemas[2];
                HandMarkers = StateHandsSchemas[2];
                break;
            default: throw new Exception("НЕИЗВЕСТНОЕ НАИМЕНОВАНИЕ СОСТОЯНИЯ");
        }
        if(FingMarkers is not null)
        {
            for(var i = 0; i < 3; i++)
            // if(FingMarkers[i] is not null)
                gridPlot.Plot.Remove(FingMarkers[i]);
        }
        if(HandMarkers is not null)
        {
            if(HandMarkers[0] is not null)
                gridPlot.Plot.Remove(HandMarkers[0]);
            if(HandMarkers[1] is not null)
                gridPlot.Plot.Remove(HandMarkers[1]);
            // if(HandMarkers[2] is not null)
            //     gridPlot.Plot.Remove(HandMarkers[1]);
        }
        gridPlot.Refresh();
    }

    public MainWindow()
    {
        InitializeComponent();
        StateFingersSchemas = new();
        for(var i = 0; i < 3; i++)
            StateFingersSchemas.Add(new ScottPlot.Plottables.Ellipse[3]);
        StateHandsSchemas = new();
        for(var i = 0; i < 3; i++)
            StateHandsSchemas.Add(new ScottPlot.Plottables.LinePlot[2]);
        gridPlot = this.Find<AvaPlot>("GridPlot");
        InputJsonPath = this.Find<TextBox>("InputPathToJson");
        NameOfPipeDeskLabel = this.Find<TextBlock>("PipeDeskName");
        StateInfoLabel = this.Find<TextBlock>("StateInfo");
        AutoRegimeFlagCheckBox = this.Find<CheckBox>("CheckForAuto");
        gridPlot.Plot.Axes.SquareUnits();
        Initialization(F0, F1, F2);
    }

    public void ClickRenew(object sender, RoutedEventArgs args)
    {
        foreach(var state in States)
            RemoveSomeState(state);
        PipeRobot.NewState([2, 2], [2, 2], [2, 2]);
        foreach(var state in States)
            DrawState(state);
        gridPlot.Refresh();
    }

    public void EnterPathToFile(object sender, RoutedEventArgs args)
    {
        gridPlot.Plot.Clear();
        var Path = InputJsonPath.Text is not null ? InputJsonPath.Text : ConstPathToFile;
        Initialization(F0, F1, F2, Path);
    }

    public void ShowFinalState(object sender, RoutedEventArgs args)
    {
        //PipeRobot.FindAllAccessibleForHose();
        DrawHoles();
    }

    public void Fing0Accessible(object sender, RoutedEventArgs args)
    {

    }

    public void Fing1Accessible(object sender, RoutedEventArgs args)
    {

    }

    public void Fing2Accessible(object sender, RoutedEventArgs args)
    {

    }
}