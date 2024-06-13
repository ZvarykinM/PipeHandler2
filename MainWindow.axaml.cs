using Avalonia.Controls;
using ScottPlot.Avalonia;
using Avalonia.Interactivity;
namespace PipeHandler2;

public partial class MainWindow : Window
{
    // private AvaPlot gridPlot;

    //private Robot PipeRobot;

    private double FingerRad; 

    private TextBlock NameOfPipeDeskLabel;

    private CheckBox AutoRegimeFlagCheckBox;

    private TextBox InputJsonPath;

    private TextBlock StateInfoLabel;

    private AvaPlot gridPlot;
    public MainWindow()
    {
        InitializeComponent();
        var gridPlot = this.Find<AvaPlot>("GridPlot");
        InputJsonPath = this.Find<TextBox>("InputPathToJson");
        NameOfPipeDeskLabel = this.Find<TextBlock>("PipeDeskName");
        StateInfoLabel = this.Find<TextBlock>("StateInfo");
        AutoRegimeFlagCheckBox = this.Find<CheckBox>("CheckForAuto");
    }

    public void ClickRenew(object sender, RoutedEventArgs args)
    {
        
    }

    public void EnterPathToFile(object sender, RoutedEventArgs args)
    {
        
    }

    public void ShowFinalState(object sender, RoutedEventArgs args)
    {
        
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