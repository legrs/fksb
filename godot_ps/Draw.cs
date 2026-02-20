using System;
using System.Collections.Generic;
using System.Numerics;
using G = Godot;
using GD = Godot.GD;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<double>;
using static Main;

public partial class Draw : G.ColorRect
{
    float rad = 3;
    G.Color WHITE = new G.Color(1,1,1);
    G.Color BLUE  = new G.Color(0,0,1);
    G.Color GREEN = new G.Color(0,1,0);
    G.Color RED   = new G.Color(1,0,0);
    G.Color R_B   = new G.Color(1,0,1);
    G.Color R_G   = new G.Color(1,1,0);
    G.Color B_G   = new G.Color(0,1,1);

    public static G.Vector2 ppos;
    G.Vector2 dp;

    public static List<float> blue = new List<float>();
    public static List<float> green = new List<float>();
    public static List<float> red = new List<float>();
    public static List<float> black = new List<float>();

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
    {
        QueueRedraw();
    }

    public override void _Draw(){
        ppos.X = (float)(pos.x/camd) * Size.X;
        ppos.Y = -(float)(pos.y/camd) * Size.Y;
        dp = ppos + Size/2;
        DrawCircle(dp, rad, RED);

        dp = new G.Vector2(MathF.Cos(-camAngl[0]) * Size.X / 2, MathF.Sin(-camAngl[0]) * Size.Y / 2);
        dp = dp + Size/2;
        DrawCircle(dp, rad, WHITE);

        if(green.Count < 2) return;

        G.Vector2[] points = new G.Vector2[green.Count];

        DrawLine(new G.Vector2( - Position.X , - Position.Y + svc.Size.Y/2) , new G.Vector2( 0, - Position.Y + svc.Size.Y/2) , WHITE , 2);

        for(int i=0; i<green.Count; i++){
            points[i].X = i*5 - Position.X;
            points[i].Y = blue[i] - Position.Y + svc.Size.Y/2;
        }
        DrawPolyline(points, BLUE, 2);

        for(int i=0; i<green.Count; i++){
            points[i].X = i*5 - Position.X;
            points[i].Y = red[i] - Position.Y + svc.Size.Y/2;
        }
        DrawPolyline(points, RED, 2);

        for(int i=0; i<green.Count; i++){
            points[i].X = i*5 - Position.X;
            points[i].Y = green[i] - Position.Y + svc.Size.Y/2;
        }
        DrawPolyline(points, GREEN, 2);

        for(int i=0; i<green.Count; i++){
            points[i].X = i*5 - Position.X;
            points[i].Y = black[i] - Position.Y + svc.Size.Y/2;
        }
        DrawPolyline(points, G.Colors.Black, 2);

    }
}
