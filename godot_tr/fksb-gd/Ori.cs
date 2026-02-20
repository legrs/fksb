using Godot;
using System;
using static Main;

public partial class Ori : ColorRect
{

    float sin,cos;

    Vector2[] arrowHead = new Vector2[3];
    Vector2 vec1 = new Vector2(0,0);
    Vector2 vec2 = new Vector2(0,0);

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        sin = MathF.Sin(Main.arrowAngl);
        cos = MathF.Cos(Main.arrowAngl);
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
    {
        QueueRedraw();
    }
    public void makeArrowHead(Vector2[] poly, Vector2 v1, Vector2 v2){
        v1 = v1 + v2;
        v2 = v2 * Main.arrowLength / v2.Length();
        poly[0].X = v1.X +  v2.X*cos - v2.Y*sin;
        poly[0].Y = v1.Y +  v2.X*sin + v2.Y*cos;

        poly[1].X = v1.X +  v2.X*cos + v2.Y*sin;
        poly[1].Y = v1.Y + -v2.X*sin + v2.Y*cos;

        poly[2] = v1;
    }

    public override void _Draw(){
    }
}
