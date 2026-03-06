using System.Numerics;
using System;

public partial class CelCon
{
    public static void fixCelPosition(in int ci, in double[] diffPos, in double d){ //相対位置にする
        float radius = Main.CelE.radius[ci] * Main.Inputs.scale[0];//km

        Godot.Vector3 gvec;
        if(d > Main.MAXD){
            gvec.X = (float)(diffPos[0] * Main.MAXD * 1000 / d);//各成分の割合 * MAXD
            gvec.Y = (float)(diffPos[1] * Main.MAXD * 1000 / d);
            gvec.Z = (float)(diffPos[2] * Main.MAXD * 1000 / d);
            radius = radius * (float)(Main.MAXD * 1000 / d);

            float minRadius = (float)Main.MAXD * 1000 *  MathF.Sin(Main.FOV/300);
            if ( radius < minRadius){
                    radius = minRadius;
            }
        }else{//ちかいとき
            gvec.X = (float)diffPos[0] * 1000;
            gvec.Y = (float)diffPos[1] * 1000;
            gvec.Z = (float)diffPos[2] * 1000;
        }

        Main.transform.Origin = gvec;
        Main.transform.Basis.X = new Godot.Vector3( radius , 0 , 0 );
        Main.transform.Basis.Y = new Godot.Vector3( 0 , radius , 0 );
        Main.transform.Basis.Z = new Godot.Vector3( 0 , 0 , radius );
        Main.CelN[ci].Transform = Main.transform;
    }
    
}
