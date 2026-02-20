using System;
using System.Numerics;

public partial class Main{
    
    public static string displayTEL(int i){
        string result = "";
        //result += $"R SHT : {Math.Round(RSHT            , 4)}\n";
        result += $"R UTC : {Main.SHT2date(RSHT)            }\n";
        //result += $"V SHT : {Math.Round(VSHT            , 4)}\n";
        result += $"V UTC : {Main.SHT2date(VSHT)            }\n";
        result += $"acce:  {Math.Round(Probe.acce[0]   , 4)} , {Math.Round(Probe.acce[1]   , 4)} , {Math.Round(Probe.acce[2]   , 4)}\n";                              
        result += $"velo:  {Math.Round(Probe.velo[0]   , 4)} , {Math.Round(Probe.velo[1]   , 4)} , {Math.Round(Probe.velo[2]   , 4)}  \n";
        result += $"pos:   {Math.Round(positions[0,0]  , 4)} , {Math.Round(positions[0,1]  , 4)} , {Math.Round(positions[0,2]  , 4)}\n";
        result += $"aAcce: {Math.Round(Probe.aAcce.v.X , 4)} , {Math.Round(Probe.aAcce.v.Y , 4)} , {Math.Round(Probe.aAcce.v.Z , 4)} , {Math.Round(Probe.aAcce.s , 4)}\n";
        result += $"aVelo: {Math.Round(Probe.aVelo.v.X , 4)} , {Math.Round(Probe.aVelo.v.Y , 4)} , {Math.Round(Probe.aVelo.v.Z , 4)} , {Math.Round(Probe.aVelo.s , 4)}\n";
        result += $"angleQ:{Probe.anglQ.ToString()}\n";
        result += $"solarE:{Probe.solarEnergy} W\n";
        result += $"scale:{Inputs.scale[0]}\n";

        result += $"\nrate of t0 :{Probe.thruster[0].rate}\n";
        result += $"rate of t1 :{Probe.thruster[1].rate}\n";
        result += $"rate of t2 :{Probe.thruster[2].rate}\n";
        result += $"rate of t3 :{Probe.thruster[2].rate}\n";
        return result;
    }
    public static string mtostring(Vector3 v){
        return $"{v.X} , {v.Y} , {v.Z}";
    }
    public static string gtostring(Godot.Vector3 v){
        return $"{v.X} , {v.Y} , {v.Z}";
    }
    public static string gtostring2(Godot.Vector2 v){
        return $"{v.X} , {v.Y}";
    }
}
