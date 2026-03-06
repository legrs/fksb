//using Godot;
using System;
using System.Numerics;
using static Main;

public partial class Canvas : Godot.ColorRect
{
    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
    {
    }
    //位置x1,y1の(x2,y2)を画面端まで伸ばしたときに何倍になるかを計算する
    private float calMax(float x1, float y1, float x2, float y2){
        if(x2==-0)
            x2 = 0; //x2 == -0 のとき負の無限大に発散するのを防ぐため
        if(y2==-0)
            y2 = 0; //x2 == -0 のとき負の無限大に発散するのを防ぐため
        float k1,k2;
        if(0<=x2 && 0<=y2){
            return MathF.Min((Main.canvas.Size.Y-y1)/y2,(Main.canvas.Size.X-x1)/x2);
        }else if(x2<0 && 0<=y2){
            return MathF.Min((Main.canvas.Size.Y-y1)/y2,-x1/x2);
        }else if(x2<0 && y2<0){
            return MathF.Min(-y1/y2,-x1/x2);
        }else{
            return MathF.Min(-y1/y2,(Main.canvas.Size.X-x1)/x2);
        }

    }

public static Godot.Vector2 del = new Godot.Vector2();
public static Godot.Vector2 del2 = new Godot.Vector2();

    public override void _Draw()
    {
        /*
        for(int i=0; i<Main.orbits.Count; i++){
            //GD.Print(Main.orbits[i].e);
            if(Main.orbits[i].e == 0){
                //円
                float sita=0;
                Godot.Vector2 pos = new Godot.Vector2(
                        Main.orbits[i].l + Main.orbits[i].x, 
                        Main.orbits[i].l + Main.orbits[i].y);
                for(int j=0; j<Main.drawPoly; j++){
                    sita = 2 * MathF.PI * (j/Main.drawPoly);
                    DrawLine(new Godot.Vector2(Main.orbits[i].l * MathF.Cos(sita) + Main.orbits[i].x, Main.orbits[i].l * MathF.Cos(sita) + Main.orbits[i].y) , pos , new Godot.Color(1,1,1));
                    pos.X = Main.orbits[i].l * MathF.Cos(sita) + Main.orbits[i].x;
                    pos.Y = Main.orbits[i].l * MathF.Sin(sita) + Main.orbits[i].y;
                }
            }else if(Main.orbits[i].e == 1){
                //放物線
            }else{
                //楕円
                float r1;
                Godot.Vector2 pos1;        
                float r2 = Main.orbits[i].l / 
                        (1+ (Main.orbits[i].e));
                float sita=0;
                Godot.Vector2 pos2 = new Godot.Vector2(
                                r2 * MathF.Cos(Main.orbits[i].angl) + Main.orbits[i].x, 
                                r2 * MathF.Sin(Main.orbits[i].angl) + Main.orbits[i].y);
                for(int j=0; j<Main.drawPoly; j++){
                    sita = 2 * MathF.PI * j/Main.drawPoly;

                    r1 = Main.orbits[i].l / 
                        (1+ (Main.orbits[i].e * MathF.Cos(sita)));
                    pos1.X = r1 * MathF.Cos(sita + Main.orbits[i].angl) + Main.orbits[i].x;
                    pos1.Y = r1 * MathF.Sin(sita + Main.orbits[i].angl) + Main.orbits[i].y;
                    //GD.Print(r1);
                    //GD.Print(r2);
                    DrawLine(pos2, pos1 , new Godot.Color(1,1,1));
                    r2 = Main.orbits[i].l / 
                        (1+ (Main.orbits[i].e * MathF.Cos(sita)));
                    pos2.X = r2 * MathF.Cos(sita + Main.orbits[i].angl) + Main.orbits[i].x;
                    pos2.Y = r2 * MathF.Sin(sita + Main.orbits[i].angl) + Main.orbits[i].y;
                    

                }
            }
        }
        */

        float s0 = MathF.Sin(Main.camAngl[1, 0]);
        float c0 = MathF.Cos(Main.camAngl[1, 0]);
        float s1 = MathF.Sin(Main.camAngl[1, 1]);
        float c1 = MathF.Cos(Main.camAngl[1, 1]);
        Godot.Vector2 pos;
        Godot.Color col = new Godot.Color(1,1,1);

        // add points and orbits  0:probe 1:sun 2:mercury 3:...
        Main.points.Clear();

        pos = Main.paraPro(  //probe
                (float)Main.vpositions[0, 0] * Main.Inputs.scale[1],
                (float)Main.vpositions[0, 1] * Main.Inputs.scale[1],
                (float)Main.vpositions[0, 2] * Main.Inputs.scale[1],
                in s0,in s1,in c0,in c1);
        Main.points.Add(pos);

        pos = new Godot.Vector2(0,0); //sun
        Main.points.Add(pos);
        for(int ci=1; ci<Main.maxCI; ci++){
            Main.calOrbiterPosition(ci , Main.VSHT , true);
            pos = Main.paraPro(
                    (float)Main.vpositions[ci, 0] * Main.Inputs.scale[1],
                    (float)Main.vpositions[ci, 1] * Main.Inputs.scale[1],
                    (float)Main.vpositions[ci, 2] * Main.Inputs.scale[1],
                    in s0,in s1,in c0,in c1);
            Main.points.Add(pos);
        }
        for(int i=0;i<orbits.Count;i++){
            float r = (float)orbits[i].l / (1 + orbits[i].e * MathF.Cos(orbits[i].f));
            float x = r * MathF.Cos(orbits[i].f + orbits[i].om);
            float y = r * MathF.Sin(orbits[i].f + orbits[i].om);
            pos = Main.paraPro(
             (x * Main.CelE.o[i].c2) - (y * Main.CelE.o[i].c1 * Main.CelE.o[i].s2),
             (x * Main.CelE.o[i].s2) + (y * Main.CelE.o[i].c1 * Main.CelE.o[i].c2),
             y * Main.CelE.o[i].s1,
                in s0,in s1,in c0,in c1);
            Main.points.Add(pos);
        }

        Main.orbits.RemoveAt(0); //探査機の軌道
        Vector3 rP,rV;
        //Main.gravCenter = 3;
        if(Main.gravCenter==0){
            rP = new Vector3((float)Main.positions[0,0],(float)Main.positions[0,1],(float)Main.positions[0,2]);
            rV = new Vector3((float)Main.Probe.velo[0],(float)Main.Probe.velo[1],(float)Main.Probe.velo[2]);
        }else{
            rP = new Vector3(
                                (float)(Main.positions[0,0]-Main.positions[Main.gravCenter,0]),
                                (float)(Main.positions[0,1]-Main.positions[Main.gravCenter,1]),
                                (float)(Main.positions[0,2]-Main.positions[Main.gravCenter,2]));
            rV = new Vector3(
                                (float)(Main.Probe.velo[0])-Main.CelE.v[Main.gravCenter].X,
                                (float)(Main.Probe.velo[1])-Main.CelE.v[Main.gravCenter].Y,
                                (float)(Main.Probe.velo[2])-Main.CelE.v[Main.gravCenter].Z);
        }
        //Godot.GD.Print(rP.Length()," ",rV.Length());
        Main.orbits.Add(Main.solveOrbit(
                    in rV, 
                    in Main.gravCenter, 
                    in rP,
                    rV.Length(),
                    rP.Length()));

        if(bSelecting.Count == 2){
            orbits.RemoveAt(orbits.Count-1);
            orbits.Add(new Orbit());
            float n = MathF.Sqrt(MathF.Pow((float)positions[bSelecting[1] , 0] , 2) + MathF.Pow((float)positions[bSelecting[1] , 1] , 2) + MathF.Pow((float)positions[bSelecting[1] , 2] , 2));
            float r = n * (-Main.points[bSelecting[1]+1].Dot(Main.Inputs.MMV-canvas.GetGlobalPosition() - Canvas.del) / (MathF.Pow(Main.points[bSelecting[1]+1].X,2)+MathF.Pow(Main.points[bSelecting[1]+1].Y,2))); 
            orbits[orbits.Count-1].e = MathF.Abs(r-n)/(r+n);
            orbits[orbits.Count-1].f = 0;
            orbits[orbits.Count-1].s1 = CelE.o[bSelecting[1]].s1;
            orbits[orbits.Count-1].c1 = CelE.o[bSelecting[1]].c1;
            orbits[orbits.Count-1].s2 = CelE.o[bSelecting[1]].s2;
            orbits[orbits.Count-1].c2 = CelE.o[bSelecting[1]].c2;
            orbits[orbits.Count-1].om = CelE.o[bSelecting[1]].om + CelE.o[bSelecting[1]].f;
            if( r < n )
                orbits[orbits.Count-1].om += MathF.PI;
            if( 2*MathF.PI < orbits[orbits.Count-1].om){
                orbits[orbits.Count-1].om -= 2*MathF.PI;
            }
            if( 2*MathF.PI < orbits[orbits.Count-1].om){
                orbits[orbits.Count-1].om -= 2*MathF.PI;
            }
            orbits[orbits.Count-1].c0 = MathF.Cos(orbits[orbits.Count-1].om);
            orbits[orbits.Count-1].s0 = MathF.Sin(orbits[orbits.Count-1].om);
            orbits[orbits.Count-1].ci = bSelecting[0];
            orbits[orbits.Count-1].period = (double)(2 * MathF.PI * MathF.Sqrt(MathF.Pow(r+n,3)/(float)CelE.grav[bSelecting[0]])); //peri = 2 pi (a^3 / μ) wikipediaより
            orbits[orbits.Count-1].epo = 2 * MathF.PI * (( (float)VSHT/(float)orbits[orbits.Count-1].period)%1);
            orbits[orbits.Count-1].l = (2*r*n)/(r+n);
            Godot.GD.Print(bSelecting[0], " ", bSelecting[1]);

            //bSelecting.Clear();
            //MODE[1] = 0;
        }

        Godot.GD.Print(orbits.Count);
        //del調整
        del2 = Main.canvas.Size/2 + Main.Inputs.scale[1]*1000000*(Main.Inputs.camDiff-Main.canvas.Size/2);
        del = del2 - Main.points[Main.viewCenter]; //delは中心星の画面上の位置

        //Main.Inputs.scale[1]*1000000*(Main.Inputs.camDiff-Main.canvas.Size/2)

        //draw grid and axis
        float a = 15; //やじるしのながさ
        float b = 6; //やじるしのはばの半分
        for(int i=0; i<3; i++){
            if(i==0){
                pos = Main.paraPro(1000,0,0,in s0,in s1,in c0,in c1);
                col = new Godot.Color(1,0,0);
            }else if(i==1){
                pos = Main.paraPro(0,1000,0,in s0,in s1,in c0,in c1);
                col = new Godot.Color(0,1,0);
            }else{
                pos = Main.paraPro(0,0,1000,in s0,in s1,in c0,in c1);
                col = new Godot.Color(0,0,1);
            }
            float k1 = calMax(del2.X,del2.Y,pos.X,pos.Y);
            float k2 = calMax(del2.X,del2.Y,-pos.X,-pos.Y);
            DrawLine(pos*k1 + del2, -pos*k2 + del2 , col);
            float len = pos.Length();
            Godot.Vector2[] arrow = new Godot.Vector2[]{
                new Godot.Vector2(pos.X*k1 + del2.X , pos.Y*k1 + del2.Y),
                new Godot.Vector2(pos.X*(k1 - a/len) - pos.Y*b/len + del2.X  ,  pos.Y*(k1 - a/len) + pos.X*b/len + del2.Y),
                new Godot.Vector2(pos.X*(k1 - a/len) + pos.Y*b/len + del2.X  ,  pos.Y*(k1 - a/len) - pos.X*b/len + del2.Y)
            };
            DrawPolygon( arrow , new Godot.Color[]{col});
        }

        //draw other option
        pos = new Godot.Vector2(8,8);
        DrawLine(del2+pos, del2-pos , new Godot.Color(1,1,1));
        pos = new Godot.Vector2(-8,8);
        DrawLine(del2+pos, del2-pos , new Godot.Color(1,1,1));

        // draw orbits
        float mouseSlope = (Main.Inputs.MMV.Y-del.Y-Main.canvas.GetGlobalPosition().Y)/(Main.Inputs.MMV.X-del.X-Main.canvas.GetGlobalPosition().X);
        col = new Godot.Color(1,1,1);
        for(int ci=0; ci<Main.maxCI; ci++){
            float l = (float)Main.CelE.o[ci].l * Main.Inputs.scale[1];
            float r = l / (1 + (Main.CelE.o[ci].e));
            float sita;
            float d = float.MaxValue;
            float mSita = 0;

            Vector3 tmp = new Vector3(
                    r * Main.CelE.o[ci].c0,
                    r * Main.CelE.o[ci].s0,
                    0
                    );
            Godot.Vector2 pos1;
            pos1.X = (tmp.X * Main.CelE.o[ci].c2) - (tmp.Y * Main.CelE.o[ci].c1 * Main.CelE.o[ci].s2);
            pos1.Y = (tmp.X * Main.CelE.o[ci].s2) + (tmp.Y * Main.CelE.o[ci].c1 * Main.CelE.o[ci].c2);
            tmp.Z = tmp.Y * Main.CelE.o[ci].s1;
            pos1 = Main.paraPro(pos1.X, pos1.Y, tmp.Z, in s0, in s1, in c0, in c1);
            pos1.X += del.X; pos1.Y += del.Y + 10;
            Godot.Vector2 pos2;
            for(int j=0; j<=Main.drawPoly; j++){
                sita = 2 * MathF.PI * j / Main.drawPoly;
                tmp.X = MathF.Cos(sita);
                /*
                if(tmp.X < 0.005){
                    tmp.X = 0.005F;
                }
                */
                sita = sita * (Main.CelE.o[ci].e*tmp.X+1);

                r = l / (1 + (Main.CelE.o[ci].e * MathF.Cos(sita)));
                tmp = new Vector3(
                        r * MathF.Cos(sita + Main.CelE.o[ci].om),
                        r * MathF.Sin(sita + Main.CelE.o[ci].om),
                        0
                        );
                pos2.X = (tmp.X * Main.CelE.o[ci].c2) - (tmp.Y * Main.CelE.o[ci].c1 * Main.CelE.o[ci].s2);
                pos2.Y = (tmp.X * Main.CelE.o[ci].s2) + (tmp.Y * Main.CelE.o[ci].c1 * Main.CelE.o[ci].c2);
                tmp.Z = tmp.Y * Main.CelE.o[ci].s1;
                pos2 = Main.paraPro(pos2.X, pos2.Y, tmp.Z, in s0, in s1, in c0, in c1);
                pos2 = pos2 + del;
                DrawLine(pos1, pos2 , col);
                pos1 = pos2;
                if(Main.Inputs.Drag==ci+1){ //ドラックで時間変更
                    float tan;
                    if((Main.Inputs.MMV.X-del.X-Main.canvas.GetGlobalPosition().X)==0 && pos1.X-del.X!=0){
                        tan = MathF.Abs(65536 - ((pos1.Y-del.Y)/(pos1.X-del.X)));
                    }else{
                        tan = MathF.Abs(mouseSlope - ((pos1.Y-del.Y)/(pos1.X-del.X)));
                    }
                    if((Main.Inputs.MMV.Y-del.Y-Main.canvas.GetGlobalPosition().Y)*(pos1.Y-del.Y)>0 && tan < d){
                        
                        d = tan;
                        mSita = sita;
                    }
                }
            }
            if(Main.Inputs.Drag==ci+1){ //ドラックで時間変更
                //Godot.GD.Print(mouseSlope );
                //mSita += MathF.PI/2;
                float M0 = mSita - Main.CelE.o[ci].e * MathF.Sin(mSita) ;
                M0 = mSita;
                float M1 = (float)Main.VSHT / (float)Main.CelE.o[ci].period;
                int p = (int)MathF.Truncate(M1);
                M1 = M1 % 1; //小数部分o
                M1 = M1 * 2 * MathF.PI; //平均近点角rad
                M1 += Main.CelE.o[ci].epo;
                if( 2*MathF.PI <= M1){
                    M1 -= 2*MathF.PI;
                }
                M0 -= Main.CelE.o[ci].epo;
                if( M0 <= 0){
                    M0 += 2*MathF.PI;
                }
                M1 -= Main.CelE.o[ci].epo;
                if( M1 <= 0){
                    M1 += 2*MathF.PI;
                }
                if(M1 - M0 >= MathF.PI){ //近点越え
                    Godot.GD.Print("++++ ",Main.toDegF(M0)," ",Main.toDegF(M1));
                    p++;
                    M0 += 0.01F;
                }else if(M0 - M1 >= MathF.PI){
                    Godot.GD.Print("---- ",Main.toDegF(M0)," ",Main.toDegF(M1));
                    p--;
                    M0 -= 0.01F;
                }
                Godot.GD.Print(Main.toDegF(M0)," ",Main.toDegF(M1)," ",(p+(M0)/(2*Math.PI)));
                Main.VSHT = (double)(p + M0/(2*Math.PI)) * Main.CelE.o[ci].period;
            }
        }
                    //Godot.GD.Print(((Main.Inputs.MMV.X-del.X), "  ",(Main.Inputs.MMV.Y-del.Y))," ",del);
                    //Godot.GD.Print(Main.Inputs.MMV-del-Main.canvas.GetGlobalPosition()," ",del);
        for(int i=0; i<Main.orbits.Count; i++){
            float l = (float)Main.orbits[i].l * Main.Inputs.scale[1];
            float r = l / (1 + (Main.orbits[i].e));
            Vector3 tmp = new Vector3(
                    r * Main.orbits[i].c0,
                    r * Main.orbits[i].s0,
                    0
                    );
            Godot.Vector2 pos1;
            pos1.X = (tmp.X * Main.orbits[i].c2) - (tmp.Y * Main.orbits[i].c1 * Main.orbits[i].s2);
            pos1.Y = (tmp.X * Main.orbits[i].s2) + (tmp.Y * Main.orbits[i].c1 * Main.orbits[i].c2);
            tmp.Z = tmp.Y * Main.orbits[i].s1;
            pos1 = Main.paraPro(pos1.X, pos1.Y, tmp.Z, in s0, in s1, in c0, in c1);
            pos1.X += del.X+Main.points[Main.orbits[i].ci+1].X;
            pos1.Y += del.Y + 10+Main.points[Main.orbits[i].ci+1].Y;
            Godot.Vector2 pos2;
            float sita;
            for(int j=0; j<=Main.drawPoly; j++){
                sita = 2 * MathF.PI * j/Main.drawPoly;
                if(Main.orbits[i].e < 1)
                    sita = sita + Main.orbits[i].e*MathF.Sin(sita);
                r = l / (1 + (Main.orbits[i].e * MathF.Cos(sita)));
                tmp = new Vector3(
                        r * MathF.Cos(sita + Main.orbits[i].om),
                        r * MathF.Sin(sita + Main.orbits[i].om),
                        0
                        );
                pos2.X = (tmp.X * Main.orbits[i].c2) - (tmp.Y * Main.orbits[i].c1 * Main.orbits[i].s2);
                pos2.Y = (tmp.X * Main.orbits[i].s2) + (tmp.Y * Main.orbits[i].c1 * Main.orbits[i].c2);
                tmp.Z = tmp.Y * Main.orbits[i].s1;
                pos2 = Main.paraPro(pos2.X, pos2.Y, tmp.Z, in s0, in s1, in c0, in c1);
                pos2 += del+Main.points[Main.orbits[i].ci+1];

                sita = 2 * MathF.PI * (j+1)/Main.drawPoly;
                r = l / (1 + (Main.orbits[i].e * MathF.Cos(sita)));
                if(r < 0){
                  //双曲線の描画時によけいな線をえがかないため
                    pos2 = pos1 + calMax(pos1.X,pos1.Y,pos2.X - pos1.X,pos2.Y - pos1.Y) * (pos2 - pos1);

                    DrawLine(pos1, pos2 , new Godot.Color(1,1,1)); //発散させる線1

                    j = Main.drawPoly - j;
                    sita = 2 * MathF.PI * (j+1)/Main.drawPoly;
                    r = l / (1 + (Main.orbits[i].e * MathF.Cos(sita)));
                    tmp = new Vector3(
                            r * MathF.Cos(sita + Main.orbits[i].om),
                            r * MathF.Sin(sita + Main.orbits[i].om),
                            0
                            );
                    pos1.X = (tmp.X * Main.orbits[i].c2) - (tmp.Y * Main.orbits[i].c1 * Main.orbits[i].s2);
                    pos1.Y = (tmp.X * Main.orbits[i].s2) + (tmp.Y * Main.orbits[i].c1 * Main.orbits[i].c2);
                    tmp.Z = tmp.Y * Main.orbits[i].s1;
                    pos1 = Main.paraPro(pos1.X, pos1.Y, tmp.Z, in s0, in s1, in c0, in c1);
                    pos1 += del+Main.points[Main.orbits[i].ci+1];
                    
                    j++;

                    sita = 2 * MathF.PI * (j+1)/Main.drawPoly;
                    r = l / (1 + (Main.orbits[i].e * MathF.Cos(sita)));
                    tmp = new Vector3(
                            r * MathF.Cos(sita + Main.orbits[i].om),
                            r * MathF.Sin(sita + Main.orbits[i].om),
                            0
                            );
                    pos2.X = (tmp.X * Main.orbits[i].c2) - (tmp.Y * Main.orbits[i].c1 * Main.orbits[i].s2);
                    pos2.Y = (tmp.X * Main.orbits[i].s2) + (tmp.Y * Main.orbits[i].c1 * Main.orbits[i].c2);
                    tmp.Z = tmp.Y * Main.orbits[i].s1;
                    pos2 = Main.paraPro(pos2.X, pos2.Y, tmp.Z, in s0, in s1, in c0, in c1);
                    pos2 += del+Main.points[Main.orbits[i].ci+1];
                    pos1 = pos2 + calMax(pos2.X,pos2.Y,pos2.X - pos2.X,pos1.Y - pos2.Y) * (pos1 - pos2);

                }else{
                }
                DrawLine(pos1, pos2 , new Godot.Color(1,1,1));
                pos1 = pos2;
            }
        }

        //draw directions(vector)
        Godot.Vector2 pos3;
        float transScale = Main.Probe.node.Transform.Basis.Z.Length()*0.06F;
        pos3 = Main.paraPro(Main.Probe.node.Transform.Basis.Z.X, Main.Probe.node.Transform.Basis.Z.Y, Main.Probe.node.Transform.Basis.Z.Z, in s0, in s1, in c0, in c1);
        DrawLine(Main.points[0]+del, Main.points[0] + del + pos3/transScale , new Godot.Color(1,1,1));
        float length = 5F;
        //pos3 = Main.paraPro((float)Main.Probe.velo[0], (float)Main.Probe.velo[1], (float)Main.Probe.velo[2], in s0, in s1, in c0, in c1);
        pos3 = Main.paraPro((float)Main.Probe.velo[0]-Main.CelE.v[3].X, (float)Main.Probe.velo[1]-Main.CelE.v[3].Y, (float)Main.Probe.velo[2]-Main.CelE.v[3].Z, in s0, in s1, in c0, in c1);
        DrawLine(Main.points[0]+del, Main.points[0] + del + pos3*length , new Godot.Color(0,1,1));
        pos3 = Main.paraPro(Main.CelE.v[3].X, Main.CelE.v[3].Y, Main.CelE.v[3].Z, in s0, in s1, in c0, in c1);
        DrawLine(Main.points[4]+del, Main.points[4] + del + pos3 , new Godot.Color(0,1,1));

        //draw points
        Godot.Color color;
        float rad = Main.pointMinRad;
        for(int i=0; i<Main.points.Count; i++){
            if(i==0)
                color = new Godot.Color(1,1,0);
            else if(i==1)
                color = new Godot.Color(1,0,0);
            else
                color = new Godot.Color(1,1,1);
            if(i == Main.selecting)
                color = new Godot.Color(0,1,0);
            if(i == Main.hovering)
                color = new Godot.Color(0,0,1);
            for(int j=0;j<Main.bSelecting.Count;j++){
                if(i == Main.bSelecting[j])
                    color = new Godot.Color(1,0,1);
            }
            if( i==0 ){
                rad = Main.pointMinRad;
            }else{
                if(i<=maxCI){
                    rad = Main.CelE.radius[i-1] * Main.Inputs.scale[1];
                    if( rad < Main.pointMinRad )
                        rad = Main.pointMinRad;
                }
            }
            DrawCircle(Main.points[i]+del, rad, color);
        }
        
        /*
        for(int i=0; i<Main.bSelecting.Count; i++){
            DrawCircle(Main.points[Main.bSelecting[i]], Main.pointMinRad, new Godot.Color(0.3F,0.3F,1));
        }
        */


        //
    }

}
