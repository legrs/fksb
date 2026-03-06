using System;
using System.Linq;
using System.Collections.Generic;
using System.Numerics;

public partial class Main : Godot.Node3D
{
    //debugよう変数
    public static int maxCI=1+    9  ; //ここを変更 1<= <=9


    public struct Inputs{
        // 0:cg 1:design
        public static bool MBL = false;//mouse light button
        public static bool MBR = false;// \\   right
        public static bool MBM = false;// \\   middle
        public static float[] MWV = new float[2]{0,0}; //wheel
        public static Godot.Vector2 MMV; //mouse
        public static Godot.Vector2 camDiff = new Godot.Vector2(242,145);
        public static float[] scale = new float[2]{1,0.000001F};
        public static bool isMouseInside = false;
        public static int Drag = -2;
    }
    
   //定数 
    public static float sensit = 1.3F;
    public static float CAMD = 32;
    public static double[] ts = new double[2]{1,1}; //0:rtimescale 1:vtimescale
    public static double MAXD = 4.096;
    public static float FOV = (float)toRad(50); //rad
    public static float WHEEL_STEP = 0.05F;
    public static int[] MODE = new int[2]{1,0}; //0 cg , 1 trajectory design , 2  
                                                //mode[0] = 1のとき，mode[1] 0 なにもしない 1 軌道描画
    public static float near = 10; //選択判定のしきい値
    public static int pointMinRad = 3;
    public static int config = 0; //bitwise operation なるものをしてみようと思った
    public static int drawPoly = 500;
    //32bit 0:sync vtime with rtime 1:


    //基本変数
    public static double RSHT = 0;
    public static double VSHT = 0;
    public static double[,] positions = new double[10,3];
    public static double[,] vpositions = new double[10,3];
    public static double[] diffPos = new double[3];
    public static float[,] camAngl = new float[2,2]{{0,0},{0.785375F,0.785375F}};

    public static List<Godot.Vector2> points = new List<Godot.Vector2>();
    public static List<Orbit> orbits = new List<Orbit>();

    public static int selecting = 0;
    public static int viewCenter = 1;
    public static int gravCenter = 0;
    public static List<int> bSelecting = new List<int>();
    public static int hovering = -2;
    public static Quaternion posContQ;
    public static float posContT = -2; //-2は制御してないとき
    public static bool harfC = true; //重くて重要じゃないやつを実行するときのフラグ


    //database
    public static int[] mNissu = {31,28,31,30,31,30,31,31,30,31,30,31};//それぞれの月の日の数
    public static int URUUQTY2000 = 484; //2000年時てんのうるう年の数 2000年はうるう年だが含まない
    string[] celName = {"Sun","Mercury","Venus","Earth","Mars","Jupitor","Saturn","Neptune","Uranus","Pluto"};
    public class CelE{
        public static float[] radius = {696340F , 2439.7F , 6051.8F , 6378.0F , 3389.5F , 69911F , 58232F , 25362F, 24622F , 1188.3F}; //赤道半径 km
        public static double[] grav  =   {132712440000000,22032080,324858700,398600400,042828290,126712600000,37939520000,5780159000,6871308000,1020865};// km^2 m / s^2  重力定数
        public static Vector3[] v = new Vector3[10];
        public static Orbit[] o = new Orbit[10]{
            new Orbit(),
                //      period        e      Laxis       epo       s0     s1 s2 c1 c2 ci  s1~sc2は実行時に計算して埋める period,Laxis,epoなどは調整される この数値はいとうこうやの本にのってたもの
            new Orbit(2111.256     ,0.20563F,57909185  ,358.660F,77.462F ,0,0,0,0,1,1, 0, 0),
            new Orbit(5392.824     ,0.00678F,108208926 ,295.535F,131.564F,0,0,0,0,1,1, 0, 0),
            new Orbit(8765.812536  ,0.01672F,149597870 ,175.647F,102.950F,0,0,0,0,1,1, 0, 0),
            new Orbit(16487.52     ,0.09338F,227940928 ,328.285F,336.076F,0,0,0,0,1,1, 0, 0),
            new Orbit(103976.123686,0.04829F,778332734 ,126.078F,14.339F ,0,0,0,0,1,1, 0, 0),
            new Orbit(258873.378343,0.05604F,1426978478,359.712F,93.077F ,0,0,0,0,1,1, 0, 0),
            new Orbit(738546.091254,0.04612F,2870991216,156.021F,173.008F,0,0,0,0,1,1, 0, 0),
            new Orbit(1444518.24781,0.01011F,4497071892,263.861F,48.112F ,0,0,0,0,1,1, 0, 0),
            new Orbit(2171648.20414,0.24847F,5913523048,19.403F ,224.141F,0,0,0,0,1,1, 0, 0)
        };
    }
    public class Probe{　//探査機の構造体
        public static string name;
        public static float mass;
        public static float[,] MOI;
        public static Godot.Node3D node;
        public static Thruster[] thruster;
        public static double[] acce; //(m/ss)
        public static double[] acce2; //(m/ss)
        public static double[] velo; //km/s
        public static Rotate aAcce; ///ss
        public static Rotate aVelo; ///s
        public static Quaternion aVeloQ;
        public static Quaternion anglQ;
        public static float scal;
        public static float[] reactWheel; //リアクションホイールの回転速度rad/s xyz
        public static float solarEnergy; 
        public static float solarConf; //真上から距離10^8kmで太陽光があたったときどれくらい発電するか
    }


    //特殊変数
    public static Godot.RichTextLabel[] TEL = new Godot.RichTextLabel[2];
    public static Godot.Node3D[] CelN = new Godot.Node3D[10]; //node
    private Godot.Camera3D camN ;
    private Godot.Node3D lightN ;
    private Godot.Button sendBtn ;
    private Godot.TextEdit cmdBox ;
    public static Godot.ColorRect canvas ;
    public static Godot.SubViewport viewPort ;
    public static Godot.TabContainer tab;
    //public static Godot.Button[] btn = new Godot.Button[6];
    public static Godot.SubViewportContainer viewPortContainer;
    //process loopないでのみ使う
    public static Vector3 mvec = new Vector3(0,0,0);
    public static Godot.Vector3 gvec = new Godot.Vector3(0,0,0);
    public static Godot.Transform3D transform = Godot.Transform3D.Identity;
    public static Godot.LineEdit fovN;
    public static Godot.HSlider fovSlideN;
    public static Godot.LineEdit[] tsN = new Godot.LineEdit[2];
    public static Godot.HSlider[]  tsDigiN = new Godot.HSlider[2];
    public static Godot.HSlider[]  tsValueN = new Godot.HSlider[2];
    public static Godot.Button[]   tsSetN = new Godot.Button[2];
    public static Godot.Button[]   tsLockN = new Godot.Button[2];
    public static Godot.RichTextLabel[] tsValueMinN = new Godot.RichTextLabel[2];
    public static Godot.RichTextLabel[] tsValueMaxN = new Godot.RichTextLabel[2];   
    public static Godot.LineEdit[] sensitN = new Godot.LineEdit[2];
    public static Godot.LineEdit exposN;
    public static Godot.HSlider exposSlideN;
    public static Godot.WorldEnvironment worlEnvi;
    public static Godot.Button[] sync = new Godot.Button[4]; //checkbuttonをbuttonに格納してもうまくいくww
    public static Godot.LineEdit[] VSHTdisp = new Godot.LineEdit[5];
    public static Godot.Button setVtime ;

    public override void _Ready()
    {
        string viewportcontainer = "/root/Main/Control/HSC1/Tab/CG画面/Control/ViewportContainer";
        string viewport = viewportcontainer + "/Viewport";
        //string viewport = "./SubViewport";
        for(int i=0; i<9; i++){
            CelN[i] = (Godot.Node3D)GetNode($"{viewport}/{celName[i]}");
        }
        //nodeをget*/
        camN = (Godot.Camera3D)GetNode( viewport + "/Camera");
        lightN = (Godot.Node3D)GetNode( viewport +"/light");
        sendBtn = (Godot.Button)GetNode("/root/Main/Control/cmdPanel/sendBtn");
        cmdBox = (Godot.TextEdit)GetNode("/root/Main/Control/cmdPanel/cmdBox");
        TEL[0] = (Godot.RichTextLabel)GetNode("/root/Main/Control/HSC1/Tab/CG画面/Control/TEL");
        TEL[1] = (Godot.RichTextLabel)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/HSC2/HVC2/TELP/TEL");
        canvas = (Godot.ColorRect)GetNode( "/root/Main/Control/HSC1/Tab/軌道設計/HSC1/HSC2/HVC2/Canvas");
        tab = (Godot.TabContainer)GetNode( "/root/Main/Control/HSC1/Tab");
        viewPort = (Godot.SubViewport)GetNode(viewport);
        viewPortContainer = (Godot.SubViewportContainer)GetNode(viewportcontainer);
        //for文つかうとなぜかうまくいかない
        ((Godot.Button)GetNode($"/root/Main/Control/HSC1/Tab/軌道設計/toolBar/toolBar/b-1")).Pressed += ()=>ButtonClicked(-1); //for debug
        ((Godot.Button)GetNode($"/root/Main/Control/HSC1/Tab/軌道設計/toolBar/toolBar/b0")).Pressed += ()=>ButtonClicked(0);
        ((Godot.Button)GetNode($"/root/Main/Control/HSC1/Tab/軌道設計/toolBar/toolBar/b1")).Pressed += ()=>ButtonClicked(1);
        ((Godot.Button)GetNode($"/root/Main/Control/HSC1/Tab/軌道設計/toolBar/toolBar/b2")).Pressed += ()=>ButtonClicked(2);
        ((Godot.Button)GetNode($"/root/Main/Control/HSC1/Tab/軌道設計/toolBar/toolBar/b3")).Pressed += ()=>ButtonClicked(3);
        ((Godot.Button)GetNode($"/root/Main/Control/HSC1/Tab/軌道設計/toolBar/toolBar/b4")).Pressed += ()=>ButtonClicked(4);
        tsN[0] = (Godot.LineEdit)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/HFC/LineEdit");
        tsDigiN[0] = (Godot.HSlider)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/HBoxContainer6/digit");
        tsValueN[0] = (Godot.HSlider)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/value");
        tsSetN[0] = (Godot.Button)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/set");
        tsLockN[0] = (Godot.Button)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/reset");
        tsValueMinN[0] = (Godot.RichTextLabel)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/HBoxContainer3/RichTextLabel");
        tsValueMaxN[0] = (Godot.RichTextLabel)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/HBoxContainer3/RichTextLabel8");
        tsN[1] = (Godot.LineEdit)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/HFlowContainer/LineEdit");
        tsDigiN[1] = (Godot.HSlider)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/HBoxContainer/digit");
        tsValueN[1] = (Godot.HSlider)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/value");
        tsLockN[1] = (Godot.Button)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/reset");
        tsValueMinN[1] = (Godot.RichTextLabel)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/HBoxContainer3/RichTextLabel");
        tsValueMaxN[1] = (Godot.RichTextLabel)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/HBoxContainer3/RichTextLabel8");
        fovN = (Godot.LineEdit)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/HFlowContainer2/LineEdit");
        fovSlideN = (Godot.HSlider)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/HSlider");
        sensitN[0] = (Godot.LineEdit)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/HFC2/LineEdit");
        sensitN[1] = (Godot.LineEdit)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/HFC2/LineEdit");
        exposN = (Godot.LineEdit)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/HFlowContainer3/LineEdit");
        exposSlideN = (Godot.HSlider)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/HSlider2");
        worlEnvi = (Godot.WorldEnvironment)GetNode("/root/Main/Control/HSC1/Tab/CG画面/Control/ViewportContainer/Viewport/WorldEnvironment");
        sync[0] = (Godot.Button)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/Button"); //v with r [button]
        sync[1] = (Godot.Button)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/Button2"); //vts with r  [button]
        sync[2] = (Godot.Button)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/CheckButton");  //rts with vts
        sync[3] = (Godot.Button)GetNode("/root/Main/Control/HSC1/Tab/CG画面/TOOLP/HC/VC/HFC3/set2");  //r with v        [button]
        VSHTdisp[0] = (Godot.LineEdit)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/HFC/time/yyyy");
        VSHTdisp[1] = (Godot.LineEdit)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/HFC/time/mm");
        VSHTdisp[2] = (Godot.LineEdit)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/HFC/time/dd");
        VSHTdisp[3] = (Godot.LineEdit)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/HFC/time/hh");
        VSHTdisp[4] = (Godot.LineEdit)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/HFC/time/mm2");
        setVtime = (Godot.Button)GetNode("/root/Main/Control/HSC1/Tab/軌道設計/HSC1/TOOLP/HC/VC/HFC/set");
        //ui操作
        //tsN[0].Textが書きかわるのは、手入力,sync[2].Pressed、tsLockN[0].Pressed、tsValueNとtdDigiNのChanded
        //lock時に書きかえられないやうにす
        tsSetN[0].Pressed += ()=>{
            if(doCangeTsValue(MODE[0]))
                tsSet(MODE[0]);
        };
        tsLockN[0].Pressed += ()=>{
            inputToTs(MODE[0]);
        };
        tsLockN[1].Pressed += ()=>{
            inputToTs(MODE[0]);
        };
        sync[2].Pressed += ()=>{
            inputToTs(MODE[0]);
        };
        sync[1].Pressed += ()=>{
            if(sync[1].ButtonPressed){
                ts[1] = ts[0];
            }
        };
        sync[3].Pressed += ()=>{
            if(sync[3].ButtonPressed){
                if(RSHT <= VSHT){ //やったことは元にはもどらない。
                    RSHT = VSHT;
                }
            }
        };
        setVtime.Pressed += ()=>{
            VSHT = date2SHT(VSHTdisp[0].Text,VSHTdisp[1].Text,VSHTdisp[2].Text,VSHTdisp[3].Text,VSHTdisp[4].Text,"0");
        };
        sendBtn.Pressed += sendCmd;
        cmdBox.GrabFocus();
        canvas.Connect("mouse_entered", new Godot.Callable(this, nameof(OnMouseEntered)));
        canvas.Connect("mouse_exited", new Godot.Callable(this, nameof(OnMouseExited)));
        viewPortContainer.Connect("mouse_entered", new Godot.Callable(this, nameof(OnMouseEntered)));
        viewPortContainer.Connect("mouse_exited", new Godot.Callable(this, nameof(OnMouseExited)));
        OnViewportSizeChanged();
        viewPortContainer.Connect("resized", new Godot.Callable(this, nameof(OnViewportSizeChanged)));
        tsDigiN[0].Connect("value_changed",  new Godot.Callable(this, nameof(OnSliderValueChangedr)));
        tsValueN[0].Connect("value_changed", new Godot.Callable(this, nameof(OnSliderValueChangedr)));
        tsDigiN[1].Connect("value_changed",  new Godot.Callable(this, nameof(OnSliderValueChangedv)));
        tsValueN[1].Connect("value_changed", new Godot.Callable(this, nameof(OnSliderValueChangedv)));
        fovSlideN.Connect("value_changed",   new Godot.Callable(this, nameof(OnSliderValueChangedo)));
        exposSlideN.Connect("value_changed", new Godot.Callable(this, nameof(OnSliderValueChangedo)));
        //probe初期化
        Probe.node = (Godot.Node3D)GetNode(viewport + "/Fukashi");
        Probe.name = "ふかしFKS-A";
        Probe.mass = 250;//kg
        //慣性テンソル kg m m
        Probe.MOI = new float[3,3]{
            {106.7f, 0, 0},
            {0, 106.7f, 0},
            {0, 0, 143.1f}
        };
        float[,] tmpt = new float[3,7]{
            {0, 0, -1.3f,    0, 0,   1f,   200f},//thrusterの位置xyz(m),Fの方向xyz(正規化する必要なし),推力(N),
            {1f, 0, 0,       0, 1f,  0f,   10f },
            {1f, 0, 0,       0, -1f, 0f,   10f }
        };
        //probeの座標系 --> grobal座標系への第一段階( probeがstdRotateで0,0,0のときの座標系に変換 )
        Probe.MOI = new float[3,3]{
            {Probe.MOI[2,2] , -Probe.MOI[2,1] , Probe.MOI[2,0]},
            {-Probe.MOI[1,2] , Probe.MOI[1,1] , -Probe.MOI[1,0]},
            {Probe.MOI[0,2] , -Probe.MOI[0,1] , Probe.MOI[0,0]}
        };
        //probeの座標系 --> grobal座標系への第一段階( probeがstdRotateで0,0,0のときの座標系に変換 )
        for(int i=0; i<tmpt.GetLength(0); i++){
            Godot.GD.Print(i);
            float[] tmp = new float[7]{ tmpt[i,2], -tmpt[i,1], tmpt[i,0],   tmpt[i,5], -tmpt[i,4], tmpt[i,3],    tmpt[i,6]};
            tmpt[i,0] = tmp[0];
            tmpt[i,1] = tmp[1];
            tmpt[i,2] = tmp[2];
            tmpt[i,3] = tmp[3];
            tmpt[i,4] = tmp[4];
            tmpt[i,5] = tmp[5];
            tmpt[i,6] = tmp[6];
        }
        // cal and setup
        Probe.thruster = new Thruster[tmpt.GetLength(0)];//classの配列は2回newっする
        for(int i=0; i<Probe.thruster.Length; i++){
            Probe.thruster[i] = new Thruster();
            Vector3 R = new Vector3(tmpt[i,0],tmpt[i,1],tmpt[i,2]);
            Vector3 F = new Vector3(tmpt[i,3],tmpt[i,4],tmpt[i,5]);
            float d = F.Length();
            F.X = tmpt[i,3] * tmpt[i,6] / d;
            F.Y = tmpt[i,4] * tmpt[i,6] / d;
            F.Z = tmpt[i,5] * tmpt[i,6] / d;
            Vector3 T = Vector3.Cross( R, F );
            Probe.thruster[i].F  = F;
            Probe.thruster[i].T = T;//トルク
            Probe.thruster[i].rate = 0;
        }            
        Probe.reactWheel = new float[3]{0,0,0};
        Probe.solarConf = 100;
        //celEの初期化
        CelE.v[0] = new Vector3(0,0,0);
        float[,] tmps = new float[2,10]{
            {0F,7.004F ,3.394F  ,0F      ,1.850F  ,1.307F  ,2.486F  ,0.774F  ,1.771F  ,17.150F },
            {0F,48.327F,76.670F,174.865F ,49.548F ,100.472F,113.656F,74.009F ,131.782F,110.318F}
            };
        for(int i=1; i<10; i++){
            
            CelE.o[i].om = CelE.o[i].om - tmps[1,i];
            if(CelE.o[i].om < 0){
                CelE.o[i].om += 360;
            }else if(360 < CelE.o[i].om){
                CelE.o[i].om -= 360;
            }
            CelE.o[i].om = toRadF(CelE.o[i].om);
            CelE.o[i].c0 = MathF.Cos(CelE.o[i].om);
            CelE.o[i].s0 = MathF.Sin(CelE.o[i].om);
            CelE.o[i].s1 = MathF.Sin(toRadF(tmps[0,i]));
            CelE.o[i].s2 = MathF.Sin(toRadF(tmps[1,i]));
            CelE.o[i].c1 = MathF.Cos(toRadF(tmps[0,i]));
            CelE.o[i].c2 = MathF.Cos(toRadF(tmps[1,i]));

            //Godot.GD.Print($"{CelE.o[i].s0} , {CelE.o[i].s1} , {CelE.o[i].s2} , {CelE.o[i].c1} , {CelE.o[i].c2}");
            //伊藤弘也の軌道計算の薄い本のデータベースの値を変換している
            double GSHT = date2SHT("2003","7","1","0","0","0");
            GSHT = GSHT / (double)CelE.o[i].period;
            GSHT = GSHT % 1;
            GSHT = GSHT * 360;
            CelE.o[i].epo = CelE.o[i].epo - (float)GSHT;
            if(CelE.o[i].epo < 0){
                CelE.o[i].epo += 360;
            }else if(360 < CelE.o[i].epo){
                CelE.o[i].epo -= 360;
            }

            CelE.o[i].epo = toRadF(CelE.o[i].epo);
            Godot.GD.Print("aiuaiuau");
            Godot.GD.Print(CelE.o[i].epo);
            CelE.o[i].l = CelE.o[i].l / ((1+(double)CelE.o[i].e)*(1-(double)CelE.o[i].e)); //Laxis |-> l
        }
        //こまかい初期値
        string nowDate = Godot.Time.GetDatetimeStringFromSystem(true,false);
        orbits.Clear();
        orbits.Add(new Orbit());
        orbits.Add(new Orbit());
        Godot.GD.Print(orbits.Count);
        //2025-03-23T02:37:40 こんな形式
        RSHT = date2SHT(nowDate.Substring(0,4),nowDate.Substring(5,2),nowDate.Substring(8,2),nowDate.Substring(11,2),nowDate.Substring(14,2),nowDate.Substring(17,2));
        Godot.GD.Print(nowDate);
        VSHT = RSHT;
        calOrbiterPosition(3 , RSHT , false);
        Probe.acce = new double[3]{0, 0, 0};
        Probe.acce2 = new double[3]{0, 0, 0};
        Probe.velo = new double[3]{0, 0, 0};
        Probe.aAcce = new Rotate(1,0,0,0);
        Probe.aVelo = new Rotate(1,0,0,0);
        Probe.aVeloQ = new Quaternion(0,0,0,1);
        Probe.anglQ = new Quaternion(0,0,0,1);
        
        Inputs.scale[0] = 1;


        OnSliderValueChangedo(0);
        camN.Fov = (float)toDeg(FOV);
        
        //debugよう
        //ButtonClicked(-1);

        positions[0,0] = positions[3,0];
        positions[0,1] = positions[3,1];//静止軌道 高度+地球の半径
        positions[0,2] = positions[3,2] + 100000;
        Probe.velo[0] = -positions[3,1] * 32 / 149600000;
        Probe.velo[1] = positions[3,0] * 32 / 149600000;

        //positions[0,0] = 100000000;
        //positions[0,1] = 0;//静止軌道 高度+地球の半径
        //positions[0,2] = 0;
        //Probe.velo[0] = 0;
        //Probe.velo[1] = 30;
        //Probe.velo[2] = 0;

        //CelE.o[3].s0 = 0;
        //CelE.o[3].s1 = 0;
        //CelE.o[3].s2 = 0;
        //CelE.o[3].c0 = 1;
        //CelE.o[3].c1 = 1;
        //CelE.o[3].c2 = 1;
        
        //double AU = 149597870;
        /*
        for(int i=1;i<10;i++){
            CepoSHT[i] = (CepoSHT[i] /360) * (float)Cperiod[i];
            GD.Print(CepoSHT[i]);
            CepoSHT[i] = 30648 - CepoSHT[i];
            while(CepoSHT[i] > 0){
                CepoSHT[i] -= (float)Cperiod[i];
            }
            while(CepoSHT[i] < 0){
                CepoSHT[i] += (float)Cperiod[i];
            }
        }
        
        GD.Print(" ");
        for(int i=1;i<10;i++){
            GD.Print(CFepoSHT[i]);
        }*/
    }



    public override void _Input(Godot.InputEvent Event){
        if(Event is Godot.InputEventMouseButton mouseClickEvent){ //[InputEvent] is [InputEventMouseButton] [variable name] 方の一致不一致と型変換と代入と宣言を同時に行うscopeはたぶん_Inputのなか
            //click
            switch(mouseClickEvent.ButtonIndex){
                case Godot.MouseButton.Left:
                    if(Inputs.isMouseInside)
                        Inputs.MBL = mouseClickEvent.Pressed;
                    else
                        Inputs.MBL = false;
                    if( MODE[0] == 1  && Inputs.MBL ){ //mode1のとき，ホバーしててクリックするとselectingに追加
                        if(MODE[1] == 1){
                            if(bSelecting.Count == 2){
                                bSelecting.Clear();
                                MODE[1] = 0;
                            }
                            Godot.GD.Print("a");
                            //トグル選択j
                            if(bSelecting.Contains(hovering-1)){
                                for(int i=0; i<bSelecting.Count; i++){
                                    if(bSelecting[i] == hovering-1){
                                        bSelecting.RemoveAt(i);
                                        Godot.GD.Print("removeat "+i);
                                    }
                                }
                            }else{
                                //点を追加していかないといけないモード
                                bSelecting.Add(hovering-1);
                            }

                            //点操作以外の処理

                        } //トグル選択
                        
                        if( selecting == hovering ){
                            selecting = -2;
                        }else{
                            selecting = hovering;
                        }
                        
                    }
                    MODE[0] = tab.CurrentTab;
                    break;
                case Godot.MouseButton.Right:
                    if(Inputs.isMouseInside)
                        Inputs.MBR = mouseClickEvent.Pressed;
                    else
                        Inputs.MBR = false;
                    break;
                case Godot.MouseButton.Middle:
                    if(Inputs.isMouseInside)
                        Inputs.MBM = mouseClickEvent.Pressed;
                    else
                        Inputs.MBM = false;
                    break;
                case Godot.MouseButton.WheelUp:
                case Godot.MouseButton.WheelDown:
                    if(Inputs.isMouseInside){
                        if(MODE[0] <= 1){
                            if(mouseClickEvent.ButtonIndex == Godot.MouseButton.WheelUp)
                                Inputs.MWV[MODE[0]] += 1.5F*WHEEL_STEP;
                            else
                                Inputs.MWV[MODE[0]] -= 1.5F*WHEEL_STEP;
                            Inputs.scale[MODE[0]] = MathF.Pow(2 , Main.Inputs.MWV[MODE[0]]);
                            Inputs.scale[1] /= 1000000F;
                        }
                    }
                    break;
            }
        } 
        if(Event is Godot.InputEventMouseMotion mouseMotionEvent){
            //move
            if(Inputs.isMouseInside){
                if(Inputs.MBR){
                    //回転操作
                    if( MODE[0] <= 1){
                        if(MODE[0]==0){
                            camAngl[0, 0] -= sensit * 2 * FOV * (mouseMotionEvent.Position.X - Inputs.MMV.X)/viewPort.Size.X;
                        }else{
                            camAngl[MODE[0], 0] -= sensit * (mouseMotionEvent.Position.X - Inputs.MMV.X) *  0.002F;
                        }
                        if( camAngl[MODE[0], 0] <= -MathF.PI ){
                            camAngl[MODE[0], 0] += 2 * MathF.PI;
                        }else if( MathF.PI <= camAngl[MODE[0], 0] ){
                            camAngl[MODE[0], 0] -= 2 * MathF.PI;
                        }
                        //制限をかけるため 角度へらしてるときには-piの制限，ふやしてるときには+pi こうすれば制限外にでても復帰できる
                        if(mouseMotionEvent.Position.Y - Inputs.MMV.Y < 0){
                            if(-MathF.PI/2 < camAngl[MODE[0], 1])
                                if(MODE[0]==0){
                                    camAngl[0,1] += sensit * 2 * FOV * (mouseMotionEvent.Position.Y - Inputs.MMV.Y)/viewPort.Size.Y;
                                }else{
                                    camAngl[MODE[0], 1] += sensit * (mouseMotionEvent.Position.Y - Inputs.MMV.Y) *  0.002F;
                                }
                        }else{
                            if(MathF.PI/2 > camAngl[MODE[0], 1])
                                if(MODE[0]==0){
                                    camAngl[0,1] += sensit * 2 * FOV * (mouseMotionEvent.Position.Y - Inputs.MMV.Y)/viewPort.Size.Y;
                                }else{
                                    camAngl[MODE[0], 1] += sensit * (mouseMotionEvent.Position.Y - Inputs.MMV.Y)*  0.002F;
                                }
                        }
                        if(MODE[0] == 0){
                            //camNのTransformに適用
                            Godot.Transform3D trans = Godot.Transform3D.Identity;
                            trans.Origin = toGV(Vector3.Multiply(stdRotate(camAngl[0, 0], camAngl[0, 1]), CAMD) );
                            trans = trans.LookingAt(new Godot.Vector3(0,0,0) , new Godot.Vector3(0,0,1) , false);
                            camN.Transform = trans;
                        }
                    }
                }else if(Inputs.MBM){
                    if(MODE[0]==1){
                        Inputs.camDiff.X += (mouseMotionEvent.Position.X - Inputs.MMV.X)/(Main.Inputs.scale[1]*1000000);
                        Inputs.camDiff.Y += (mouseMotionEvent.Position.Y - Inputs.MMV.Y)/(Main.Inputs.scale[1]*1000000);
                    }
                }else if(Inputs.MBL){
                    if(MODE[0]==1){
                    }
                }
                if(MODE[0]==1){
                    //ドラックで時間変更
                    if(Inputs.MBL){
                        if(Inputs.Drag== -2 && hovering!=-2){
                            Inputs.Drag = hovering;
                        }
                    }else{
                        Inputs.Drag = -2;
                    }
                    //hoveringを更新
                    //しきい値中かついちばん近いpointを求める
                    float d = 99999;
                    float diffX;
                    float d2 = -1;
                    for(int i=0; i<points.Count; i++){
                        diffX = mouseMotionEvent.Position.X - points[i].X - canvas.GetGlobalPosition().X - Canvas.del.X;
                        if( diffX < 0 ){
                            diffX = -diffX;
                        }
                        if( diffX < near ){
                            float diffY;
                            diffY = mouseMotionEvent.Position.Y - points[i].Y - canvas.GetGlobalPosition().Y - Canvas.del.Y;
                            if( diffY < 0 ){
                                diffY = -diffY;
                            }
                            if( diffY < near ){
                                d2 = MathF.Pow(diffX, 2) + MathF.Pow(diffY, 2);
                                if( d2 < d ){
                                    d = d2;
                                    hovering = i;
                                }
                            }
                        }
                    }
                    if(d2 == -1){
                        hovering = -2;
                    }
                }
                
            Inputs.MMV = mouseMotionEvent.Position;
            }
        }
        if(Event is Godot.InputEventKey keyEvent){
            //key
            if(keyEvent.Pressed){
                //enter
                if(cmdBox.HasFocus() || tsN[0].HasFocus() || tsN[1].HasFocus() || fovN.HasFocus() || exposN.HasFocus()){
                    switch(keyEvent.Keycode){
                        case Godot.Key.Enter:
                            if(cmdBox.HasFocus()){
                                sendCmd();
                                cmdBox.AcceptEvent();
                            }
                            if(tsN[0].HasFocus()){
                                tsSet(0);
                            }
                            if(tsN[1].HasFocus()){
                                tsSet(1);
                            }
                            if(fovN.HasFocus()){
                                camN.Fov = float.Parse(fovN.Text);
                                FOV = toRadF(camN.Fov);
                            }
                            if(exposN.HasFocus()){

                            }
                            break;
                        case Godot.Key.Escape:
                            cmdBox.ReleaseFocus();
                            break;
                        default:
                            break;
                    }
                }else{
                    switch(keyEvent.Keycode){
                        case Godot.Key.Space:
                            cmdBox.GrabFocus();
                            cmdBox.AcceptEvent();
                            break;
                        case Godot.Key.Q:
                            if(MODE[0] <= 1)
                                tsDigiN[MODE[0]].Value = 1;
                            inputToTs(MODE[0]);
                            break;
                        case Godot.Key.W:
                            if(MODE[0] <= 1)
                                tsDigiN[MODE[0]].Value = 2;
                            inputToTs(MODE[0]);
                            break;
                        case Godot.Key.E:
                            if(MODE[0] <= 1)
                                tsDigiN[MODE[0]].Value = 3;
                            inputToTs(MODE[0]);
                            break;
                        case Godot.Key.R:
                            if(MODE[0] <= 1){
                                tsDigiN[MODE[0]].Value = 4;
                            }
                            inputToTs(MODE[0]);
                            break;
                        case Godot.Key.T:
                            if(MODE[0] <= 1)
                                tsDigiN[MODE[0]].Value = 5;
                            inputToTs(MODE[0]);
                            break;
                        case Godot.Key.Y:
                            if(MODE[0] <= 1)
                                tsDigiN[MODE[0]].Value = 6;
                            inputToTs(MODE[0]);
                            break;
                        case Godot.Key.U:
                            if(MODE[0] <= 1)
                                tsDigiN[MODE[0]].Value = 7;
                            inputToTs(MODE[0]);
                            break;
                        case Godot.Key.I:
                            if(MODE[0] <= 1)
                                tsDigiN[MODE[0]].Value = 8;
                            inputToTs(MODE[0]);
                            break;
                        case Godot.Key.P:
                            if(MODE[0] <= 1){
                                if(tsLockN[MODE[0]].ButtonPressed){
                                    tsLockN[MODE[0]].ButtonPressed = false;
                                }else{
                                    tsLockN[MODE[0]].ButtonPressed = true;
                                }
                            }
                            inputToTs(MODE[0]);
                            Godot.GD.Print($"Pause {MODE[0]}");
                            break;
                        case Godot.Key.Key2:
                            break;
                        case Godot.Key.Key3:
                            break;
                        case Godot.Key.Key4:
                            break;
                        case Godot.Key.Key5:
                            break;
                        case Godot.Key.Key6:
                            break;
                        case Godot.Key.Key7:
                            break;
                        case Godot.Key.Key8:
                            break;
                        case Godot.Key.Key9:
                            break;
                        case Godot.Key.Key0:
                            sync[2].ButtonPressed = true;
                            Godot.GD.Print("Pause");
                            break;
                        default:
                            Godot.GD.Print(keyEvent.Keycode);
                            break;
                    }
                if(MODE[0]==0){
                    if(float.TryParse(sensitN[0].Text, out sensit)){
                        sensitN[1].Text = sensitN[0].Text;
                    }
                }else{
                    if(float.TryParse(sensitN[1].Text, out sensit)){
                        sensitN[0].Text = sensitN[1].Text;
                    }
                }
                }
            }
        }
    }



    public void sendCmd(){
        cmdBox.Text = cmdBox.Text.Replace("/n","");
        Godot.GD.Print(cmdBox.Text);
        List<string> cmd = cmdBox.Text.Split(" ").ToList();
        List<string> opt = new List<string>();
        for(int i=0; i<cmd.Count; i++){
            Godot.GD.Print(cmd[i]);
        }
        for(int i=0; i<cmd.Count; i++){
            Godot.GD.Print(cmd[i]);
            if(cmd[i].Length > 1){
                if(cmd[i][0] == '-' && !(char.IsDigit(cmd[i][1]))){
                    Godot.GD.Print("opt : ",cmd[i]);
                    opt.Add(cmd[i]);
                    cmd.RemoveAt(i);
                }
            }else if(cmd[i] == ""){
                cmd.RemoveAt(i);
            }
        }
        cmdBox.Text = "";
        switch( cmd[0].ToLower() ){
            case "pos":
                switch(opt[0]){
                    case null:
                    case "--at":
                    case "-a":
                        Godot.GD.Print(cmd[3]);
                        float lamb = toRadF(float.Parse(cmd[1]));
                        float beta = toRadF(float.Parse(cmd[2]));
                        float sita = toRadF(float.Parse(cmd[3]));
                        posContT = toRadF(float.Parse(cmd[4]));
                        Quaternion direQ = System.Numerics.Quaternion.Concatenate( System.Numerics.Quaternion.CreateFromAxisAngle( new Vector3(1,0,0) , sita ) , System.Numerics.Quaternion.CreateFromAxisAngle( new Vector3(0,0,1) , lamb ) );
                        posContQ = System.Numerics.Quaternion.Concatenate( direQ , System.Numerics.Quaternion.CreateFromAxisAngle( new Vector3( MathF.Sin(lamb), -MathF.Cos(lamb), 0), beta ) );

                        break;
                    case "--keep":
                    case "-k":
                        posContT = -1;
                        posContQ = Probe.anglQ;
                        break;
                    case "--look":
                    case "-l":
                        int ci = Int32.Parse(cmd[1]);
                        break;
                    case "--end":
                    case "-e":
                        break;
                }
                break;
            case "thr":
                int index = Int32.Parse(cmd[1]);
                float power;
                switch(opt[0]){
                    case null:
                         power = float.Parse(cmd[2]);
                        float time = float.Parse(cmd[3]);
                        break;
                    case "--start":
                    case "-s":
                         power = float.Parse(cmd[2]);
                            if( Probe.thruster[index].F.Length() <= power ){
                                Probe.thruster[index].rate = 1f;
                            }else{
                                Probe.thruster[index].rate = power / Probe.thruster[index].F.Length();
                            }
                        break;
                    case "--end":
                    case "-e":
                        Probe.thruster[index].rate = 0;
                        break;
                }
                break;
            // ここから下はデバッグよう
            case "set":
                switch(cmd[1]){
                    case "timescale":
                        ts[0] = double.Parse(cmd[2]);
                        break;
                    case "fov":
                        FOV = (float)toRad(double.Parse(cmd[2]));
                        camN.Fov = float.Parse(cmd[2]);
                        break;
                    default:
                        Godot.GD.Print(cmd[1].ToLower());
                        break;
                }
                break;
            case "rot":
                switch(opt[0]){
                    case "-at":
                        float lamb = (float)toRad(double.Parse(cmd[1]));
                        float beta = (float)toRad(double.Parse(cmd[2]));
                        float sita = (float)toRad(double.Parse(cmd[3]));
                        Quaternion direQ = System.Numerics.Quaternion.Concatenate( System.Numerics.Quaternion.CreateFromAxisAngle( new Vector3(1,0,0) , sita ) , System.Numerics.Quaternion.CreateFromAxisAngle( new Vector3(0,0,1) , lamb ) );
                        Probe.anglQ = System.Numerics.Quaternion.Concatenate( direQ , System.Numerics.Quaternion.CreateFromAxisAngle( new Vector3( MathF.Sin(lamb), -MathF.Cos(lamb), 0), beta ) );
                        Godot.GD.Print(lamb);
                        Godot.GD.Print(" ");
                        Godot.GD.Print(beta);
                        Godot.GD.Print("\n");
                        break;
                }
                break;
            default:
                Godot.GD.Print(cmd[0].ToLower());
                break;
        }
    } 

    public void ButtonClicked(in int i){
        Godot.GD.Print(i);
        switch(i){
            case -1:
                //canvas.QueueRedraw();
                orbits.Clear();
                Main.orbits.Add(Main.solveOrbit(
                    new Vector3((float)Main.Probe.velo[0],(float)Main.Probe.velo[1],(float)Main.Probe.velo[2]) , 
                    0, 
                    new Vector3((float)Main.positions[0,0],(float)Main.positions[0,1],(float)Main.positions[0,2]),
                    Math.Sqrt(Math.Pow(Main.Probe.velo[0],2)+Math.Pow(Main.Probe.velo[1],2)+Math.Pow(Main.Probe.velo[0],2)),
                    Math.Sqrt(Math.Pow(Main.positions[0,0],2)+Math.Pow(Main.positions[0,1],2)+Math.Pow(Main.positions[0,2],2))));
                Godot.GD.Print(" ");
                Godot.GD.Print(orbits[0].l);
                Godot.GD.Print(" ");
                break;

            case 0:
                //viewCenterを選択に
                if(selecting != -2)
                    viewCenter = selecting;
                Inputs.camDiff = Main.canvas.Size/2;
                break;
            case 1:
                Inputs.camDiff = Main.canvas.Size/2;
                camAngl[0,0] = 0.785375F;
                camAngl[0,1] = 0.785375F;
                break;
            case 2:
                //gravCenterを選択に
                if(selecting != -2 && selecting != 0)
                    gravCenter = selecting-1;
                break;
            case 4:
                //probeの現在の軌道を追加
                if(MODE[1]==1){
                    MODE[1] = 0;
                }else{
                    MODE[1] = 1;
                }
                break;
        }
    }
    // txt |-> value
    private bool doCangeTsValue(int mode){
        bool doCal = true;
        //sync[2],tsLockN[0],tsLockN[1]
        if(tsLockN[1].ButtonPressed){ //vtsLock
            ts[1] = 0;
            tsN[1].Text = "0にロック中";
            tsN[1].Editable = false;
            if(mode==1)
                doCal = false;
        }else{
            tsN[1].Editable = true;
        }
        if(sync[2].ButtonPressed){ //rts with vts
            if(ts[1] == 0){
                ts[0] = 1;
                tsN[0].Text = "1";
            }else{
                ts[0] = ts[1];
                tsN[0].Text = tsN[1].Text;
            }
            tsN[0].Editable = false;
            if(mode==0)
                doCal = false;
        }else{
            tsN[0].Editable = true;
        }
        if(tsLockN[0].ButtonPressed){ //rtsLock lockのほうが優先さるべき
            ts[0] = 1;
            tsN[0].Text = "1にロック中";
            tsN[0].Editable = false;
            if(mode==0)
                doCal = false;
        }else{
            tsN[0].Editable = true;
        }
        return doCal;
    }
    private void inputToTs(int mode){
        if(mode==0){
            if(doCangeTsValue(0)){
                OnSliderValueChangedr(0);
                Godot.GD.Print(tsN[mode].Text);
                tsSet(0);
            }
        }
        if(mode==1){
            if(doCangeTsValue(1)){
                OnSliderValueChangedv(0);
                Godot.GD.Print(tsN[mode].Text);
            }
        }
    }
    private void tsSet(int mode){
        ts[mode] = Double.Parse(tsN[mode].Text);
        double tmp = ts[mode];
        tsDigiN[mode].Value = Math.Floor(Math.Log10(ts[mode]))+1;
        //OnSliderValueChandedが発火し，ts[0]が起きかえらる
        if(mode == 1 && tsDigiN[1].Value == 1){
            tsValueN[1].Value = tmp/9;
        }else{
            float min = MathF.Pow(10, (float)tsDigiN[mode].Value - 1);
            float max = 9 * (min + min/10);
            tsValueN[mode].Value = (double)(tmp-min)/(max-min);
        }
        
    }
    private void OnMouseEntered(){
        Inputs.isMouseInside = true;
    }
    private void OnMouseExited(){
        Inputs.isMouseInside = false;
    }
    private void OnViewportSizeChanged(){
        viewPort.Size = (Godot.Vector2I)viewPortContainer.GetRect().Size;
        Inputs.camDiff = Main.canvas.Size/2;
    }
    //slider
    public void OnSliderValueChangedr(float value){
        int digi = Convert.ToInt32(tsDigiN[0].Value);
        Godot.GD.Print($"sliderchanged : {digi}");
        tsValueMinN[0].Text = "1" + new String('0' , digi - 1);
        tsValueMaxN[0].Text = new String('9' , digi);
        tsN[0].Text = (Math.Floor(10 * (Double.Parse(tsValueMinN[0].Text) + tsValueN[0].Value * (Double.Parse(tsValueMaxN[0].Text) - Double.Parse(tsValueMinN[0].Text))))/10).ToString();
    }
    public void OnSliderValueChangedv(float value){
        int digi = Convert.ToInt32(tsDigiN[1].Value);
        if(digi == 1){
            tsValueMinN[1].Text = "0";
            tsValueMaxN[1].Text = "9";
            ts[1] = Math.Floor(90 * tsValueN[1].Value)/10;
        }else{
            tsValueMinN[1].Text = "1" + new String('0' , digi - 1);
            tsValueMaxN[1].Text = new String('9' , digi);
            ts[1] = Math.Floor(10 * (Double.Parse(tsValueMinN[1].Text) + tsValueN[1].Value * (Double.Parse(tsValueMaxN[1].Text) - Double.Parse(tsValueMinN[1].Text))))/10;
        }
        if(sync[2].ButtonPressed){
            if(ts[1] == 0 || tsLockN[1].ButtonPressed){
                ts[0] = 1;
            }else{
                ts[0] = ts[1];
            }
        }
        if(doCangeTsValue(1)){
            tsN[1].Text = ts[1].ToString();
        }
    }
    public void OnSliderValueChangedo(float value){

        camN.Fov = (float)(17378.9/(fovSlideN.Value + 60.49) - 107.28);
        fovN.Text = (Math.Floor(10*camN.Fov)/10).ToString();
        FOV = toRadF(camN.Fov);

        exposN.Text = exposSlideN.Value.ToString();
        // y = (x-1+n)/nm x=1のときのyを1/m かたむきを1/nmにする
        worlEnvi.Environment.AdjustmentBrightness = ((float)exposSlideN.Value -1 + (3))/ (3*5);

    }


    public override void _Process(double deltaT)
    {       
        VSHT += deltaT * ts[1] / 3600;
        deltaT = deltaT * ts[0]; //s
        lightN.Transform = CelN[0].Transform;

            // km/s
        int sm = (int)ts[0]/131072;
        if(sm==0)
            sm=1;
        rFS = new Vector3(0,0,0); //rF合計
        for(int s=0;s<sm;s++){
            calProbePosition(in s, deltaT/sm);
        }
        if(MODE[0]==1){
        }
        //表示系{
        if(harfC){
            if(MODE[0]==1){
                canvas.QueueRedraw();
                string vutc = SHT2date(VSHT);
                VSHTdisp[0].Text = vutc.Substring(0,4);
                VSHTdisp[1].Text = vutc.Substring(4,2);
                VSHTdisp[2].Text = vutc.Substring(7,2);
                VSHTdisp[3].Text = vutc.Substring(9,2);
                VSHTdisp[4].Text = vutc.Substring(11,2);
            }
            TEL[MODE[0]].Text = displayTEL(0);
            transform = Godot.Transform3D.Identity;
            transform.Basis.X = toGV(Vector3.Transform( new Vector3( 0 , 0 , Inputs.scale[0])  , Probe.anglQ ));
            transform.Basis.Y = toGV(Vector3.Transform( new Vector3( 0 , -Inputs.scale[0] , 0) , Probe.anglQ ));
            transform.Basis.Z = toGV(Vector3.Transform( new Vector3( Inputs.scale[0] , 0 , 0)  , Probe.anglQ ));
            Probe.node.Transform = transform;
        }
        //}
        RSHT += deltaT / 3600; //2000標準時
        if(sync[0].ButtonPressed)
            VSHT = RSHT;

        harfC = !harfC;

        //以降debugよう
        
    }
    
}
