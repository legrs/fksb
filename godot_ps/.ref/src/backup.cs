// 制御うまくいってたときの
using System;
using System.Numerics;
using G = Godot;
using GD = Godot.GD;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using static Draw;

// templates
// GD.Print($"ori : {ori.toString()}");

public partial class Main : G.Control {

        // control parameters
    public static float Kp = 0.1F; 
    public static float Kd = 10.0F;


    public struct Input{
        // 0:cg 1:design
        public static bool MBL = false;//mouse light button
        public static bool MBR = false;// \\   right
        public static bool MBM = false;// \\   middle
        public static float[] MWV = new float[2]{0,0}; //wheel
        public static Godot.Vector2[] MMV = new G.Vector2[2]; //mouse
        public static Godot.Vector2 camDiff = new Godot.Vector2(242,145);
        public static float[] scale = new float[2]{1,0.000001F};
        public static bool isMouseInside = false;
        public static bool isDragging = false;
    }
    float FOV = (float)toRad(50); //rad
    public static float[] camAngl = new float[2]{0,0};
    float sensit = 3F;
    public static float camd = 800F;

        // physics constants
    static float cd = 60; //mm
    static float ch = 85;
    static float cm = 300; //g
    Vec3 r = new Vec3(0,0,-ch/2); // vector from forcePoint to centerOfGravity [local]
    //Matrix moi = Matrix.Build.DenseOfArray(new double[,]{
    //        {10,0,0},
    //        {0,10,0},
    //        {0,0,10}
    //        });

        // moment of inertia of cansat (primary 
    Vec3 cmoi = new Vec3(cm*cd*cd/16 + cm*ch*ch/3 , cm*cd*cd/16 + cm*ch*ch/3 , cm*cd*cd/8);
        // reaction wheels
    static float wd1 = 20; //mm smaller diameter ( hole )
    static float wd2 = 40; //mm
    static float wm = 30; //g
    static float wmoi = wm*(wd2*wd2 - wd1*wd1)/8;

    public static G.Vector2[] rpos = new G.Vector2[2]; // position on rect
    public static Vec3 pos = new Vec3(); // global
    public static Quat ori = new Quat(new Vec3(1,0,1) , 0.0F); // global

    public static Vec3 vel = new Vec3(); // global
    public static Vec3 avel = new Vec3(); // global

    double time = 0;
    bool f2 = true;
    bool doControl = false;

    Vec3 force = new Vec3(0.00F,0.0F,0.0F); // global
    Vec3 impulse = new Vec3(0.00F,0.0F,0.00F); // global
    Vec3 torque = new Vec3(0,0,0); // local !!
    //Vec3 torqueG = new Vec3(0,0,0); // local !!
    // x z y

    Vec3 deltaV =  new Vec3();
    G.Vector2 v2 =  new G.Vector2();
    G.Quaternion quat = new G.Quaternion();
    G.Transform3D trans = G.Transform3D.Identity;
    //node
    G.Node3D cansat;
    public static G.SubViewportContainer svc;
    G.Camera3D cam;
    G.ColorRect cr;
    G.RigidBody3D rb;
    G.StaticBody3D cp;
    G.MeshInstance3D vo;

    public static float scale = 1;
    // Called when the node enters the scene tree for the first time.
    public override void _Ready(){
        rb = (G.RigidBody3D)GetNode(".//svc/sv/Node3D/rb");
        cp = (G.StaticBody3D)GetNode(".//svc/sv/Node3D/cp");
        cansat = (G.Node3D)GetNode("./svc/sv/Node3D/rb/cansat");
        cam = (G.Camera3D)GetNode("./svc/sv/Node3D/cam");
        cr = (G.ColorRect)GetNode("./cr");
        vo = (G.MeshInstance3D)GetNode("./svc/sv/Node3D/visualObj");

        
        G.Transform3D transs = G.Transform3D.Identity;
        transs.Origin = new G.Vector3(camd,0,0);
        transs.Basis.X = new G.Vector3(0,1,0);
        transs.Basis.Y = new G.Vector3(0,0,1);
        transs.Basis.Z = new G.Vector3(1,0,0);
        cam.Transform = transs;

        transs.Origin = new G.Vector3(0,0,0);
        transs.Basis.X = (new Vec3(1,0,0).rotate(ori)).toGV();
        transs.Basis.Y = (new Vec3(0,1,0).rotate(ori)).toGV();
        transs.Basis.Z = (new Vec3(0,0,1).rotate(ori)).toGV();
        rb.Transform = transs;

        svc = (G.SubViewportContainer)GetNode("./svc");
        svc.Connect("mouse_entered", new Godot.Callable(this, nameof(OnMouseEntered)));
        svc.Connect("mouse_exited", new Godot.Callable(this, nameof(OnMouseExited)));


        rb.Mass = cm;
        rb.Inertia = cmoi.toGV();
        rb.CenterOfMass = r.toGV();

        GD.Print($"ori {ori.toString()}");
        GD.Print($"{(double)ori.x/Math.Sqrt(1-Math.Pow(ori.w, 2)) * Math.Acos(ori.w) * 2}");
    }

    public override void _Input(Godot.InputEvent Event){
        if(Event is Godot.InputEventMouseButton mouseClickEvent){ //[InputEvent] is [InputEventMouseButton] [variable name] 方の一致不一致と型変換と代入と宣言を同時に行うscopeはたぶん_Inputのなか
            //click
            switch(mouseClickEvent.ButtonIndex){
                case Godot.MouseButton.Left:
                    Input.MBL = mouseClickEvent.Pressed;
                    break;
                case Godot.MouseButton.Right:
                    Input.MBR = mouseClickEvent.Pressed;
                    break;
                case Godot.MouseButton.Middle:
                    Input.MBM = mouseClickEvent.Pressed;
                    break;
                case Godot.MouseButton.WheelUp:
                case Godot.MouseButton.WheelDown:
                    break;
            }
        } 
        if(Event is Godot.InputEventMouseMotion mouseMotionEvent){
            //move
            //if(Input.isMouseInside){
            Input.MMV[0] = mouseMotionEvent.Position;
                if(Input.MBR){
                    //回転操作
                            camAngl[0] -= sensit * 2 * FOV * (Input.MMV[0].X - Input.MMV[1].X)/svc.Size.X;
                        if(camAngl[0] <= -MathF.PI){
                            camAngl[0] += 2 * MathF.PI;
                        }else if( MathF.PI <= camAngl[0] ){
                            camAngl[0] -= 2 * MathF.PI;
                        }
                        //制限をかけるため 角度へらしてるときには-piの制限，ふやしてるときには+pi こうすれば制限外にでても復帰できる
                        if(Input.MMV[0].Y - Input.MMV[1].Y < 0){
                            if(-MathF.PI/2 < camAngl[1])
                                    camAngl[1] += sensit * 2 * FOV * (Input.MMV[0].Y - Input.MMV[1].Y)/svc.Size.Y;
                        }else{
                            if(MathF.PI/2 > camAngl[1])
                                    camAngl[1] += sensit * 2 * FOV * (Input.MMV[0].Y - Input.MMV[1].Y)/svc.Size.Y;
                        }
                            //camのTransformに適用
                            G.Transform3D transs = G.Transform3D.Identity;

                            GD.Print($"camangl ${camAngl[0]}, ${camAngl[1]}");

                            //camAngl[0] = (float)time * 10;
                            Quat q = new Quat(new Vec3(0,0,1), camAngl[0]);

                            q = Quat.concat(q,new Quat(new Vec3(0,-1,0) , camAngl[1]));
                            
                            transs.Origin = (new Vec3(camd,0,0).rotate(q)).toGV();

                            GD.Print($"origin {transs.Origin.X},{transs.Origin.Y},{transs.Origin.Z},");

                            transs.Basis.X = (new Vec3(0,1,0).rotate(q)).toGV();
                            transs.Basis.Y = (new Vec3(0,0,1).rotate(q)).toGV();
                            transs.Basis.Z = (new Vec3(1,0,0).rotate(q)).toGV();


                            cam.Transform = transs;
                }else if(Input.MBM){
                }
                if(Input.MBL){
                    // dragging
                    if((Input.MMV[0] - ppos - cr.Position - cr.Size/2).Length() < 10){
                        Input.isDragging = true;
                    }
                }else{
                    Input.isDragging = false;
                }
                
            Input.MMV[1] = Input.MMV[0];
            //}
        }
        if(Event is Godot.InputEventKey keyEvent){
            //key
            if(keyEvent.Pressed){
                //enter
                /*
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
                */
                    switch(keyEvent.Keycode){
                        case Godot.Key.Space:
                            break;
                        case Godot.Key.S:
                            doControl = true;
                            break;
                        case Godot.Key.Q:
                            doControl = false;
                            break;
                        case Godot.Key.Key1:
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
                            break;
                        default:
                            break;
                    }
            }
        }
    }
    private void OnMouseEntered(){
        Input.isMouseInside = true;
    }
    private void OnMouseExited(){
        Input.isMouseInside = false;
    }
    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta) {
        time += delta;

            // instance → data  update
        ori.setFromGQ(new G.Quaternion(rb.Transform.Basis));
            //GD.Print($"ori {ori.toString()}");
        GD.Print(rb.AngularVelocity.X);
        avel.setFromGV(rb.AngularVelocity);
            GD.Print($"avel(global) : {avel.toString()}");
        avel = avel.rotate(ori.invert());
            //GD.Print($"avel(local) : {(new Vec3(1000,0,0).rotate(ori.invert())).toString()}");

        //rb.ApplyTorque(new G.Vector3(0,0,1000000));


        /*
        double deltaT = 1;
        double startTime = 1;
        if(startTime < time && time < startTime+deltaT){
            doControl = true;

        }else if(startTime+deltaT < time && time < startTime+2*deltaT){
            doControl = false;
        }else{
            doControl = false;
        }
        */

        if(doControl){
            majiKeisandekiru3(delta);
        }else{
            torque.x = 0;
            torque.y = 0;
            torque.z = 0;
        }


            // local to global   apply torque(change energy of reaction wheel)
        rb.ApplyTorque(torque.rotate(ori).toGV());

        




            // data → instance  dragging
        if(Input.isDragging){
            trans.Origin.X = camd*(Input.MMV[0].X - cr.Position.X - cr.Size.X/2)/cr.Size.X;
            trans.Origin.Y = camd*(-Input.MMV[0].Y + cr.Position.Y + cr.Size.Y/2)/cr.Size.Y;
            cp.Transform = trans;
        }
        //trans.Origin.Y = -camd + (float)time*1000;
            //cp.Transform = trans;

        pos.setFromGV(cp.Transform.Origin);


        /*


        // 1/2 で実行
        if(f2){
        }

        // for test
        //avel.multiply((1-ATTE*delta));

        //impulse = Vec3.multiply(force, delta);

        // Fdt = dp = mdv mdx/dt
        //Vec3 acc = Vec3.multiply(force , delta/m);
        //vel.add(acc);
        //vel.add(Vec3.multiply(impulse , 1/m));
        vel.add(Vec3.multiply(force , delta/m));
        pos.add(Vec3.multiply(vel, delta));

        double tdelta = delta;
        Vec3 aimpulse = Vec3.multiply(torque , tdelta);
        Vec3 tv = Vec3.add( force.invert() , new Vec3(0,0,GRAV*m ));
        for(int i=0; i<STEP; i++){

            // eular method  
            
            // torque * dt = r×Fdt
            //aimpulse = Vec3.add(aimpulse,Vec3.cross( r , tv.rotate(ori.invert())));
            aimpulse = Vec3.cross( r , tv.rotate(ori.invert()));

            //GD.Print($"torque : {torquetmp.toString()}");
                // calculate second derivative
                // coordinate G to L
            Vec3 avelL = avel.rotate(ori.invert());

            //GD.Print($"avelL : {avelL.toString()}");
                // from "eular's equation of motion" ( at primary moment axis coordinate )
            Vec3 aavel = new Vec3();
            aavel.x = (aimpulse.x + (pmoi.y - pmoi.z)*avelL.y*avelL.z) / pmoi.x ; 
            aavel.y = (aimpulse.y + (pmoi.z - pmoi.x)*avelL.z*avelL.x) / pmoi.y ; 
            aavel.z = (aimpulse.z + (pmoi.x - pmoi.y)*avelL.x*avelL.y) / pmoi.z ; 

            //GD.Print($"aavel : {aavel.toString()}");

                // coordinate trans. L2G
            aavel = aavel.rotate(ori);
            //aavel.multiply(tdelta);

                //
                // cal. first derivative
                //
            avel.add(Vec3.multiply(aavel,tdelta));
            //GD.Print($"avel : {avel.toString()}");

            //GD.Print($"pos : {pos.toString()}");


            //avel = new Vec3(0.05,0.05,0.00);



                // cal. zero d.
            ori = Quat.concat(new Quat(avel , avel.abs()*tdelta), ori);
            //GD.Print($"ori : {ori.toString()}");
            //GD.Print($"pos : {pos.toString()}");
            //GD.Print($"quat : {ori.toString()}");
            //GD.Print($"|quat| : {Math.Sqrt(Math.Pow(ori.w,2)+Math.Pow(ori.x,2)+Math.Pow(ori.y,2)+Math.Pow(ori.z,2))}");
        }

        //pos.x = (double)rpos[0].X*camd/(double)cr.Size.X;
        //pos.y = (double)rpos[0].Y*camd/(double)cr.Size.Y;

            // substitude 
        trans.Origin = pos.toGV();
        trans.Basis.X = (new Vec3(1,0,0).rotate(ori)).toGV();
        trans.Basis.Y = (new Vec3(0,1,0).rotate(ori)).toGV();
        trans.Basis.Z = (new Vec3(0,0,1).rotate(ori)).toGV();


        //trans.Origin = new G.Vector3(0,(float)time*100,0);
        //trans.Basis.X = new G.Vector3(1,0,0);
        //trans.Basis.Y = new G.Vector3(0,1,0);
        //trans.Basis.Z = new G.Vector3(0,0,1);

        //GD.Print($"basis x : {(new Vec3(1,0,0).rotate(ori)).toString()}");
        //GD.Print($"basis y : {(new Vec3(0,1,0).rotate(ori)).toString()}");
        //GD.Print($"basis z : {(new Vec3(0,0,1).rotate(ori)).toString()}");


        cansat.Transform = trans;
        //GD.Print($"{pos.x},{pos.y},{pos.z}");

        f2 = !f2;
        */
    }
    private void majiKeisandekiru3(double delta){
        // bno can export  acceleration , orientation , gyro(= anglar velocity)


        //G.Transform3D transs = vo.Transform;
        //transs.Basis = new G.Basis(Quat.concat(new Quat(new Vec3(0,1,0) , tmp) , ori).toGQ());
        //vo.Transform = transs;


        GD.Print($"avel : {avel.toString()}");
        double sin = (Math.Sqrt(1-Math.Pow(ori.w, 2)));
        deltaV = Vec3.multiply(avel.invert() , Kp/2);
        if( sin == 0){

        }else{
            float fac =  Kd * -(float)(2*Math.Acos(ori.w) / sin );
            //tmp = (double)avel.x + (double)ori.x/sin * Math.Acos(ori.w) * 2/0.1;
            deltaV.x += ori.x*fac;
            deltaV.y += ori.y*fac;
            deltaV.z += ori.z*fac;


            avelL.Add(avel.x);
            anglL.Add(-ori.x*(float)fac/Kd);
            deltavL.Add(deltaV.x);
            if( 100 < avelL.Count ){
                avelL.RemoveAt(0);
                anglL.RemoveAt(0);
                deltavL.RemoveAt(0);
            }
        }

        // deltaV to torque
        // Eular's equation of motion
        GD.Print($"ori : {ori.toString()}");
        GD.Print($"deltaV : {deltaV.toString()}");

        torque.x = cmoi.x*deltaV.x/(float)delta + (cmoi.z - cmoi.y)*avel.z*avel.y;
        torque.y = cmoi.y*deltaV.y/(float)delta + (cmoi.x - cmoi.z)*avel.x*avel.z;
        torque.z = cmoi.z*deltaV.z/(float)delta + (cmoi.y - cmoi.x)*avel.y*avel.x;
        GD.Print($"torque : {torque.toString()}");

        GD.Print("\n");
    }
}

// いまのやつ
using System;
using System.Numerics;
using G = Godot;
using GD = Godot.GD;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using static Draw;

// templates
// GD.Print($"ori : {ori.toString()}");

public partial class Main : G.Control {

        // control parameters
    public static float Kp = 1F; 
    public static float Kd = 10F;


    public struct Input{
        // 0:cg 1:design
        public static bool MBL = false;//mouse light button
        public static bool MBR = false;// \\   right
        public static bool MBM = false;// \\   middle
        public static float[] MWV = new float[2]{0,0}; //wheel
        public static Godot.Vector2[] MMV = new G.Vector2[2]; //mouse
        public static Godot.Vector2 camDiff = new Godot.Vector2(242,145);
        public static float[] scale = new float[2]{1,0.000001F};
        public static bool isMouseInside = false;
        public static bool isDragging = false;
    }
    float FOV = (float)toRad(50); //rad
    public static float[] camAngl = new float[2]{0,0};
    float sensit = 3F;
    public static float camd = 800F;

        // physics constants
    static float cd = 60; //mm
    static float ch = 85;
    static float cm = 300; //g
    Vec3 r = new Vec3(0,0,-ch/2); // vector from forcePoint to centerOfGravity [local]
    //Matrix moi = Matrix.Build.DenseOfArray(new double[,]{
    //        {10,0,0},
    //        {0,10,0},
    //        {0,0,10}
    //        });

        // moment of inertia of cansat (primary 
    Vec3 cmoi = new Vec3(cm*cd*cd/16 + cm*ch*ch/3 , cm*cd*cd/16 + cm*ch*ch/3 , cm*cd*cd/8);
        // reaction wheels
    static float wd1 = 20; //mm smaller diameter ( hole )
    static float wd2 = 40; //mm
    static float wm = 30; //g
    static float wmoi = wm*(wd2*wd2 - wd1*wd1)/8;
    static float maxTorque = 100000;

    // 0:current 1:previous
    public static G.Vector2[] rpos = new G.Vector2[2]; // position on rect
    public static Vec3 pos = new Vec3(); // global
    public static Quat[] ori = new Quat[2]; // global
    public static Vec3[] oriv = new Vec3[2];

    public static Vec3 vel = new Vec3(); // global
    public static Vec3 avel = new Vec3(); // global

    double time = 0;
    bool f2 = true;
    bool doControl = false;

    Vec3 force = new Vec3(0.00F,0.0F,0.0F); // global
    Vec3 impulse = new Vec3(0.00F,0.0F,0.00F); // global
    Vec3 torque = new Vec3(0,0,0); // local !!
    //Vec3 torqueG = new Vec3(0,0,0); // local !!
    // x z y

    Vec3 deltaV =  new Vec3();
    G.Vector2 v2 =  new G.Vector2();
    G.Quaternion quat = new G.Quaternion();
    G.Transform3D trans = G.Transform3D.Identity;
    //node
    G.Node3D cansat;
    public static G.SubViewportContainer svc;
    G.Camera3D cam;
    G.ColorRect cr;
    G.RigidBody3D rb;
    G.AnimatableBody3D ab;
    G.MeshInstance3D vo;

    public static float scale = 1;
    // Called when the node enters the scene tree for the first time.
    public override void _Ready(){
        // get nodes
        rb = (G.RigidBody3D)GetNode(".//svc/sv/Node3D/rb");
        ab = (G.AnimatableBody3D)GetNode(".//svc/sv/Node3D/ab");
        cansat = (G.Node3D)GetNode("./svc/sv/Node3D/rb/cansat");
        cam = (G.Camera3D)GetNode("./svc/sv/Node3D/cam");
        cr = (G.ColorRect)GetNode("./cr");
        vo = (G.MeshInstance3D)GetNode("./svc/sv/Node3D/visualObj");

        // setup
        ori[0] = new Quat(new Vec3(1,0,0) , 0.0F);
        ori[1] = new Quat(new Vec3(1,0,0) , 0.0F);
        oriv[1] = new Vec3();
        oriv[1] = new Vec3();

        
        G.Transform3D transs = G.Transform3D.Identity;
        transs.Origin = new G.Vector3(camd,0,0);
        transs.Basis.X = new G.Vector3(0,1,0);
        transs.Basis.Y = new G.Vector3(0,0,1);
        transs.Basis.Z = new G.Vector3(1,0,0);
        cam.Transform = transs;

        transs.Origin = new G.Vector3(0,0,0);
        transs.Basis.X = (new Vec3(1,0,0).rotate(ori[0])).toGV();
        transs.Basis.Y = (new Vec3(0,1,0).rotate(ori[0])).toGV();
        transs.Basis.Z = (new Vec3(0,0,1).rotate(ori[0])).toGV();
        rb.Transform = transs;

        svc = (G.SubViewportContainer)GetNode("./svc");
        svc.Connect("mouse_entered", new Godot.Callable(this, nameof(OnMouseEntered)));
        svc.Connect("mouse_exited", new Godot.Callable(this, nameof(OnMouseExited)));


        rb.Mass = cm;
        rb.Inertia = cmoi.toGV();
        rb.CenterOfMass = r.toGV();

    }

    public override void _Input(Godot.InputEvent Event){
        if(Event is Godot.InputEventMouseButton mouseClickEvent){ //[InputEvent] is [InputEventMouseButton] [variable name] 方の一致不一致と型変換と代入と宣言を同時に行うscopeはたぶん_Inputのなか
            //click
            switch(mouseClickEvent.ButtonIndex){
                case Godot.MouseButton.Left:
                    Input.MBL = mouseClickEvent.Pressed;
                    break;
                case Godot.MouseButton.Right:
                    Input.MBR = mouseClickEvent.Pressed;
                    break;
                case Godot.MouseButton.Middle:
                    Input.MBM = mouseClickEvent.Pressed;
                    break;
                case Godot.MouseButton.WheelUp:
                case Godot.MouseButton.WheelDown:
                    break;
            }
        } 
        if(Event is Godot.InputEventMouseMotion mouseMotionEvent){
            //move
            //if(Input.isMouseInside){
            Input.MMV[0] = mouseMotionEvent.Position;
                if(Input.MBR){
                    //回転操作
                            camAngl[0] -= sensit * 2 * FOV * (Input.MMV[0].X - Input.MMV[1].X)/svc.Size.X;
                        if(camAngl[0] <= -MathF.PI){
                            camAngl[0] += 2 * MathF.PI;
                        }else if( MathF.PI <= camAngl[0] ){
                            camAngl[0] -= 2 * MathF.PI;
                        }
                        //制限をかけるため 角度へらしてるときには-piの制限，ふやしてるときには+pi こうすれば制限外にでても復帰できる
                        if(Input.MMV[0].Y - Input.MMV[1].Y < 0){
                            if(-MathF.PI/2 < camAngl[1])
                                    camAngl[1] += sensit * 2 * FOV * (Input.MMV[0].Y - Input.MMV[1].Y)/svc.Size.Y;
                        }else{
                            if(MathF.PI/2 > camAngl[1])
                                    camAngl[1] += sensit * 2 * FOV * (Input.MMV[0].Y - Input.MMV[1].Y)/svc.Size.Y;
                        }
                            //camのTransformに適用
                            G.Transform3D transs = G.Transform3D.Identity;

                            GD.Print($"camangl ${camAngl[0]}, ${camAngl[1]}");

                            //camAngl[0] = (float)time * 10;
                            Quat q = new Quat(new Vec3(0,0,1), camAngl[0]);

                            q = Quat.concat(q,new Quat(new Vec3(0,-1,0) , camAngl[1]));
                            
                            transs.Origin = (new Vec3(camd,0,0).rotate(q)).toGV();

                            GD.Print($"origin {transs.Origin.X},{transs.Origin.Y},{transs.Origin.Z},");

                            transs.Basis.X = (new Vec3(0,1,0).rotate(q)).toGV();
                            transs.Basis.Y = (new Vec3(0,0,1).rotate(q)).toGV();
                            transs.Basis.Z = (new Vec3(1,0,0).rotate(q)).toGV();


                            cam.Transform = transs;
                }else if(Input.MBM){
                }
                if(Input.MBL){
                    // dragging
                    if((Input.MMV[0] - ppos - cr.Position - cr.Size/2).Length() < 10){
                        Input.isDragging = true;
                    }
                }else{
                    Input.isDragging = false;
                }
                
            Input.MMV[1] = Input.MMV[0];
            //}
        }
        if(Event is Godot.InputEventKey keyEvent){
            //key
            if(keyEvent.Pressed){
                //enter
                /*
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
                */
                    switch(keyEvent.Keycode){
                        case Godot.Key.Space:
                            break;
                        case Godot.Key.S:
                            doControl = true;
                            break;
                        case Godot.Key.Q:
                            doControl = false;
                            break;
                        case Godot.Key.Key1:
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
                            break;
                        default:
                            break;
                    }
            }
        }
    }
    private void OnMouseEntered(){
        Input.isMouseInside = true;
    }
    private void OnMouseExited(){
        Input.isMouseInside = false;
    }
    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta) {
        time += delta;

            // instance → data  update
        pos.setFromGV(ab.Transform.Origin);
        ori[0].setFromGQ(new G.Quaternion(rb.Transform.Basis));
        oriv[0] = ori[0].toV();
        //avel = Vec3.divide(Vec3.add(oriv[0] , oriv[1].invert()) , (float)delta);
        avel.setFromGV(rb.AngularVelocity);
        avel = avel.rotate(ori[0].invert());

            //GD.Print($"ori {ori.toString()}");
        //GD.Print(rb.AngularVelocity.X);
        //avel.setFromGV(rb.AngularVelocity);
            //DebugDraw3D.DrawArrow(new G.Vector3(0,0,0), rb.AngularVelocity*20, G.Colors.Red, 1F, false);
            //DebugDraw3D.DrawCylinder(new G.Transform3D(G.Basis.Identity,rb.AngularVelocity*20));
            //DebugDraw3D.DrawCylinderAb(new G.Vector3(0,0,0), rb.AngularVelocity*20, 2f, G.Colors.Red, 0.0f);
            //DebugDraw3D.DrawArrowRay(new G.Vector3(0,0,0), new G.Vector3(100,0,0), 80f, G.Colors.Lavender, 50f, true);
        //avel = avel.rotate(ori[0].invert());
            //GD.Print($"avel(local) : {(new Vec3(1000,0,0).rotate(ori.invert())).toString()}");



        /*
        double deltaT = 1;
        double startTime = 1;
        if(startTime < time && time < startTime+deltaT){
            doControl = true;

        }else if(startTime+deltaT < time && time < startTime+2*deltaT){
            doControl = false;
        }else{
            doControl = false;
        }
        */


        if(doControl){
            majiKeisandekiru3(delta);
        }else{
            torque.x = 0;
            torque.y = 0;
            torque.z = 0;
        }


            // local to global   apply torque(change energy of reaction wheel)
        rb.ApplyTorque(torque.rotate(ori[0]).toGV());


        ori[1] = ori[0];
        oriv[1] = oriv[0];

        /*


        // 1/2 で実行
        if(f2){
        }

        // for test
        //avel.multiply((1-ATTE*delta));

        //impulse = Vec3.multiply(force, delta);

        // Fdt = dp = mdv mdx/dt
        //Vec3 acc = Vec3.multiply(force , delta/m);
        //vel.add(acc);
        //vel.add(Vec3.multiply(impulse , 1/m));
        vel.add(Vec3.multiply(force , delta/m));
        pos.add(Vec3.multiply(vel, delta));

        double tdelta = delta;
        Vec3 aimpulse = Vec3.multiply(torque , tdelta);
        Vec3 tv = Vec3.add( force.invert() , new Vec3(0,0,GRAV*m ));
        for(int i=0; i<STEP; i++){

            // eular method  
            
            // torque * dt = r×Fdt
            //aimpulse = Vec3.add(aimpulse,Vec3.cross( r , tv.rotate(ori.invert())));
            aimpulse = Vec3.cross( r , tv.rotate(ori.invert()));

            //GD.Print($"torque : {torquetmp.toString()}");
                // calculate second derivative
                // coordinate G to L
            Vec3 avelL = avel.rotate(ori.invert());

            //GD.Print($"avelL : {avelL.toString()}");
                // from "eular's equation of motion" ( at primary moment axis coordinate )
            Vec3 aavel = new Vec3();
            aavel.x = (aimpulse.x + (pmoi.y - pmoi.z)*avelL.y*avelL.z) / pmoi.x ; 
            aavel.y = (aimpulse.y + (pmoi.z - pmoi.x)*avelL.z*avelL.x) / pmoi.y ; 
            aavel.z = (aimpulse.z + (pmoi.x - pmoi.y)*avelL.x*avelL.y) / pmoi.z ; 

            //GD.Print($"aavel : {aavel.toString()}");

                // coordinate trans. L2G
            aavel = aavel.rotate(ori);
            //aavel.multiply(tdelta);

                //
                // cal. first derivative
                //
            avel.add(Vec3.multiply(aavel,tdelta));
            //GD.Print($"avel : {avel.toString()}");

            //GD.Print($"pos : {pos.toString()}");


            //avel = new Vec3(0.05,0.05,0.00);



                // cal. zero d.
            ori = Quat.concat(new Quat(avel , avel.abs()*tdelta), ori);
            //GD.Print($"ori : {ori.toString()}");
            //GD.Print($"pos : {pos.toString()}");
            //GD.Print($"quat : {ori.toString()}");
            //GD.Print($"|quat| : {Math.Sqrt(Math.Pow(ori.w,2)+Math.Pow(ori.x,2)+Math.Pow(ori.y,2)+Math.Pow(ori.z,2))}");
        }

        //pos.x = (double)rpos[0].X*camd/(double)cr.Size.X;
        //pos.y = (double)rpos[0].Y*camd/(double)cr.Size.Y;

            // substitude 
        trans.Origin = pos.toGV();
        trans.Basis.X = (new Vec3(1,0,0).rotate(ori)).toGV();
        trans.Basis.Y = (new Vec3(0,1,0).rotate(ori)).toGV();
        trans.Basis.Z = (new Vec3(0,0,1).rotate(ori)).toGV();


        //trans.Origin = new G.Vector3(0,(float)time*100,0);
        //trans.Basis.X = new G.Vector3(1,0,0);
        //trans.Basis.Y = new G.Vector3(0,1,0);
        //trans.Basis.Z = new G.Vector3(0,0,1);

        //GD.Print($"basis x : {(new Vec3(1,0,0).rotate(ori)).toString()}");
        //GD.Print($"basis y : {(new Vec3(0,1,0).rotate(ori)).toString()}");
        //GD.Print($"basis z : {(new Vec3(0,0,1).rotate(ori)).toString()}");


        cansat.Transform = trans;
        //GD.Print($"{pos.x},{pos.y},{pos.z}");

        f2 = !f2;
        */
    }
    public override void _PhysicsProcess(double delta) {
            // moving 
        if(Input.isDragging){
            trans.Origin.X = camd*(Input.MMV[0].X - cr.Position.X - cr.Size.X/2)/cr.Size.X;
            trans.Origin.Y = camd*(-Input.MMV[0].Y + cr.Position.Y + cr.Size.Y/2)/cr.Size.Y;
            ab.Transform = trans;
        }
        //trans.Origin.Y = -camd + (float)time*1000;
        //    ab.Transform = trans;
    }
    private void majiKeisandekiru3(double delta){
        // bno can export  acceleration , orientation , gyro(= anglar velocity)


        //G.Transform3D transs = vo.Transform;
        //transs.Basis = new G.Basis(Quat.concat(new Quat(new Vec3(0,1,0) , tmp) , ori).toGQ());
        //vo.Transform = transs;


        GD.Print($"avel : {avel.toString()}");
        deltaV = Vec3.multiply(avel.invert() , Kp);

        deltaV.x -= oriv[0].x*Kd;
        deltaV.y -= oriv[0].y*Kd;
        deltaV.z -= oriv[0].z*Kd;

        // for graph
        avelL.Add(avel.z);
        anglL.Add(oriv[0].z);
        deltavL.Add(deltaV.z);
        if( 100 < avelL.Count ){
            avelL.RemoveAt(0);
            anglL.RemoveAt(0);
            deltavL.RemoveAt(0);
        }

        // deltaV to torque
        // Eular's equation of motion
        GD.Print($"ori : {ori[0].toString()}");
        GD.Print($"deltaV : {deltaV.toString()}");

        torque.x = cmoi.x*deltaV.x/(float)delta + (cmoi.z - cmoi.y)*avel.z*avel.y;
        torque.y = cmoi.y*deltaV.y/(float)delta + (cmoi.x - cmoi.z)*avel.x*avel.z;
        torque.z = cmoi.z*deltaV.z/(float)delta + (cmoi.y - cmoi.x)*avel.y*avel.x;
        torque.clip(maxTorque);
        GD.Print($"torque : {torque.toString()}");

        GD.Print("\n");
    }
}
