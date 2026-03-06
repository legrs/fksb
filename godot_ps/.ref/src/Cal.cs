using System.Numerics;
using System;

public partial class Main
{
    public static Vector3 rFS;
    
        //mass Ekg(10^18) size km
    //  Sun Macury Venus Earth,moon Mars,phobos,deimos Juptor,Io,europa,ganymede,callisto 
    //Saturn,titan,enceladus,dione,tethys,rhea,mimas Neptune,miranda,ariel,umbriel,titania,oberon
    //Uranus,triton,prometeus Plto,kalon
    //sun mercury venus earth mars jupitor saturn neptune uranus pluto
        
    public static double date2SHT(in string y,in string m,in string d,in string h,in string m2,in string s){ //グレゴリオ暦で2000とのHour差を符号付きで計算します..たぶん３０万年ぐらいでオーバーフローするからちゅうい.
        int.TryParse(y, out int year);
        int.TryParse(m, out int month);
        int.TryParse(d, out int date);
        int.TryParse(h, out int hour);
        int.TryParse(m2, out int minute);
        int.TryParse(s, out int second);
        Godot.GD.Print(month);
       
        //year = 2003;month=7;date=1;hour=0;    
        double result=0;
        if(year < 2000){///2000年前後で処理を分ける
            int uruuQty = year/4;
            uruuQty -= year/100;
            uruuQty += year/400;//グレゴリオ暦によるうるう年の定義
            uruuQty = Main.URUUQTY2000 - uruuQty; //うるう年計算終了
            
            year++; // 現在年をかぞえたらいけないから
            year = 2000 - year; //yearは０はじまりだから普通に引いてok
            result += uruuQty * 366;
            result += (year - uruuQty) * 365; //yearしょり終了
            year--;//もどした  
            for(int i=12;month<i;i--){ //mのしょり
                if(i==2){ //iは2~12の値を取る
                    if(year%4 == 0 && year%100 != 0 && year%200 != 0 && year%300 != 0 ){//うるう年かどうか
                        result+=29;
                    }else{
                        result+=28;
                    }
                }else{
                    result+= Main.mNissu[i-1];//配列が0始まりだkら
                } 
            }
            result += (Main.mNissu[month-1]) - date;
            result = result * 24;
            
            result += 23 - hour; // 24 - ( Hour +1 )を変形した形
            result += (59 - minute) / 60; //上に同じ原理
            result = -result;
            
        }else{

            year--; //現在年のうるう年はかぞえたらいけない
            int uruuQty = year/4;
            uruuQty -= year/100;
            uruuQty += year/400;
            uruuQty = uruuQty - Main.URUUQTY2000; //うるう年計算終了
            year++; //もどした

            year = year - 2000;
            result += uruuQty * 366;
            result += (year - uruuQty) * 365; //yearしょり終了
            
            month--; //１始まりだから経験した月の数はマイナス１になる
            for(int i=0;i<month;i++){ //mのしょり 現在月はかぞえないので<
                if(i==1){ //0はじまりだからi=1で２月
                    if(year%4 == 0 && year%100 != 0 && year%200 != 0 && year%300 != 0 ){//うるう年かどうか
                        result+=29;
                    }else{
                        result+=28;
                    }
                }else{
                    result+=Main.mNissu[i];
                } 
            }
            date--; //１始まりだから
            result += date;
            result = result * 24;　//太陽日からhourへ
            result += hour;
            result += (double)minute/60;
            result += (double)second/3600;
        }
        return result;

    }/*
    public static void fixEpochTime(in int ci , in float SHTtime){//近日点通過時刻を修正する
        float TpeD;
        if(SHTtime < Main.CelE.o[ci].epo){
            TpeD = (Main.CelE.o[ci].epo - SHTtime) / Main.CelE.o[ci].period;
            TpeD = Main.CelE.o[ci].period * (int)TpeD;
            TpeD += Main.CelE.o[ci].period;//１つ前の近日点通過時刻にする必要があるため
            TpeD = -TpeD;
        }else{
            TpeD = (SHTtime - Main.CelE.o[ci].epo) / Main.CelE.o[ci].period;
            TpeD = Main.CelE.o[ci].period * (int)TpeD;
        }
        Main.CelE.Fepo[ci] = Main.CelE.o[ci].epo + TpeD;
        
    }*/
    public static void calOrbiterPosition(in int ci , in double SHTtime, in bool isV){ //newton method
        if( ci == 0 ){

        }else{
            double M, E, tmp, f;
            float a,b,c;

            M = SHTtime / Main.CelE.o[ci].period;
            M = M % 1; //小数部分o
            M = M * 2 * Math.PI + (double)Main.CelE.o[ci].epo; //平均近点角rad
            if( 2*Math.PI <= M){
                M -= 2*Math.PI;
            }
            //                                                                                 Godot.GD.Print(toDeg(M));
            a = (float)(M / (1 - Main.CelE.o[ci].e));
            b = (float)(M + Main.CelE.o[ci].e);
            c = (float)((M + Math.PI * Main.CelE.o[ci].e)/(1 + Main.CelE.o[ci].e));
            //どれが一番小さいか（安定して収束させるためのE0を求める
                if(a < b){
                    if(a < c){
                        E = a;
                    }else{
                        E = c;
                    }
                }else{ // b < a
                    if(b < c){
                        E = b;
                    }else{
                        E = c;
                    }
                }
            for(int i=0; i<4; i++){ //ニュートン法 M = E - eSinEを解く
                tmp = E - Main.CelE.o[ci].e * Math.Sin(E) -M;//分母
                tmp = tmp /( 1 - Main.CelE.o[ci].e * Math.Cos(E));//分子
                if((float)tmp == 0F){
                    E -= tmp;//
                    break;
                }
                E -= tmp;//
            } 
            tmp = Math.Cos(E); 
            f = (tmp + Main.CelE.o[ci].e)/(1 + Main.CelE.o[ci].e * tmp); //cossita

            //速度ベクトルだす
            b = -MathF.Sqrt(1 - MathF.Pow((float)f,2)); //x
            c = CelE.o[ci].e+(float)f; //y
            if( Math.PI < E )
                b = -b;
            if(ci==3){
                //Godot.GD.Print($" cosf : {f}  b : {b}  c : {c} ");
            }
            a = b;
            b = a * CelE.o[ci].c0 - c * CelE.o[ci].s0;
            c = a * CelE.o[ci].s0 + c * CelE.o[ci].c0;
            CelE.v[ci].X = (b * Main.CelE.o[ci].c2) - (c * Main.CelE.o[ci].c1 * Main.CelE.o[ci].s2);
            CelE.v[ci].Y = (b * Main.CelE.o[ci].s2) + (c * Main.CelE.o[ci].c1 * Main.CelE.o[ci].c2);
            CelE.v[ci].Z = c * Main.CelE.o[ci].s1;
            CelE.v[ci] = CelE.v[ci] * ((float)CelE.grav[0]/((float)CelE.o[ci].l*1000)); //1000は単位あわせ これはvis-vivaから
            CelE.v[ci] = CelE.v[ci] * (MathF.Sqrt(CelE.v[ci].Length())/CelE.v[ci].Length());

            

            //E = 極座標のr  変数をつりたくなかった
            E = Main.CelE.o[ci].l / (1 + Main.CelE.o[ci].e * f);

            //ここでf = cosf
            CelE.o[ci].f = MathF.Acos((float)f);
            tmp = Math.Sqrt(1 - Math.Pow(f,2));
            //fはcosなので半分越えてたら反転
            if( M > Math.PI){
                tmp = -tmp;
                CelE.o[ci].f = MathF.PI - CelE.o[ci].f;
            }
            M   = E * (f*CelE.o[ci].c0 - tmp*CelE.o[ci].s0);
            tmp = E * (f*CelE.o[ci].s0 + tmp*CelE.o[ci].c0);
            if(isV){
                Main.vpositions[ci , 0] = (M * (double)Main.CelE.o[ci].c2) - (tmp * (double)Main.CelE.o[ci].c1 * (double)Main.CelE.o[ci].s2);
                Main.vpositions[ci , 1] = (M * (double)Main.CelE.o[ci].s2) + (tmp * (double)Main.CelE.o[ci].c1 * (double)Main.CelE.o[ci].c2);
                Main.vpositions[ci , 2] = tmp * (double)Main.CelE.o[ci].s1;
            }else{
                Main.positions[ci , 0] = (M * (double)Main.CelE.o[ci].c2) - (tmp * (double)Main.CelE.o[ci].c1 * (double)Main.CelE.o[ci].s2);
                Main.positions[ci , 1] = (M * (double)Main.CelE.o[ci].s2) + (tmp * (double)Main.CelE.o[ci].c1 * (double)Main.CelE.o[ci].c2);
                Main.positions[ci , 2] = tmp * (double)Main.CelE.o[ci].s1;
            }
        }

    }

    public static Orbit solveOrbit(in Vector3 v,in int ci,in Vector3 r, double v0, double r0){ //速度, 主星, 主星からの位置
        Orbit o = new Orbit();
        o.ci = ci;
        double tmp = (r0 * Math.Pow(v0, 2) * 1000)/ (CelE.grav[ci]); //たくさん出てくるので   単位あわせで1000かけてる
        double sin = Vector3.Dot(v, r) / (v0 * r0); //sinγ
        double cos = Math.Sqrt(1 - Math.Pow(sin, 2)); //なす角はpi未満
        o.e = (float)Math.Sqrt( ((tmp - 2) * tmp * Math.Pow(cos, 2)) + 1);
        // e ok
        

        // 軌道面を決定
        Vector3 n = Vector3.Cross(r, v); //軌道面の法線 
        float n0 = n.Length();
        o.c1 = n.Z/n0; //n0!=0
        o.s1 = MathF.Sqrt(1 - MathF.Pow(o.c1 , 2));
        if(o.s1 != 0){ //ゼロ除算防止 軌道傾斜角0ならそもΩ定まらず
            o.s2 = n.X/(o.s1*n0);
            o.c2 = -n.Y/(o.s1*n0);
        }


        //vis-viva 方程式によるLaxsis
        //準線と焦点を使った楕円の定義から導出
        o.l = (CelE.grav[ci]*r0 / (((CelE.grav[ci]/500) - Math.Pow(v0, 2)*r0)*1000)) * (1+(double)o.e)*(1-(double)o.e);

        //近点引数を計算

            //軌道面上に変換したやつ
        Vector2 vv = new Vector2(v.X * o.c2 + v.Y * o.s2 , o.c1 * ( v.Y * o.c2 - v.X * o.s2 ) + v.Z * o.s1);//  ,  v.Z * o.c1 - o.s1 * ( v.Y * o.c2 - v.X * o.s2 ) );
        Vector2 rr = new Vector2(r.X * o.c2 + r.Y * o.s2 , o.c1 * ( r.Y * o.c2 - r.X * o.s2 ) + r.Z * o.s1);//  ,  r.Z * o.c1 - o.s1 * ( r.Y * o.c2 - r.X * o.s2 ) );

        float bunbo = (float)((tmp * Math.Pow(cos, 2))-1); //c^2 -1 = -s^2
        float bunshi = (float)(tmp * sin * cos);

        float som = 0;
        if(MathF.Log(MathF.Abs(bunshi)) - MathF.Log(MathF.Abs(bunbo)) < 7){ //許容する桁数 10 だったら 10^10までのtanを許容
        //aをだす

            float tf = bunshi/bunbo;//tan(真近点角)

            //楕円の極方程式より cos(真近点角)
            //float cf = -((float)r0 - o.l)/(o.e*(float)r0);

            Vector2 rrr = new Vector2(rr.X + (tf*rr.Y) , rr.Y - (tf*rr.X));
            som = MathF.Atan2(rrr.Y , rrr.X); // -pi< <pi atan2はすばらしい
            /*
            if(cf < 0){
                som -= MathF.PI;
            }
            */
            if(r0 > o.l){  // 正負がわかれば計算しなくていい
                som -= MathF.PI;
            }
            



        }else{
            ///f=pi/2 , 3pi/2
Godot.GD.Print("aaaaaa");
            som = MathF.Atan2(-rr.X , rr.Y);
            if( Vector2.Dot(rr,vv) > 0 ){ //とおざかっているときは逆がわ
                som += MathF.PI;
            }
        }
            o.c0 = MathF.Cos(som); //rad
            o.s0 = MathF.Sin(som); //rad
            o.om = som; //rad
        return o;
    }
    public static String SHT2date(double t){
        int yd = (int)Math.Truncate(t / 24);
        int y = yd / 365;
        int u = 0;
        if( 0 < t ){
            y -= 1;
            u += y / 4;
            u -= y / 100;
            u += y / 400;
            if( 0 <= y ){
                u++; //2000のも含めたいから
            }
            yd -= u;
        }else{
            y += 2001;
            u += y / 4;
            u -= y / 100;
            u += y / 400;
            u = Main.URUUQTY2000 - u;
            yd += u;
        }
        // yが正しくないので注意
        y = yd / 365;
        int d = yd - 365*y + 1; //1 base ydを調整してあるのでつねに365日としてよい
        int m = 0;
        for(m=0; mNissu[m]<d; m++){
            d -= mNissu[m];
            if(((y%4 == 0 && y%100 != 0) || (y%400 == 0))&&(d!=29)&&(m==1)){
                d--;
            }
        }
        m++;
        y += 2000;

        float h = (float)(t % 24);
        float min = (h % 1)*60;
        float s = (min % 1)*60;
        
        h = MathF.Truncate(h);
        min = MathF.Truncate(min);
        s = MathF.Truncate(s);

        return $"{y.ToString().PadLeft(4, '0')}{m.ToString().PadLeft(2, '0')}{d.ToString().PadLeft(2, '0')}{h.ToString().PadLeft(2, '0')}{min.ToString().PadLeft(2, '0')}{s.ToString().PadLeft(2, '0')}";

    }
    public static void calProbePosition(in int s, in double deltaT){
        //probeの位置計算 { x[i+1] = x[i] + v[i]Dt + a[i]Dt^2 / 2
            positions[0 ,0] += Probe.velo[0] * deltaT + (Probe.acce2[0] * Math.Pow(deltaT,2) / 2000);
            positions[0 ,1] += Probe.velo[1] * deltaT + (Probe.acce2[1] * Math.Pow(deltaT,2) / 2000);
            positions[0 ,2] += Probe.velo[2] * deltaT + (Probe.acce2[2] * Math.Pow(deltaT,2) / 2000);
            vpositions[0, 0] = positions[0 ,0];
            vpositions[0, 1] = positions[0 ,1];
            vpositions[0, 2] = positions[0 ,2];
            Probe.aVeloQ = System.Numerics.Quaternion.CreateFromAxisAngle( Probe.aVelo.v , Probe.aVelo.s * (float)deltaT );
            Probe.anglQ = System.Numerics.Quaternion.Concatenate( Probe.anglQ , Probe.aVeloQ );
        // }
            //////////////////////////
        Probe.acce[0] = 0;
        Probe.acce[1] = 0;
        Probe.acce[2] = 0;
        for(int ci=0; ci<maxCI; ci++){
            //天体の位置の計算 {
            if(ci == 0){// sun
                diffPos[0] =  -positions[0, 0]; //km
                diffPos[1] =  -positions[0, 1]; 
                diffPos[2] =  -positions[0, 2]; 
            }else{
                calOrbiterPosition(ci , RSHT , false);
                diffPos[0] = positions[ci, 0] - positions[0, 0]; //km 
                diffPos[1] = positions[ci, 1] - positions[0, 1]; 
                diffPos[2] = positions[ci, 2] - positions[0, 2]; 
            }
            // } 

            //重力 {
            double d = Math.Sqrt(Math.Pow(diffPos[0], 2) + Math.Pow(diffPos[1], 2) + Math.Pow(diffPos[2], 2)); //原点（ｐｒｏｂｅ）からの距離km
            if( CelE.radius[ci] < d ){//零除算防止
                Probe.acce[0] += CelE.grav[ci] * (diffPos[0]) / Math.Pow(d, 3);//重力定数/きょり2条 に各成分の割合をかける
                Probe.acce[1] += CelE.grav[ci] * (diffPos[1]) / Math.Pow(d, 3);
                Probe.acce[2] += CelE.grav[ci] * (diffPos[2]) / Math.Pow(d, 3);
            }else{
                Probe.velo[0]  = 0;
                Probe.velo[1]  = 0;
                Probe.velo[2]  = 0;
                Probe.acce[0] = 0;
                Probe.acce[1] = 0;
                Probe.acce[2] = 0;
                //衝突中
            }
            //}
            if(harfC && s==0){ //1度実行すればいい重い処理
                if(ci==0 ){ //発電計算
                    Vector3 z = QVQ(Probe.anglQ, new Vector3(1,0,0));
                    Vector3 p = new Vector3(-(float)positions[0,0]/100000000 , -(float)positions[0,1]/100000000 , -(float)positions[0,2]/100000000);
                    if(Vector3.Dot(p,z) < 0){
                        Probe.solarEnergy = 0;
                    }else{
                        d = d / 100000000;
                        Probe.solarEnergy = Probe.solarConf * Vector3.Dot(p,z)/ MathF.Pow((float)d,3);
                        d = d*100000000;
                    }
                }
                //表示上の天体の位置計算 {
                if( MODE[0] == 0 ){
                    diffPos[0] = diffPos[0] * Inputs.scale[0];
                    diffPos[1] = diffPos[1] * Inputs.scale[0];
                    diffPos[2] = diffPos[2] * Inputs.scale[0];
                    d = d * Inputs.scale[0];
                    CelCon.fixCelPosition(in ci, in diffPos, in d);
                }else if(MODE[0] == 1){
                    //
                }
                // }
            }
        }


        //推力{

            if(s==0){ //1フレーム内でrFSの値は変わらないので最初だけ
                Probe.aAcce.v = new Vector3(0,0,0); //合計するためにとりあえずぜろ
                Probe.aAcce.s = 0;
                Vector3 rF;
                for(int i=0; i<Probe.thruster.Length; i++){
                    if( Probe.thruster[i].rate != 0 ){
                        rF = Vector3.Transform( Probe.thruster[i].F , Probe.anglQ );
                        //Godot.GD.Print(mtostring(rF));
                        //Godot.GD.Print(rF.Z * Probe.thruster[i].rate);
                        // m/ss
                        rFS += rF * Probe.thruster[i].rate / Probe.mass;

                        Probe.aAcce.v += Vector3.Transform( Probe.thruster[i].T * Probe.thruster[i].rate , Probe.anglQ ); //トルクを合計する
                    }
                }
                Probe.aAcce.s = Probe.aAcce.v.Length();
                if( Probe.aAcce.s == 0){ //回転なし  でも軸ひつよう
                    Probe.aAcce.v.X = 1;
                    Probe.aAcce.v.Y = 0;
                    Probe.aAcce.v.Z = 0;
                }else{
                    Probe.aAcce.v = Probe.aAcce.v / Probe.aAcce.s; //正規化
                    //In = Ixx nx^2 + Iyy ny^2 + Izz nz^2 + 2Ixy nx ny + 2I xz nx nz + 2I yz ny nz
                    float I = (Probe.MOI[0,0] * Probe.aAcce.v.X * Probe.aAcce.v.X) + (Probe.MOI[1,1] * Probe.aAcce.v.Y * Probe.aAcce.v.Y) + (Probe.MOI[2,2] * Probe.aAcce.v.Z * Probe.aAcce.v.Z) + (2 * Probe.MOI[0,1] * Probe.aAcce.v.X * Probe.aAcce.v.Y) + (2 * Probe.MOI[0,2] * Probe.aAcce.v.X * Probe.aAcce.v.Z) + (2 * Probe.MOI[1,2] * Probe.aAcce.v.Y * Probe.aAcce.v.Z);
                    if( I==0 ){
                        Godot.GD.Print("reijozann");
                    }else{
                        Probe.aAcce.s = Probe.aAcce.s / I;
                    }
                }
            }
            Probe.acce[0] += (double)rFS.X;
            Probe.acce[1] += (double)rFS.Y;
            Probe.acce[2] += (double)rFS.Z;

        // }

        //姿勢制御{
            if(posContT >= 0 && s==0){
                //Quaternion a = System.Numerics.Quaternion.Concatenate( Probe.anglQ , System.Numerics.Quaternion.Conjugate(posContQ) );
                //Probe.anglQ = QVQ(new Quaternion(a.X,a.Y,a.Z,a.W/posContT) , )
                if(posContT<deltaT*ts[0]){
                    Probe.anglQ =  posContQ;
                }else{
                    Probe.anglQ = System.Numerics.Quaternion.Lerp(Probe.anglQ , posContQ , 1/posContT);
                }
                posContT -= (float)(deltaT*ts[0]);
            }
        //}

        //速度計算{ v[i+1] = v[i] + (a[i]+a[i+1])Dt/2
            Probe.velo[0] += (Probe.acce[0] + Probe.acce2[0]) * deltaT / 2000;
            Probe.velo[1] += (Probe.acce[1] + Probe.acce2[1]) * deltaT / 2000;
            Probe.velo[2] += (Probe.acce[2] + Probe.acce2[2]) * deltaT / 2000;
            Probe.aVeloQ = System.Numerics.Quaternion.CreateFromAxisAngle( Probe.aVelo.v , Probe.aVelo.s );
            Probe.aVeloQ = System.Numerics.Quaternion.Concatenate( Probe.aVeloQ , System.Numerics.Quaternion.CreateFromAxisAngle( Probe.aAcce.v , Probe.aAcce.s * (float)deltaT ) );
            if( Probe.aVeloQ.W >= 1 || Probe.aVeloQ.W <= -1 ){//回転なし
                Probe.aVelo.v.X = 1;
                Probe.aVelo.v.Y = 0;
                Probe.aVelo.v.Z = 0;
                Probe.aVelo.s = 0;
            }else{
                Probe.aVelo.s = 2 * MathF.Acos( Probe.aVeloQ.W );
                Probe.aVelo.v.X = Probe.aVeloQ.X / MathF.Sin(Probe.aVelo.s / 2);
                Probe.aVelo.v.Y = Probe.aVeloQ.Y / MathF.Sin(Probe.aVelo.s / 2);
                Probe.aVelo.v.Z = Probe.aVeloQ.Z / MathF.Sin(Probe.aVelo.s / 2);
            }

            Probe.acce2[0] = Probe.acce[0];
            Probe.acce2[1] = Probe.acce[1];
            Probe.acce2[2] = Probe.acce[2];
        // }
    }
    //平行投影
    public static Godot.Vector2 paraPro(float x, float y, float z, in float s0, in float s1, in float c0, in float c1){
        Godot.Vector2 v;
            v.X =  -x * s0;
            v.X +=  y * c0;
            v.Y =   x * s1 * c0;
            v.Y +=  y * s1 * s0;
            v.Y += -z * c1;
            return v;
    }
    //(0,0,|v|)からの回転を取得
    public static Quaternion V2Q(in float x, in float y, in float z){
        if( x == 0 && y == 0 ){
            if( 0 <= z ){
                return new Quaternion(0,0,0,1);
            }else{
                return new Quaternion(1,0,0,0);
            }
        }else{
            float a = (float)(Math.Pow(x, 2) + Math.Pow(y, 2));
            float d = (float)-Math.Sqrt( a + Math.Pow(z, 2) );
            a = (float)Math.Sqrt(a);
            float cos = z / d;
            float sin = a / d;
            cos = (float)Math.Sqrt( (1+cos)/2 );
            sin = sin / ( 2 * cos );
            return new Quaternion( sin * x / a , sin * -y / a , 0, cos );
        }
    }
    
    //黄経0黄緯0からの黄道座標系での回転した単位ベクトル
    public static Vector3 stdRotate(in float sita, in float phi){
        Vector3 v;
        float c1 = MathF.Cos(phi);
        v.X = c1 * MathF.Cos(sita);
        v.Y = c1 * MathF.Sin(sita);
        v.Z = MathF.Sin(phi);
        return v;


    }
    public static Vector3 QVQ(in Quaternion q, Vector3 v){
        Quaternion q2 = System.Numerics.Quaternion.Concatenate( new Quaternion(v.X, v.Y, v.Z, 0) , q );
        q2 = System.Numerics.Quaternion.Concatenate( System.Numerics.Quaternion.Conjugate(q) , q2  );
        v.X = q2.X;
        v.Y = q2.Y;
        v.Z = q2.Z;
        return v;
    }
    //class
    /*
    public class Thruster{
        public Vector3 F ;
        public Vector3 FR; 
        public float f;
        //public float fr;
        public float rate = 0;
    }
    */
    public class Thruster{
        public Vector3 F;
        public Vector3 T;  //r*F
        public float rate = 0;
    }
    public class Rotate{
         public Vector3 v;
         public float s;
         public Rotate(float x, float y, float z, float w){
            v = new Vector3(x, y, z);
            s = w;
         }
     }
    public class Orbit{
        public double period = 1000 ;//公転周期
        public float  e      = 0    ;//離心率
        public double l  = 10000;//軌道長半径
        public float  epo    = 0    ;//SHT0でのM

        public float om = 0; //近日点黄経rad
        //0文字目がsin,cos 1文字目が，1：軌道傾斜角 2：昇交点赤経
        public float s0 = 0; 
        public float s1 = 0;
        public float s2 = 0;
        public float c0 = 1; //近日点黄経cos
        public float c1 = 1;
        public float c2 = 1;

        public float f = 0;

        public int ci = 0; //主星
        public Orbit(double p, float e, double l, float epo, float om, float s0, float c0, float s1, float s2, float c1, float c2, float f,int ci){
            period = p;
            this.e = e;
            this.l = l;
            this.epo = epo;
            this.om = om;
            this.s0 = s0;
            this.s1 = s1;
            this.s2 = s2;
            this.c0 = c0;
            this.c1 = c1;
            this.c2 = c2;
            this.f = f;
            this.ci = ci;
        }
        public Orbit(){
            period = 1000;
            e = 0;
            l = 10000;
            epo = 0;
            s0 = 0;
            s1 = 0;
            s2 = 0;
            c0 = 1;
            c1 = 1;
            c2 = 1;
            f = 0;
            ci = 0;
        }
    }
    public static Godot.Vector3 toGV(in Vector3 v){
        return new Godot.Vector3(v.X , v.Y , v.Z);
    }
    public static Vector3 toMV(in Godot.Vector3 v){
        return new Vector3(v.X , v.Y , v.Z);
    }
    public static double toRad(double deg){
        return deg * Math.PI / 180;
    }
    public static double toDeg(double rad){
        return rad * 180 / Math.PI;
    }
    public static float toRadF(float deg){
        return deg * MathF.PI / 180;
    }
    public static float toDegF(float rad){
        return rad * 180 / MathF.PI;
    }
}
