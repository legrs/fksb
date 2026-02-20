using System;
using System.Numerics;
using System;
using G = Godot;

public partial class Main{

    public class Vec3{
        public float x;
        public float y;
        public float z;
        private const float fac = (float)(1.0 / (1 << 14));

        // constr.
        public Vec3(){
            x = 0;
            y = 0;
            z = 0;
        }
        public Vec3(float x, float y, float z){
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static Vec3 add(Vec3 v1, Vec3 v2){
            return new Vec3(v1.x+v2.x , v1.y+v2.y , v1.z+v2.z);
        }
        public static Vec3 multiply(Vec3 v, float s){
            return new Vec3(v.x*s , v.y*s , v.z*s);
        }
        public static Vec3 divide(Vec3 v, float s){
            return new Vec3(v.x/s , v.y/s , v.z/s);
        }
        public static float dot(Vec3 v1, Vec3 v2){
            return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
        }
        public static Vec3 cross(Vec3 v1, Vec3 v2){
            Vec3 result = new Vec3();
            result.x = v1.y*v2.z - v1.z*v2.y;
            result.y = v1.z*v2.x - v1.x*v2.z;
            result.z = v1.x*v2.y - v1.y*v2.x;
            return result;
        }

        public Vec3 invert(){
            return new Vec3(-x , -y , -z);
        }
        public G.Vector3 toGV(){
            return new G.Vector3(x,y,z);
        }
        public float abs(){
            return MathF.Sqrt(MathF.Pow(x,2)+MathF.Pow(y,2)+MathF.Pow(z,2));
        }
        public Vec3 rotate(Quat q){
            Quat q1 = new Quat(this);
            Quat q2 = Quat.concat(Quat.concat(q,q1),q.invert());
            return new Vec3(q2.x,q2.y,q2.z);
        }
        public void multiply(float s){
            x *= s;
            y *= s;
            z *= s;
            return;
        }
        public void divide(float s){
            x /= s;
            y /= s;
            z /= s;
            return;
        }
        public void add(Vec3 v){
            x += v.x;
            y += v.y;
            z += v.z;
            return;
        }
        public void add(float v){
            x += v;
            y += v;
            z += v;
            return;
        }
        public void setFromGV(G.Vector3 v){
            x = v.X;
            y = v.Y;
            z = v.Z;
        }
        public void setFromByte(ReadOnlySpan<byte> buffer){
            short x1, y1, z1;
            x1 = y1 = z1 = 0;
            x1 = (short)(((buffer[1]) << 8) | (buffer[0]));
            y1 = (short)(((buffer[3]) << 8) | (buffer[2]));
            z1 = (short)(((buffer[5]) << 8) | (buffer[4]));
            x = x1;
            y = y1;
            z = z1;
        }
        // scale vector so that max value does not exceedthe threshold
        public bool clip(Vec3 thres){
            float[] fac = new float[3]{x/thres.x , y/thres.y , z/thres.z};
            float max = fac[0];
            float min = fac[0];
            if( max < fac[1] ){
                max = fac[1];
            }
            if( max < fac[2] ){
                max = fac[2];
            }
            if( fac[1] < min ){
                min = fac[1];
            }
            if( fac[2] < min ){
                min = fac[2];
            }

            if( MathF.Abs(min) < MathF.Abs(max) ){
                // clip on upper
                if( 1 < max ){
                    divide(max);
                    return true;
                }
            }else{
                if( min < -1 ){
                    divide(-min);
                    return true;
                }
            }
            return false;
        }
        public bool clip(float thres){
            float[] fac = new float[3]{x/thres , y/thres , z/thres};
            float max = fac[0];
            float min = fac[0];
            if( max < fac[1] ){
                max = fac[1];
            }
            if( max < fac[2] ){
                max = fac[2];
            }
            if( fac[1] < min ){
                min = fac[1];
            }
            if( fac[2] < min ){
                min = fac[2];
            }

            if( MathF.Abs(min) < MathF.Abs(max) ){
                // clip on upper
                if( 1 < max ){
                    divide(max);
                    return true;
                }
            }else{
                if( min < -1 ){
                    divide(-min);
                    return true;
                }
            }
            return false;
        }
        public String toString(){
            return $"{x} {y} {z}";
        }
    }

    public class Quat{
        public float w,x,y,z;

        // constructors
        public Quat(){
            w = 1;
            x = 0;
            y = 0;
            z = 0;
        }
        public Quat(float w, float x, float y, float z){
            this.w = w;
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public Quat(Vec3 v, float s){
            if(s==0){
                w = 1;
                x = 0;
                y = 0;
                z = 0;
            }else{
                float sin = MathF.Sin(s/2);
                this.w = MathF.Cos(s/2);
                float abs = v.abs();
                this.x = sin * v.x / abs;
                this.y = sin * v.y / abs;
                this.z = sin * v.z / abs;
            }
        }
        public Quat(Vec3 v){
            w = 0;
            this.x = v.x;
            this.y = v.y;
            this.z = v.z;
        }

        public Quat invert(){
            return new Quat(w, -x, -y, -z);
        }
        public static Quat concat(Quat q1, Quat q2){
            Quat result = new Quat();
            result.w = -q1.x*q2.x - q1.y*q2.y - q1.z*q2.z + q1.w*q2.w;
            result.x =  q1.w*q2.x - q1.z*q2.y + q1.y*q2.z + q1.x*q2.w;
            result.y =  q1.z*q2.x + q1.w*q2.y - q1.x*q2.z + q1.y*q2.w;
            result.z = -q1.y*q2.x + q1.x*q2.y + q1.w*q2.z + q1.z*q2.w;
            
            return result;
        }
        public Quat multiply(float s){
            float theta = MathF.Acos(w);
            float fac = MathF.Sin(s*theta)/MathF.Sin(theta);
            Quat q = new Quat();
            q.w = MathF.Cos(s*theta);
            q.x = x * fac;
            q.y = y * fac;
            q.z = z * fac;
            return q;
        }

        public Vec3 toV(){
            Vec3 v = new Vec3(0,0,0);
            // avoid non-stable divide
            // w=0.9999 → acos(w) = 0.810degreessssss +1145141919810 points
            if( w < -0.9999 || 0.9999 < w ){

            }else{
                double fac = 2 * Math.Acos(w) / Math.Sqrt( 1 - Math.Pow((double)w , 2));
                v.x = x * (float)fac;
                v.y = y * (float)fac;
                v.z = z * (float)fac;
            }
            return v;
        }
        public String toString(){
            return $"{w} {x} {y} {z}";
        }
        public void setFromGQ(G.Quaternion q){
            // q.w = MathF.Sqrt(Basis.X.X + Basis.Y.Y + Basis.Z.Z + 1)/2
            //float tmp = Basis.X.X + Basis.Y.Y + Basis.Z.Z + 1; 

            w = q.W;
            x = q.X;
            y = q.Y;
            z = q.Z;
        }
        public void setFromByte(ReadOnlySpan<byte> buffer){
            const float scale = (float)(1.0 / (1 << 14));
            short x1, y1, z1, w1;
            x1 = y1 = z1 = w1 = 0;
            w1 = (short)(((buffer[1]) << 8) | (buffer[0]));
            x1 = (short)(((buffer[3]) << 8) | (buffer[2]));
            y1 = (short)(((buffer[5]) << 8) | (buffer[4]));
            z1 = (short)(((buffer[7]) << 8) | (buffer[6]));
            w = w1 * scale;
            x = x1 * scale;
            y = y1 * scale;
            z = z1 * scale;
        }
        public G.Quaternion toGQ(){
            G.Quaternion q = new G.Quaternion();
            q.X = x;
            q.Y = y;
            q.Z = z;
            q.W = w;
            return q;
        }
     }




    public static float toRad(float d){
        return d*MathF.PI/180;
    }
}
