
using System.Collections;
using System.Collections.Generic; 
using UnityEngine;
//using DifiningObjects_MPControl;

/*
    前置き
    コメントアウト機能を利用して各式の(全く不十分な)説明を行っているが、その時に利用する文字等について先に示しておく
    t:そのループ時の時刻
    τ:予測範囲での時刻、時刻tのときτ=0である
    d~:微小な~ ex:dx/dtはxの時間微分表す
    ð~:ex:ðx/ðuはxのuによる偏微分を表す
    x[t]:時刻tにおける目標物の位置
    v[t]:時刻tにおける目標物の速度
    u[t]:時刻tにおける入力

    パラメータ記録
    -2021/19/Jan(非ホロノミック的性質よりxは収束しないがyとthetaは上手いこと言った、入力も振動せず)
    PosRef_x 0
    PosRef_y 3
    GMRES_RepeatTime 2
    PredictionTime 10
    SstaleConstant 100
    YConstant 3
    YCOnstant_Stage 1
    ThetaConstant 3
    ThetaConstant_Stage 1
    LForceConsant_Stage 0.5
    RForceConsant_Stage 0.5
    その他 0
*/

public class MPControl : MonoBehaviour{
    public bool MPC_mode=true;
    public Natural natural;
    public MakeTargetPoint makeTargetPoint;
    public int GMRES_RepeatTime=2;
    public int PredictionTime=10;
    public float StableConstant=0.1f;
    public float XConstant=1,ZConstant=1,VxCosntant=1,  VzConstant=1,XConstant_Stage=1,ZConstant_Stage=1,VxConstant_Stage=1,VzConstant_Stage=1,
                    AxConstant_Stage=1,AzConstant_Stage=1,JxConstant_Stage=1,JzConstant_Stage=1,SxConstant_Stage=1,SzConstant_Stage=1;
    public float FinalEvaluarionScope=0.5f;//[s]
    public float[] BodyPos_x,BodyPos_z;
    public float[] BodyVel_x,BodyVel_z;
    public float[] BodyAcc_x,BodyAcc_z;
    public float[] BodyJerk_x,BodyJerk_z;
    public float[] BodySnap_x,BodySnap_z;
    //public List<float> BodyPosXForTrajectory=new List<float>(),BodyPosZForTrajectory=new List<float>(),
    //                    BodyVelXForTrajectory=new List<float>(),BodyVelZForTrajectory=new List<float>();
    float[] DiffBodySnap_x,DiffBodySnap_z;    
    float[] AdVec_x,AdVec_vx,AdVec_z,AdVec_vz,AdVec_ax,AdVec_az,AdVec_jx,AdVec_jz;//随伴変数
    float PreBodyPos_x,PreBodyVel_x,PreBodyAcc_x,PreBodyPos_z,PreBodyVel_z,PreBodyAcc_z;//previous
    float PosRef_x,PosRef_z;
    float InitialTime;
    Transform BodyTransform;
    bool isCloseToTarget=false;

    // Start is called before the first frame update
    void Start(){
        BodyPos_x=new float[PredictionTime+1];
        BodyPos_z=new float[PredictionTime+1];
        BodyVel_x=new float[PredictionTime+1];
        BodyVel_z=new float[PredictionTime+1];
        BodyAcc_x=new float[PredictionTime+1];
        BodyAcc_z=new float[PredictionTime+1];
        BodyJerk_x=new float[PredictionTime+1];
        BodyJerk_z=new float[PredictionTime+1];
        BodySnap_x=new float[PredictionTime+1];
        BodySnap_z=new float[PredictionTime+1];
        DiffBodySnap_x=new float[PredictionTime+1]; 
        DiffBodySnap_z=new float[PredictionTime+1];
        AdVec_x=new float[PredictionTime+1]; 
        AdVec_vx=new float[PredictionTime+1]; 
        AdVec_ax=new float[PredictionTime+1];
        AdVec_jx=new float[PredictionTime+1];
        AdVec_z=new float[PredictionTime+1]; 
        AdVec_vz=new float[PredictionTime+1]; 
        AdVec_az=new float[PredictionTime+1];
        AdVec_jz=new float[PredictionTime+1];
        BodyTransform=natural.BodyTransform;

        for(int i = 0; i <PredictionTime+1 ; i++){
            DiffBodySnap_x[i]=0; 
            DiffBodySnap_z[i]=0; 
            BodyAcc_x[i]=0;
            BodyAcc_z[i]=0;
            AdVec_x[i]=0; 
            AdVec_vx[i]=0; 
            AdVec_ax[i]=0; 
            AdVec_jx[i]=0; 
            AdVec_z[i]=0; 
            AdVec_vz[i]=0; 
            AdVec_az[i]=0; 
            AdVec_jz[i]=0; 
            BodyPos_x[i]=0; 
            BodyVel_x[i]=0;
            BodyAcc_x[i]=0;
            BodyJerk_x[i]=0;
            BodySnap_x[i]=0;
            BodyPos_z[i]=0; 
            BodyVel_z[i]=0;
            BodyAcc_z[i]=0;
            BodyJerk_z[i]=0;
            BodySnap_z[i]=0;
        }  
        BodyPos_x[0]=BodyTransform.position.x; 
        BodyPos_z[0]=BodyTransform.position.z; 
        BodyVel_x[0]=0;
        BodyVel_z[0]=0;
        BodyAcc_x[0]=0;
        BodyAcc_z[0]=0;
        BodyJerk_x[0]=0;
        BodyJerk_z[0]=0;
        BodySnap_x[0]=0;
        BodySnap_z[0]=0;
        DiffBodySnap_x[0]=0;
        DiffBodySnap_z[0]=0;
        PreBodyPos_x=BodyPos_x[0];  
        PreBodyPos_z=BodyPos_z[0]; 
        PreBodyVel_x=0;
        PreBodyVel_z=0;
        PreBodyAcc_x=0;
        PreBodyAcc_z=0;
        InitialTime=Time.time;
    }

    

    // Update is called once per frame
    void FixedUpdate(){
        //try{
        PosRef_x=makeTargetPoint.TargetPoint.x;
        PosRef_z=makeTargetPoint.TargetPoint.y;
        // モデルへの入力はnaturalクラスでおこなっている

        if(MPC_mode){

            //measure real delta time (not evaluation delta time)
            //制御ループ周期(dt)測定
            float dt=Time.deltaTime; 

            //meature present Object's Pos and velocity and input them into ObjectPos[0],ObjectVelocity[0]
            //目標物の位置と速度を計測し、x[τ=0],v[τ=0]に代入する
            BodyPos_x[0]=BodyTransform.position.x; 
            BodyPos_z[0]=BodyTransform.position.z; 
            BodyVel_x[0]=(BodyPos_x[0]-PreBodyPos_x)/dt;//-dt/2*PreBodyAcc_x; 
            BodyVel_z[0]=(BodyPos_z[0]-PreBodyPos_z)/dt;//-dt/2*PreBodyAcc_z;
            BodyAcc_x[0]=(BodyVel_x[0]-PreBodyVel_x)/dt;
            BodyAcc_z[0]=(BodyVel_z[0]-PreBodyVel_z)/dt;
            BodyJerk_x[0]=(BodyAcc_x[0]-PreBodyAcc_x)/dt;
            BodyJerk_z[0]=(BodyAcc_z[0]-PreBodyAcc_z)/dt;//////////////////////////////////////////////////////////////////
            PreBodyPos_x=BodyPos_x[0]; 
            PreBodyPos_z=BodyPos_z[0];
            PreBodyVel_x=BodyVel_x[0];
            PreBodyVel_z=BodyVel_z[0];
            PreBodyAcc_x=BodyAcc_x[0];
            PreBodyAcc_z=BodyAcc_z[0];

            //difine EvaluationTime in this loop
            //EvalutionTime will converge to FinalEvaluationScope/PredictionTime
            //予測範囲を予測範囲分割数で割った予測空間内でのループ周期(dτ)を設定する
            //予め設定した最終予測範囲に収束するように設定する：ループ開始時は予測の精度が高くないため予測範囲は初めは0で0→最終予測範囲となるように
            float EvalDT=FinalEvaluarionScope*(1-Mathf.Exp(-2.5f*(Time.time-InitialTime+0.1f)))/PredictionTime; 

            //forsee ObjectPos[i] and ObjectVelocity[i] using ObjectPos[0] and ObjectVelocity[0], given InputPos[i]
            //上で与えられたv[τ=0],x[τ=0](つまりv[t],x[t])とuからx[i],v[i]を順に予想していく
            for(int i=1;i<PredictionTime+1; i++) {
                BodyPos_x[i]=BodyPos_x[i-1]+BodyVel_x[i-1]*EvalDT;//+BodyAcc_x[i-1]*EvalDT*EvalDT/2+BodyJerk_x[i-1]*EvalDT*EvalDT*EvalDT/6+BodySnap_x[i-1]*EvalDT*EvalDT*EvalDT*EvalDT/24;
                BodyPos_z[i]=BodyPos_z[i-1]+BodyVel_z[i-1]*EvalDT;//+BodyAcc_z[i-1]*EvalDT*EvalDT/2+BodyJerk_z[i-1]*EvalDT*EvalDT*EvalDT/6+BodySnap_z[i-1]*EvalDT*EvalDT*EvalDT*EvalDT/24;
                BodyVel_x[i]=BodyVel_x[i-1]+BodyAcc_x[i-1]*EvalDT;//+BodyJerk_x[i-1]*EvalDT*EvalDT/2+BodySnap_x[i-1]*EvalDT*EvalDT*EvalDT/6;
                BodyVel_z[i]=BodyVel_z[i-1]+BodyAcc_z[i-1]*EvalDT;//+BodyJerk_z[i-1]*EvalDT*EvalDT/2+BodySnap_z[i-1]*EvalDT*EvalDT*EvalDT/6;
                BodyAcc_x[i]=BodyAcc_x[i-1]+BodyJerk_x[i-1]*EvalDT;//+BodySnap_x[i-1]*EvalDT*EvalDT/2;
                BodyAcc_z[i]=BodyAcc_z[i-1]+BodyJerk_z[i-1]*EvalDT;//+BodySnap_z[i-1]*EvalDT*EvalDT/2;
                BodyJerk_x[i]=BodyJerk_x[i-1]+BodySnap_x[i-1]*EvalDT;
                BodyJerk_z[i]=BodyJerk_z[i-1]+BodySnap_z[i-1]*EvalDT;
            }


            //calculate AdVec[i,0]and[i,1] :[i,0] for Pos, [i,1] for velocity
            //随伴変数を求める
            //at first, AdVec[last] is calculated by AdVec[last]=ð(TerminalCost)/ðx
            //初めに、予測範囲内で最終の随伴変数を求める、これは終端コストのx[N*dτ]での偏微分に等しい
            AdVec_x[PredictionTime]=XConstant*(BodyPos_x[PredictionTime] -PosRef_x); 
            AdVec_vx[PredictionTime]=VxCosntant*BodyVel_x[PredictionTime];
            AdVec_z[PredictionTime]=ZConstant*(BodyPos_z[PredictionTime] -PosRef_z); 
            AdVec_vz[PredictionTime]=VzConstant*BodyVel_z[PredictionTime];
            AdVec_ax[PredictionTime]=0;
            AdVec_az[PredictionTime]=0;
            AdVec_jx[PredictionTime]=0;
            AdVec_jz[PredictionTime]=0;

            //following AdVec[last], AdVec[last -1] can be calculated by AdVec[last -1]=AdVec[last]+ ðH/ðx*dτ, and so on.
            //逆順に随伴変数を求めていく。AdVec[i-1]=AdVec[i]+ ðH/ðx*dτのように求められる。
            for(int i=PredictionTime-1;i>0;i--){
                float AdXContent=XConstant_Stage*(BodyPos_x[i]-PosRef_x);
                float AdZContent=ZConstant_Stage*(BodyPos_z[i]-PosRef_z);
                float AdVxContent=VxConstant_Stage*BodyVel_x[i]+AdVec_x[i+1];
                float AdVzContent=VzConstant_Stage*BodyVel_z[i]+AdVec_z[i+1];
                float AdAxContent=AxConstant_Stage*BodyAcc_x[i]+AdVec_vx[i+1];
                float AdAzContent=AzConstant_Stage*BodyAcc_z[i]+AdVec_vz[i+1];
                float AdJxContent=JxConstant_Stage*BodyJerk_x[i]+AdVec_ax[i+1];
                float AdJzContent=JzConstant_Stage*BodyJerk_z[i]+AdVec_az[i+1];
                                         
                AdVec_x[i]=AdVec_x[i+1]+AdXContent*EvalDT;
                AdVec_vx[i]=AdVec_vx[i+1]+AdVxContent*EvalDT;
                AdVec_ax[i]=AdVec_ax[i+1]+AdAxContent*EvalDT;
                AdVec_jx[i]=AdVec_jx[i+1]+AdJxContent*EvalDT;
                AdVec_z[i]=AdVec_z[i+1]+AdZContent*EvalDT;
                AdVec_vz[i]=AdVec_vz[i+1]+AdVzContent*EvalDT;
                AdVec_az[i]=AdVec_az[i+1]+AdAzContent*EvalDT;
                AdVec_jz[i]=AdVec_jz[i+1]+AdJzContent*EvalDT;
            }


            //calculate dU/dt using GMRES method
            float[] Difference_SnapX=new float[PredictionTime];
            float[] Difference_SnapZ=new float[PredictionTime];
            float[] Difference=new float[PredictionTime*2];
            float DifferenceInnerProduct=0; 
            float[,] OrthogonalBasis=new float[GMRES_RepeatTime+1, PredictionTime*2]; 

            for(int i=0;i<PredictionTime;i++) {
                float F_SnapX(float bodySnap_x){
                    return SxConstant_Stage*bodySnap_x+AdVec_jx[i+1];
                }
                float F_SnapZ(float bodySnap_z){
                    return SzConstant_Stage*bodySnap_z+AdVec_jz[i+1];
                }
                Difference_SnapX[i]=-StableConstant*F_SnapX(BodySnap_x[i])-(F_SnapX(BodySnap_x[i]+DiffBodySnap_x[i]*EvalDT)-F_SnapX(BodySnap_x[i]))/EvalDT; 
                Difference_SnapZ[i]=-StableConstant*F_SnapZ(BodySnap_z[i])-(F_SnapZ(BodySnap_z[i]+DiffBodySnap_z[i]*EvalDT)-F_SnapZ(BodySnap_z[i]))/EvalDT; 
                Difference[i*2]=Difference_SnapX[i];
                Difference[i*2+1]=Difference_SnapZ[i];
                DifferenceInnerProduct+=(Mathf.Pow(Difference_SnapX[i], 2)+Mathf.Pow(Difference_SnapZ[i],2));//sqrt this later
            }
            DifferenceInnerProduct=Mathf.Sqrt(DifferenceInnerProduct);
            for(int i=0;i<PredictionTime*2; i++) OrthogonalBasis[0,i]=Difference[i]/DifferenceInnerProduct;

            float[,] h=new float[GMRES_RepeatTime+1, GMRES_RepeatTime];//gyo, retu 
            float[] y=new float[GMRES_RepeatTime]; 
            for(int i=0;i<GMRES_RepeatTime+1; i++){ 
                for(int j=0;j<GMRES_RepeatTime; j++){
                    h[i,j]=0;
                    y[j]=0;
                }
            }
            for(int i=0;i<GMRES_RepeatTime; i++){
                for(int j=0; j<PredictionTime; j++) {
                    float F_AccX(float bodySnap_x){
                        return SxConstant_Stage*bodySnap_x+AdVec_jx[j+1];
                    }
                    float F_AccZ(float bodySnap_z){
                        return SzConstant_Stage*bodySnap_z+AdVec_jz[j+1];
                    }

                    OrthogonalBasis[i+1,j*2]=(F_AccX(BodySnap_x[j]+OrthogonalBasis[i,j*2]*EvalDT)-F_AccX(BodySnap_x[j]))/EvalDT; 
                    OrthogonalBasis[i+1,j*2+1]=(F_AccZ(BodySnap_z[j]+OrthogonalBasis[i,j*2+1]*EvalDT)-F_AccZ(BodySnap_z[j]))/EvalDT; 
                }
                for(int j=0; j<i+1;j++){
                    for(int k=0;k<PredictionTime*2;k++) h[j,i]+=OrthogonalBasis[i+1,k]*OrthogonalBasis[j,k]; 
                    for(int k=0;k<PredictionTime*2;k++) OrthogonalBasis[i+1,k]=OrthogonalBasis[i+1,k]-h[j,i]*OrthogonalBasis[j,k];
                }
                for(int j=0; j<PredictionTime*2; j++)h[i+1,i]+=Mathf.Pow(OrthogonalBasis[i+1,j],2); //sqrt this later
                h[i+1,i]=Mathf.Sqrt(h[i+1,i]);
                for(int j=0; j<PredictionTime*2; j++) OrthogonalBasis[i+1,j]=OrthogonalBasis[i+1,j]/h[i+1,i];
            }


            /*
            ここまでの計算により
                {1  {h00,h01  {y0
            |r|× 0 = h10,h11 × y1}
                0}   0 ,h21}  
            これをGives回転を用いて右辺のヘッセンベルグ行列を上三角行列にする
            */
            float[] GivensColumn_1=new float[GMRES_RepeatTime+1];
            for(int i=0;i<GMRES_RepeatTime+1; i++) GivensColumn_1[i]=1;

            for(int i=0;i<GMRES_RepeatTime; i++){
                float r=Mathf.Sqrt(Mathf.Pow(h[i+1,i],2)+Mathf.Pow(h[i,i],2));
                float SinTheta=h[i+1,i]/r; 
                float CosTheta=-h[i,i]/r;

                for(int j=0; j<GMRES_RepeatTime+1; j++){ 
                    for(int k=0; k<GMRES_RepeatTime;k++){ 
                        if(j==i)h[j,k]=h[j,k]*CosTheta-h[j+1,k]*SinTheta; 
                        else if(j==i+1 && k==i)h[j,k]=0; 
                        else if(j==i+1 && k>i)h[j,k]=h[j-1,k]*SinTheta+h[j,k]*CosTheta;
                    }
                    if(j==i)GivensColumn_1[j]*=CosTheta;
                    else if(j>i)GivensColumn_1[j]*=SinTheta;
                }
            }

            /*
            calculate y from↓
                {G0  {h00,h01  {y0
            |r|× G1}=  0 ,h11 × y1}
                 0 }   0 , 0 }   
            */

            for(int i=GMRES_RepeatTime-1;i>0-1;i--) {
                float DevidedValue=GivensColumn_1[i]*DifferenceInnerProduct;
                for(int j=GMRES_RepeatTime-1;j>i;j--) DevidedValue-=h[i,j]*y[j];
                y[i]=DevidedValue/h[i,i];
            }

            //calculate U by U=(Pre)U+dU/dt*dt
            for(int i=0;i<PredictionTime*2;i++){
                for(int j=0;j<GMRES_RepeatTime;j++){
                    if(i%2==0)DiffBodySnap_x[i/2]+=OrthogonalBasis[j,i]*y[j];
                    if(i%2==1)DiffBodySnap_z[(i-1)/2]+=OrthogonalBasis[j,i]*y[j];
                }
                if(i%2==0)BodySnap_x[i/2]+=DiffBodySnap_x[i/2]*EvalDT;
                if(i%2==1)BodySnap_z[(i-1)/2]+=DiffBodySnap_z[(i-1)/2]*EvalDT;
            }


            BodyJerk_x[0]+=BodySnap_x[0]*dt;
            BodyJerk_z[0]+=BodySnap_z[0]*dt;
            BodyAcc_x[0]+=BodyJerk_x[0]*dt;//+BodySnap_x[0]*dt*dt/2;
            BodyAcc_z[0]+=BodyJerk_z[0]*dt;//+BodySnap_z[0]*dt*dt/2;
            BodyVel_x[0]+=BodyAcc_x[0]*dt;//+BodyJerk_x[0]*dt*dt/2+BodySnap_x[0]*dt*dt*dt/6;
            BodyVel_z[0]+=BodyAcc_z[0]*dt;//+BodyJerk_z[0]*dt*dt/2+BodySnap_z[0]*dt*dt*dt/6;
            BodyPos_x[0]+=BodyVel_x[0]*dt;//+BodyAcc_x[0]*dt*dt/2+BodyJerk_x[0]*dt*dt*dt/6+BodySnap_x[0]*dt*dt*dt*dt/24;
            BodyPos_z[0]+=BodyVel_z[0]*dt;//+BodyAcc_z[0]*dt*dt/2+BodyJerk_z[0]*dt*dt*dt/6+BodySnap_z[0]*dt*dt*dt*dt/24;
            Debug.Log(BodyVel_x[0]+":"+BodyVel_z[0]);

                
            float TargetDiff=Mathf.Pow(PosRef_x-BodyPos_x[0],2)+Mathf.Pow(PosRef_z-BodyPos_z[0],2)
                                +Mathf.Pow(BodyVel_x[0],2)+Mathf.Pow(BodyVel_z[0],2);

            if(TargetDiff<0.01f) isCloseToTarget=true;
        }
        //}catch{
        //    Debug.Log("飛ばしたで");
        //}  
    }
}