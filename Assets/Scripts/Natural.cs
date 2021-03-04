using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Natural : MonoBehaviour
{   
    public int ControlMode;
    public Transform BodyTransform;
    public MPControl mPControl;
    public MPControl2 mPControl2;
    public GameObject PredictivePositionIndicaterSample;
    public GameObject PredictiveObsPosIndicatorSam;
    public Rigidbody RBody;
    public Transform Prop1;
    public Transform Prop2;
    public Transform Prop3;
    public Transform Prop4;
    GameObject[] PredictivePositionIndicater;
    GameObject[] TrajectoryIndicater;
    Transform[] PredictivePositionIndicaterTransform;
    Vector3[] PositionIndicaterPosition;
    Transform[,] PredictiveObsPosIndicatorTransform;
    int PredictionTime;
    LineRenderer lineRenderer;
    LineRenderer lineRendererForTra;
    int NumOfTrajectoryPoint;
    bool haventMade=true;
    float M=0.3f;
    float G=9.81f;

    void Start(){
        if(ControlMode==1)PredictionTime=mPControl.PredictionTime;
        else if(ControlMode==2)PredictionTime=mPControl2.PredictionTime;
        PositionIndicaterPosition=new Vector3[PredictionTime];
        PredictivePositionIndicater=new GameObject[PredictionTime+1];
        PredictivePositionIndicaterTransform=new Transform[PredictionTime+1]; 
        if(ControlMode==2)
            PredictiveObsPosIndicatorTransform=
                    new Transform[mPControl2.ObstacleMaxNum,PredictionTime+1]; 

        for(int i=0;i<PredictionTime+1;i++){
            PredictivePositionIndicaterTransform[i]=Instantiate(PredictivePositionIndicaterSample).GetComponent<Transform>();
            if(ControlMode==2){
                for(int j=0;j<mPControl2.ObstacleMaxNum;j++)PredictiveObsPosIndicatorTransform[j,i]=Instantiate(PredictiveObsPosIndicatorSam).GetComponent<Transform>();
            }
        }
        lineRenderer=gameObject.AddComponent<LineRenderer>();
        lineRenderer.positionCount=PredictionTime;
        lineRenderer.widthMultiplier=0.02f;
    }

    void FixedUpdate() 
    {
        float dt=Time.deltaTime;
        if(ControlMode==1){
            lineRenderer.SetPosition(0,new Vector3(mPControl.BodyPos_x[0],0.1f,mPControl.BodyPos_z[0]));

            for(int i=1;i<PredictionTime;i++){
                PredictivePositionIndicaterTransform[i].position=new Vector3(mPControl.BodyPos_x[i],0.1f,mPControl.BodyPos_z[i]);
                PositionIndicaterPosition[i]=new Vector3(mPControl.BodyPos_x[i],0.1f,mPControl.BodyPos_z[i]);
                lineRenderer.SetPosition(i,PositionIndicaterPosition[i]);
            }
            Prop1.localEulerAngles+=new Vector3(0,100,0);
            Prop2.localEulerAngles+=new Vector3(0,100,0);
            Prop3.localEulerAngles+=new Vector3(0,-100,0);
            Prop4.localEulerAngles+=new Vector3(0,-100,0);
            BodyTransform.position=new Vector3(mPControl.BodyPos_x[0],0,mPControl.BodyPos_z[0]);
            BodyTransform.eulerAngles=new Vector3(Mathf.Atan2(mPControl.BodyAcc_z[0],M*G)*Mathf.Rad2Deg,0,-Mathf.Atan2(mPControl.BodyAcc_x[0],M*G)*Mathf.Rad2Deg);
        }else if(ControlMode==2){
            lineRenderer.SetPosition(0,new Vector3(mPControl2.BodyPos_x[0],0.1f,mPControl2.BodyPos_z[0]));

            for(int i=1;i<PredictionTime;i++){
                PredictivePositionIndicaterTransform[i].position=new Vector3(mPControl2.BodyPos_x[i],0.1f,mPControl2.BodyPos_z[i]);
                /*for(int j=0;j<mPControl2.ObstacleMaxNum;j++){
                    PredictiveObsPosIndicatorTransform[j,i].position=new Vector3(9,1.5f,10);
                }
                for(int j=0;j<mPControl2.CurObsNum;j++)
                        PredictiveObsPosIndicatorTransform[j,i].position=new Vector3(mPControl2.ObstaclePos_x[j,i],1.5f,mPControl2.ObstaclePos_z[j,i]);*/
                PositionIndicaterPosition[i]=new Vector3(mPControl2.BodyPos_x[i],0.1f,mPControl2.BodyPos_z[i]);
                lineRenderer.SetPosition(i,PositionIndicaterPosition[i]);
            }
            Prop1.localEulerAngles+=new Vector3(0,100,0);
            Prop2.localEulerAngles+=new Vector3(0,100,0);
            Prop3.localEulerAngles+=new Vector3(0,-100,0);
            Prop4.localEulerAngles+=new Vector3(0,-100,0);
            BodyTransform.position=new Vector3(mPControl2.BodyPos_x[0],0,mPControl2.BodyPos_z[0]);
            BodyTransform.eulerAngles=new Vector3(Mathf.Atan2(mPControl2.BodyAcc_z[0],M*G)*Mathf.Rad2Deg,0,-Mathf.Atan2(mPControl2.BodyAcc_x[0],M*G)*Mathf.Rad2Deg);
        }
        
    }//Update
}//class
