using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Natural : MonoBehaviour
{   
    public Transform BodyTransform;
    public MPControl mPControl;
    public GameObject PredictivePositionIndicaterSample;
    public Rigidbody RBody;
    public Transform Pre1;
    public Transform Pre2;
    public Transform Pre3;
    public Transform Pre4;
    GameObject[] PredictivePositionIndicater;
    GameObject[] TrajectoryIndicater;
    Transform[] PredictivePositionIndicaterTransform;
    Vector3[] PositionIndicaterPosition;
    int PredictionTime;
    LineRenderer lineRenderer;
    LineRenderer lineRendererForTra;
    int NumOfTrajectoryPoint;
    bool haventMade=true;
    float M=0.3f;
    float G=9.81f;

    void Start(){
        PredictionTime=mPControl.PredictionTime;
        PositionIndicaterPosition=new Vector3[PredictionTime];
        PredictivePositionIndicater=new GameObject[PredictionTime+1];
        PredictivePositionIndicaterTransform=new Transform[PredictionTime+1]; 

        for(int i=0;i<PredictionTime+1;i++){
            PredictivePositionIndicater[i]=Instantiate(PredictivePositionIndicaterSample); 
            PredictivePositionIndicaterTransform[i]=PredictivePositionIndicater[i].GetComponent<Transform>();
        }
        lineRenderer=gameObject.AddComponent<LineRenderer>();
        lineRenderer.positionCount=PredictionTime;
        lineRenderer.widthMultiplier=0.02f;
    }

    void FixedUpdate() 
    {
        /*if(mPControl.isCloseToTarget && haventMade){
            Destroy(lineRenderer);
            NumOfTrajectoryPoint=mPControl.BodyPosXForTrajectory.Count;
            TrajectoryIndicater=new GameObject[NumOfTrajectoryPoint];
            lineRendererForTra=gameObject.AddComponent<LineRenderer>();
            lineRendererForTra.positionCount=NumOfTrajectoryPoint;
            lineRendererForTra.widthMultiplier=0.05f;
            float[]  BodyPosXForTra=mPControl.BodyPosXForTrajectory.ToArray();
            float[]  BodyPosZForTra=mPControl.BodyPosZForTrajectory.ToArray();
            float[]  BodyVelXForTra=mPControl.BodyVelXForTrajectory.ToArray();
            float[]  BodyVelZForTra=mPControl.BodyVelZForTrajectory.ToArray();
            for(int i=0;i<NumOfTrajectoryPoint;i++){
                TrajectoryIndicater[i]=Instantiate(PredictivePositionIndicaterSample); 
                TrajectoryIndicater[i].transform.position=new Vector3(BodyPosXForTra[i],0.1f,BodyPosZForTra[i]);
                lineRendererForTra.SetPosition(i,TrajectoryIndicater[i].transform.position);
            }
            BodyTransform.position=new Vector3(BodyPosXForTra[0],0.5f,BodyPosZForTra[0]);
            haventMade=false;
        }else if(haventMade){*/
        float dt=Time.deltaTime;
        lineRenderer.SetPosition(0,new Vector3(mPControl.BodyPos_x[0],0.1f,mPControl.BodyPos_z[0]));

        for(int i=1;i<PredictionTime;i++){
            PredictivePositionIndicaterTransform[i].position=new Vector3(mPControl.BodyPos_x[i],0.1f,mPControl.BodyPos_z[i]);
            PositionIndicaterPosition[i]=new Vector3(mPControl.BodyPos_x[i],0.1f,mPControl.BodyPos_z[i]);
            lineRenderer.SetPosition(i,PositionIndicaterPosition[i]);
        }
        Pre1.localEulerAngles+=new Vector3(0,100,0);
        Pre2.localEulerAngles+=new Vector3(0,100,0);
        Pre3.localEulerAngles+=new Vector3(0,-100,0);
        Pre4.localEulerAngles+=new Vector3(0,-100,0);
        BodyTransform.position=new Vector3(mPControl.BodyPos_x[0],0,mPControl.BodyPos_z[0]);
        BodyTransform.eulerAngles=new Vector3(Mathf.Atan2(mPControl.BodyAcc_z[0],M*G)*Mathf.Rad2Deg,0,-Mathf.Atan2(mPControl.BodyAcc_x[0],M*G)*Mathf.Rad2Deg);
    }
}
