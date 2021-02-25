using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MakeTargetPoint : MonoBehaviour
{
    public bool AutoMakePoint=false;
    public float x_limit=7;
    public float z_limit=7;
    public Vector2 TargetPoint=new Vector2(2,2);
    public Transform BodyTransform;
    public Transform TargetPointIndicater;
    int AchieveTime=0;
    // Start is called before the first frame update
    void Start()
    {
        //TargetPointIndicater.transform.position=new Vector3(TargetPoint.x,0,TargetPoint.y);
    }

    // Update is called once per frame
    void Update()
    {
        if(AutoMakePoint){
            Vector2 CurPosition=new Vector2(BodyTransform.position.x,BodyTransform.position.z);
            if(Vector2.Distance(CurPosition,TargetPoint)<0.3f){
                float Target_x=Random.Range(-x_limit,x_limit);
                float Target_z=Random.Range(z_limit*(-1),z_limit*1);
                TargetPoint=new Vector2(Target_x,Target_z);
                TargetPointIndicater.transform.position=new Vector3(Target_x,0,Target_z);
            }
            //Debug.Log(TargetP
        }else{
            TargetPoint=new Vector2(TargetPointIndicater.transform.position.x,TargetPointIndicater.transform.position.z);
        }
    }
}