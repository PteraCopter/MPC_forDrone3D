using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveObstacle : MonoBehaviour
{
    public Transform MovingObstacle0;
    public Transform MovingObstacle1;
    public Transform MovingObstacle2;
    public Transform MovingObstacle3;
    public Transform MovingObstacle4;
    bool UeMode=true;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        /*
        if(UeMode) MovingObstacle.position+=new Vector3(0,0,0.02f);
        else MovingObstacle.position-=new Vector3(0,0,0.02f);
        if(MovingObstacle.position.z>8)UeMode=false;
        if(MovingObstacle.position.z<-8)UeMode=true;
        */
        MovingObstacle0.eulerAngles+=new Vector3(0,0.15f,0);
        MovingObstacle1.eulerAngles+=new Vector3(0,-0.1f,0);
        MovingObstacle2.eulerAngles+=new Vector3(0,0.15f,0);
        MovingObstacle3.eulerAngles+=new Vector3(0,0.05f,0);
        MovingObstacle4.eulerAngles+=new Vector3(0,-0.05f,0);

    }
}
