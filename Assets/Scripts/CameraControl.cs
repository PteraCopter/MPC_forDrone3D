using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraControl : MonoBehaviour
{
    public MPControl mPControl;
    public GameObject Cam;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Cam.transform.eulerAngles=new Vector3(0,Mathf.Atan2(mPControl.BodyVel_x[0],mPControl.BodyVel_z[0])*Mathf.Rad2Deg,0);
    }
}
