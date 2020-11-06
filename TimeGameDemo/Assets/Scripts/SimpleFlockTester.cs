using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimpleFlockTester : MonoBehaviour
{
    public GameObject target = null;

    void Update()
    {
        if (Input.GetKey(KeyCode.W))
        {
            this.transform.position += Vector3.forward;
        }

        if (Input.GetKey(KeyCode.A))
        {
            this.transform.position += Vector3.left;
        }

        if (Input.GetKey(KeyCode.S))
        {
            this.transform.position += Vector3.back;
        }

        if (Input.GetKey(KeyCode.D))
        {
            this.transform.position += Vector3.right;
        }


        if (Input.GetKey(KeyCode.UpArrow))
        {
            this.target.transform.position += Vector3.forward;
        }

        if (Input.GetKey(KeyCode.LeftArrow))
        {
            this.target.transform.position += Vector3.left;
        }

        if (Input.GetKey(KeyCode.RightArrow))
        {
            this.target.transform.position += Vector3.right;
        }

        if (Input.GetKey(KeyCode.DownArrow))
        {
            this.target.transform.position += Vector3.down;
        }
    }
}
