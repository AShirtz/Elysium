using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 *      This class controls the a flock of 'navigation boids' in the following ways:
 *          - Sets Target for flock
 *          - Retrieves information about the flock
 *          
 *      NOTE: This class does not interpret the information about the flock, that is done 'another level up'.
 *      
 *      TODO:
 *          Implement control of boids (delivery of FixedUpdate)
 *          Make number of boids be controllable (simply deactivates extra boids)
 *          
 */
public class FlockController : MonoBehaviour
{
    [Header("Flock Parameters")]

    [Tooltip("Number of boids in this flock. Must not be changed during gameplay.")]
    public int numBoids = 50;

    [Tooltip("How many boids can act on FixedUpdate. Must be coprime with numBoids")]  // Expected to be changed by an overall controller according to performance and number of active flocks
    public int numBoidsPerUpdate = 11;

    [Tooltip("Prefab for the boid. Must be configured independently.")]
    public GameObject boidPrefab = null;


    public Vector3 targetPos = Vector3.zero;

    public Vector3 vel
    {
        get { return this.rgdbdy.velocity; }
    }


    private int curBoidIndex = 0;

    private Rigidbody rgdbdy = null;

    private List<BoidController> boids = null;

    void Start()
    {
        this.rgdbdy = this.GetComponent<Rigidbody>();

        // Create boids
        this.boids = new List<BoidController>(this.numBoids);

        for (int i = 0; i < this.numBoids; i++)
        {
            GameObject bd = GameObject.Instantiate(this.boidPrefab);

            BoidController bdCnt = bd.GetComponent<BoidController>();
            bdCnt.associatedFlockController = this;

            this.boids.Add(bdCnt);
        }
    }

    private void FixedUpdate()
    {
        // Calculate acceleration for some boids this tick
        for (int i = 0; i < this.numBoidsPerUpdate; i++)
        {
            this.boids[this.curBoidIndex].CalculateAccel();
            this.boids[this.curBoidIndex].MoveBoid();

            this.curBoidIndex = (this.curBoidIndex + 1) % this.boids.Count;
        }

        // Update positions for the rest of the boids
        for (int i = 0; i < (this.numBoids - this.numBoidsPerUpdate); i++)
        {
            this.boids[(this.curBoidIndex + i) % this.boids.Count].MoveBoid();
        }
    }
}
