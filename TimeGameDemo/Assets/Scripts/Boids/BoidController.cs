using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/*
 *      This class implements the controller for the individual boids in a navigation flock.
 *      
 *      TODO: 
 *          Implement the boid reset function
 *          Determine if boids should be affected by time dilation (probably, but may need to be a part of a class hierarchy?)
 *          Implement association with FlockController (should the FlockController determine which boids get to act when?)
 */
public class BoidController : MonoBehaviour
{
    private static int neighborSearchLayerMask = -1;

    [Header("General Boid Parameters")]

    [Tooltip("Asymptotic smoothing factor for accel direction. Is affected by accelDir recalculation schedule/timing.")]
    public float dirSmoothing = 0.2f;

    [Tooltip("Used to determine strength of random component in initial velocity. Multiplicative factor wrt velocity magnitude.")]
    public float initialVelocityRandomFactor = 0.25f;

    [Tooltip("Impacts the separation behavior. Boids that are stuck contribute more or less depending on this factor. Multiplicative.")]
    public float stuckSeparationFactor = 1.5f;

    [Tooltip("Impacts the alignment behavior. Boids that are stuck contribute more or less depending on this factor. Multiplicative.")]
    public float stuckAlignmentFactor = 1.5f;

    [Tooltip("Maximum acceleration of the boid. Used in direction -> acceleration remap.")]
    public float maxAcceleration = 1f;

    [Tooltip("Minimum acceleration of the boid. Used in direction -> acceleration remap.")]
    public float minAcceleration = 0.15f;

    [Tooltip("Maps dot product (clamped [0,1]) to interp factor between max and min acceleration.")]
    public AnimationCurve directionAccelerationRemap = null;


    [Tooltip("Radius in which nearby boids are considered.")]
    public float neighborSearchRadius = 15f;



    [Header("Lifecycle Parameters and Weight Curves")]

    [Tooltip("Time in seconds of the lifecycle of the boid")]
    public float lifeCycleDuration = 10f;


    [Tooltip("Weight given to the separation behavior of the boid over the lifecycle of the boid.")]
    public AnimationCurve separationWeightFactor = null;

    [Tooltip("Weight given to the alignment behavior of the boid over the lifecycle of the boid.")]
    public AnimationCurve alignmentWeightFactor = null;

    [Tooltip("Weight given to the cohesion behavior of the boid over the lifecycle of the boid.")]
    public AnimationCurve cohesionWeightFactor = null;

    [Tooltip("Weight given to the target facing behavior of the boid over the lifecycle of the boid.")]
    public AnimationCurve targetWeightFactor = null;

    [Tooltip("Weight given to the random direction behavior of the boid over the lifecycle of the boid.")]
    public AnimationCurve randomDirectionWeighFactor = null;



    [Header("Boid State")]

    [Tooltip("Indicates whether this boid has collided with an obstacle.")]
    public bool isStuck = false;

    [Tooltip("The associated flock controller.")]
    public FlockController associatedFlockController = null;

    // The current 'forward' for this boid. Is the surface normal when the boid is stuck.
    public Vector3 forward
    {
        get
        {
            if (this.isStuck)
                return this.stuckNormal;
            else
                return this.rgdbdy.velocity.normalized;
        }
    }

    private float lastResetTime = 0f;

    private Vector3 accelDir = Vector3.zero;        // Determined by boid behavior, needs to be stored between frames it's calculated
    private Vector3 stuckNormal = Vector3.zero;     // Normal of collision, used when the boid is 'stuck'

    private Rigidbody rgdbdy = null;

    void Start()
    {
        if (neighborSearchLayerMask == -1)
            neighborSearchRadius = LayerMask.NameToLayer("NavigationAgent");

        this.rgdbdy = this.GetComponent<Rigidbody>();

        this.ResetBoid();
    }

    void UpdateBoid(bool calcAccel)
    {

        if (Time.time - this.lastResetTime >= this.lifeCycleDuration)
            this.ResetBoid();

        if (!this.isStuck && calcAccel)
            this.CalculateAccel();
        
        this.MoveBoid();
    }

    private void ResetBoid()
    {
        this.lastResetTime = Time.time;

        this.transform.position = this.associatedFlockController.transform.position;
        this.rgdbdy.velocity = this.associatedFlockController.vel + (Random.insideUnitSphere * this.initialVelocityRandomFactor);

        this.rgdbdy.constraints = RigidbodyConstraints.None;
    }

    public void CalculateAccel ()
    {
        // TODO: What to do when no neighbors are found? Should be specific to each behavior? Should that behavior have 0 impact on final accelDir?
        // TODO: Coalesce these into a single loop once behavior is finalized

        // Find all boids within the neighbor search radius
        Collider[] neighbors = Physics.OverlapSphere(this.transform.position, this.neighborSearchRadius, neighborSearchLayerMask);

        // Find weighted center of mass of all neighbors (includes boids from other flocks, for separation behavior)
        float divisor = 0f;
        Vector3 workspace = Vector3.zero;

        for (int i = 0; i < neighbors.Length; i++)
        {
            // Skip 'this' boid
            if (neighbors[i].gameObject == this.gameObject)
                continue;

            BoidController nghbr = neighbors[i].GetComponent<BoidController>();

            if (nghbr == null)
                continue;

            if (nghbr.isStuck)
            {
                divisor += this.stuckSeparationFactor;
                workspace += neighbors[i].transform.position * this.stuckSeparationFactor;
            }
            else
            {
                divisor += 1f;
                workspace += neighbors[i].transform.position;
            }
        }

        // Find separation direction
        Vector3 separationDirection = Vector3.zero;

        if (divisor > 0f)
            separationDirection = (this.transform.position - (workspace / divisor)).normalized;



        // Find weighted average of 'forward' for neighbors from this flock (for alignment behavior)
        divisor = 0f;
        workspace = Vector3.zero;

        for (int i = 0; i < neighbors.Length; i++)
        {
            // Skip 'this' boid
            if (neighbors[i].gameObject == this.gameObject)
                continue;

            BoidController nghbr = neighbors[i].GetComponent<BoidController>();

            if (nghbr == null)
                continue;

            // Skip boids not in this flock
            if (nghbr.associatedFlockController != this.associatedFlockController)
                continue;


            if (nghbr.isStuck)
            {
                divisor += this.stuckAlignmentFactor;
                workspace += nghbr.forward * this.stuckAlignmentFactor;
            }
            else
            {
                divisor += 1f;
                workspace += nghbr.forward;
            }
        }

        // Find alignment direction
        Vector3 alignmentDirection = Vector3.zero;

        if (divisor > 0f)
            alignmentDirection = workspace / divisor;



        // Find weighted center of mass for boids from this flock (does not include stuck boids, for cohesion behavior)
        divisor = 0f;
        workspace = Vector3.zero;

        for (int i = 0; i < neighbors.Length; i++)
        {
            // Skip 'this' boid
            if (neighbors[i].gameObject == this.gameObject)
                continue;

            BoidController nghbr = neighbors[i].GetComponent<BoidController>();

            if (nghbr == null)
                continue;

            // Skip boids not in this flock or that are stuck
            if (nghbr.associatedFlockController != this.associatedFlockController || nghbr.isStuck)
                continue;

            divisor += 1f;
            workspace += nghbr.transform.position;
        }

        // Find cohesion direction
        Vector3 cohesionDirection = Vector3.zero;

        if (divisor > 0f)
            cohesionDirection = (this.transform.position - workspace) / divisor;


        
        // Find Simple directions
        Vector3 targetDirection = this.transform.position - this.associatedFlockController.targetPos;
        Vector3 randomDirection = Random.onUnitSphere;

        // Create weighted average based on lifecycle
        float curLifeTime = (Time.time - this.lastResetTime) / this.lifeCycleDuration;
        float factor = 0f;
        divisor = 0f;
        workspace = Vector3.zero;

        // Separation
        factor = this.separationWeightFactor.Evaluate(curLifeTime);
        if (factor > 0f && separationDirection != Vector3.zero)
        {
            divisor += factor;
            workspace += separationDirection * factor;
        }

        // Alignment
        factor = this.alignmentWeightFactor.Evaluate(curLifeTime);
        if (factor > 0f && alignmentDirection != Vector3.zero)
        {
            divisor += factor;
            workspace += alignmentDirection * factor;
        }

        // Cohesion
        factor = this.cohesionWeightFactor.Evaluate(curLifeTime);
        if (factor > 0f && cohesionDirection != Vector3.zero)
        {
            divisor += factor;
            workspace += cohesionDirection * factor;
        }

        // Target
        factor = this.targetWeightFactor.Evaluate(curLifeTime);
        if (factor > 0f && targetDirection != Vector3.zero)
        {
            divisor += factor;
            workspace += targetDirection * factor;
        }

        // Random
        factor = this.randomDirectionWeighFactor.Evaluate(curLifeTime);
        if (factor > 0f && randomDirection != Vector3.zero)
        {
            divisor += factor;
            workspace += randomDirection * factor;
        }

        if (divisor > 0f)
            workspace /= divisor;
        else
        {
            Debug.LogError("Failure to calculate direction.");
            workspace = this.forward;
        }

        // Use asymptotic smoothing on the results
        this.accelDir = Vector3.Slerp(this.accelDir, workspace, this.dirSmoothing);
    }

    public void MoveBoid ()
    {
        // Use dot product between accelDir and current velocity to scale acceleration force (favor turning, but still accelerates in straight line)
        float accelFactor = Mathf.Lerp(this.maxAcceleration, this.minAcceleration, this.directionAccelerationRemap.Evaluate(Mathf.Clamp01(Vector3.Dot(this.forward, this.accelDir))));

        this.rgdbdy.AddForce(this.accelDir * accelFactor * Time.fixedDeltaTime, ForceMode.Acceleration);
    }

    private void OnCollisionEnter(Collision collision)
    {
        // NOTE: The Boids must be in the NaviagtionAgent layer so they only collide with the NavigationObstacle layer

        this.isStuck = true;
        this.rgdbdy.velocity = Vector3.zero;
        this.rgdbdy.constraints = RigidbodyConstraints.FreezePosition;
        this.stuckNormal = collision.GetContact(0).normal;
    }
}
