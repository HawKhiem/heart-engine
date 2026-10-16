using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
using Random = UnityEngine.Random;

public class WalkerAgent : Agent
{
    [Header("Walk Speed")]
    [Range(0.1f, 11)]
    [SerializeField]
    //The walking speed to try and achieve
    private float m_TargetWalkingSpeed = 11;

    public float MTargetWalkingSpeed // property
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, 0.1f, m_maxWalkingSpeed); }
    }

    const float m_maxWalkingSpeed = 11; //The max walking speed

    //Should the agent sample a new goal velocity each episode?
    public bool randomizeWalkSpeedEachEpisode;

    //The direction an agent will walk during training.
    private Vector3 m_WorldDirToWalk = Vector3.right;

    [Header("Target To Walk Towards")] public Transform target; //Target the agent will walk towards during training.

    [Header("Body Parts")] public Transform hips;
    public Transform chest;
    public Transform spine;
    public Transform head;
    public Transform thighL;
    public Transform shinL;
    public Transform footL;
    public Transform thighR;
    public Transform shinR;
    public Transform footR;
    public Transform armL;
    public Transform forearmL;
    public Transform handL;
    public Transform armR;
    public Transform forearmR;
    public Transform handR;

    // === Fixed “gift-carry” pose ===
    [Header("Fixed Gift-Carry Pose (normalized [-1,1])")]
    [Tooltip("If on, arms/forearms/hands are forced to this pose every step.")]
    public bool lockArmsAndHands = true;

    [Tooltip("Shoulder flexion (X) ~ raise/lower arms. 0.10–0.20 looks natural.")]
    [Range(-1f, 1f)] public float shoulderX = 0.15f;
    [Tooltip("Shoulder adduction (Y) toward chest. Negative on left, positive on right.")]
    [Range(-1f, 1f)] public float shoulderY = 0.15f;
    [Tooltip("Elbow bend (X). 0.5–0.7 ≈ right angle on most rigs.")]
    [Range(-1f, 1f)] public float elbowBend = 0.6f;   // NOTE: normalized, not degrees
    [Tooltip("Small wrist tweak if boxes tip. Keep 0 to stay upright.")]
    [Range(-1f, 1f)] public float handZTwist = 0f;
    [Tooltip("Strength used when locking arms (1 = max).")]
    [Range(0f, 1f)] public float lockedArmStrength = 1f;

    // === NEW: lock spine & chest ===
    [Header("Lock Torso (normalized [-1,1])")]
    [Tooltip("If on, spine and chest are held at these target rotations every step.")]
    public bool lockSpineAndChest = true;

    [Tooltip("Spine tilt (X) and yaw (Y). Keep small for upright look.")]
    [Range(-1f, 1f)] public float spineX = 0.0f;
    [Range(-1f, 1f)] public float spineY = 0.0f;

    [Tooltip("Chest tilt (X) and yaw (Y). Small forward tilt looks natural (e.g., 0.05).")]
    [Range(-1f, 1f)] public float chestX = 0.05f;
    [Range(-1f, 1f)] public float chestY = 0.0f;

    [Tooltip("Strength used when locking torso (1 = max).")]
    [Range(0f, 1f)] public float lockedTorsoStrength = 1f;

    // Stabilization helpers
    OrientationCubeController m_OrientationCube;
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;
    EnvironmentParameters m_ResetParams;

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

        //Setup each body part
        m_JdController = GetComponent<JointDriveController>();
        m_JdController.SetupBodyPart(hips);
        m_JdController.SetupBodyPart(chest);
        m_JdController.SetupBodyPart(spine);
        m_JdController.SetupBodyPart(head);
        m_JdController.SetupBodyPart(thighL);
        m_JdController.SetupBodyPart(shinL);
        m_JdController.SetupBodyPart(footL);
        m_JdController.SetupBodyPart(thighR);
        m_JdController.SetupBodyPart(shinR);
        m_JdController.SetupBodyPart(footR);
        m_JdController.SetupBodyPart(armL);
        m_JdController.SetupBodyPart(forearmL);
        m_JdController.SetupBodyPart(handL);
        m_JdController.SetupBodyPart(armR);
        m_JdController.SetupBodyPart(forearmR);
        m_JdController.SetupBodyPart(handR);

        m_ResetParams = Academy.Instance.EnvironmentParameters;
    }

    public override void OnEpisodeBegin()
    {
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        hips.rotation = Quaternion.Euler(0, 0, 0);

        UpdateOrientationObjects();

        MTargetWalkingSpeed =
            randomizeWalkSpeedEachEpisode ? Random.Range(.1f, m_maxWalkingSpeed) : MTargetWalkingSpeed;
    }

    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        sensor.AddObservation(bp.groundContact.touchingGround);
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.linearVelocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.position - hips.position));

        if (bp.rb.transform != hips && bp.rb.transform != handL && bp.rb.transform != handR)
        {
            sensor.AddObservation(bp.rb.transform.localRotation);
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = m_OrientationCube.transform.forward;

        var velGoal = cubeForward * MTargetWalkingSpeed;
        var avgVel = GetAvgVelocity();

        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));

        sensor.AddObservation(Quaternion.FromToRotation(hips.forward, cubeForward));
        sensor.AddObservation(Quaternion.FromToRotation(head.forward, cubeForward));

        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(target.transform.position));

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var bpDict = m_JdController.bodyPartsDict;
        var i = -1;

        var continuousActions = actionBuffers.ContinuousActions;
        bpDict[chest].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[spine].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        bpDict[thighL].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[thighR].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[shinL].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[shinR].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[footR].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[footL].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        // Shoulders, elbows, head (will override below if locking enabled)
        bpDict[armL].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[armR].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[forearmL].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[forearmR].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[head].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        // update joint strength settings
        bpDict[chest].SetJointStrength(continuousActions[++i]);
        bpDict[spine].SetJointStrength(continuousActions[++i]);
        bpDict[head].SetJointStrength(continuousActions[++i]);
        bpDict[thighL].SetJointStrength(continuousActions[++i]);
        bpDict[shinL].SetJointStrength(continuousActions[++i]);
        bpDict[footL].SetJointStrength(continuousActions[++i]);
        bpDict[thighR].SetJointStrength(continuousActions[++i]);
        bpDict[shinR].SetJointStrength(continuousActions[++i]);
        bpDict[footR].SetJointStrength(continuousActions[++i]);
        bpDict[armL].SetJointStrength(continuousActions[++i]);
        bpDict[forearmL].SetJointStrength(continuousActions[++i]);
        bpDict[armR].SetJointStrength(continuousActions[++i]);
        bpDict[forearmR].SetJointStrength(continuousActions[++i]);

        // ===== OVERRIDE: lock arms/elbows/hands to fixed pose =====
        if (lockArmsAndHands)
        {
            bpDict[armL].SetJointTargetRotation(shoulderX, -Mathf.Abs(shoulderY), 0f);
            bpDict[armR].SetJointTargetRotation(shoulderX, Mathf.Abs(shoulderY), 0f);

            bpDict[forearmL].SetJointTargetRotation(elbowBend, 0f, 0f);
            bpDict[forearmR].SetJointTargetRotation(elbowBend, 0f, 0f);

            bpDict[handL].SetJointTargetRotation(0f, 0f, -handZTwist);
            bpDict[handR].SetJointTargetRotation(0f, 0f, handZTwist);

            bpDict[armL].SetJointStrength(lockedArmStrength);
            bpDict[forearmL].SetJointStrength(lockedArmStrength);
            bpDict[handL].SetJointStrength(lockedArmStrength);
            bpDict[armR].SetJointStrength(lockedArmStrength);
            bpDict[forearmR].SetJointStrength(lockedArmStrength);
            bpDict[handR].SetJointStrength(lockedArmStrength);
        }

        // ===== OVERRIDE: lock spine & chest =====
        if (lockSpineAndChest)
        {
            // Keep torso upright with a slight forward lean if desired.
            bpDict[spine].SetJointTargetRotation(spineX, spineY, 0f);
            bpDict[chest].SetJointTargetRotation(chestX, chestY, 0f);

            bpDict[spine].SetJointStrength(lockedTorsoStrength);
            bpDict[chest].SetJointStrength(lockedTorsoStrength);
        }
    }

    //Update OrientationCube and DirectionIndicator
    void UpdateOrientationObjects()
    {
        m_WorldDirToWalk = target.position - hips.position;
        m_OrientationCube.UpdateOrientation(hips, target);
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        var cubeForward = m_OrientationCube.transform.forward;

        // a. Match target speed
        var matchSpeedReward = GetMatchingVelocityReward(cubeForward * MTargetWalkingSpeed, GetAvgVelocity());

        if (float.IsNaN(matchSpeedReward))
        {
            throw new ArgumentException(
                "NaN in moveTowardsTargetReward.\n" +
                $" cubeForward: {cubeForward}\n" +
                $" hips.velocity: {m_JdController.bodyPartsDict[hips].rb.linearVelocity}\n" +
                $" maximumWalkingSpeed: {m_maxWalkingSpeed}"
            );
        }

        // b. Face target (horizontal)
        var headForward = head.forward; headForward.y = 0;
        var lookAtTargetReward = (Vector3.Dot(cubeForward, headForward) + 1) * .5F;

        if (float.IsNaN(lookAtTargetReward))
        {
            throw new ArgumentException(
                "NaN in lookAtTargetReward.\n" +
                $" cubeForward: {cubeForward}\n" +
                $" head.forward: {head.forward}"
            );
        }

        AddReward(matchSpeedReward * lookAtTargetReward);
    }

    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;

        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.linearVelocity;
        }

        var avgVel = velSum / numOfRb;
        return avgVel;
    }

    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, MTargetWalkingSpeed);
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / MTargetWalkingSpeed, 2), 2);
    }

    public void TouchedTarget()
    {
        AddReward(1f);
    }
}
