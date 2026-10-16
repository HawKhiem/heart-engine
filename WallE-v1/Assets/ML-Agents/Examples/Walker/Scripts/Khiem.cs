using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;

public class Khiem : Agent
{
    [Header("Stand / Lock")]
    [Tooltip("Lock the whole lower body so the agent stands still. Not affected by gravity.")]
    public bool lockLowerBody = true;

    [Tooltip("Light torso lock to keep upright while arms hold gifts.")]
    public bool lockSpineAndChest = true;
    [Range(-1f, 1f)] public float spineX = 0.03f;
    [Range(-1f, 1f)] public float spineY = 0.0f;
    [Range(-1f, 1f)] public float chestX = 0.04f;
    [Range(-1f, 1f)] public float chestY = 0.0f;
    [Range(0f, 1f)] public float lockedTorsoStrength = 1f;

    // Keep speed ≈ 0 so the built-in reward stays sane (we keep the fields for compatibility)
    [Header("Walk Speed (kept near-zero)")]
    [SerializeField, Range(0.0001f, 10f)] private float m_TargetWalkingSpeed = 0.0001f;
    public float MTargetWalkingSpeed { get => m_TargetWalkingSpeed; set => m_TargetWalkingSpeed = Mathf.Max(0.0001f, value); }
    const float m_maxWalkingSpeed = 10f;
    public bool randomizeWalkSpeedEachEpisode = false;

    // Gift-carry pose
    [Header("Fixed Gift-Carry Pose (normalized [-1,1])")]
    public bool lockArmsAndHands = true;
    [Tooltip("Shoulder flexion (X). 0.10–0.20 looks natural.")]
    [Range(-1f, 1f)] public float shoulderX = 0.15f;
    [Tooltip("Shoulder adduction (Y) toward chest. Negative on L, positive on R.")]
    [Range(-1f, 1f)] public float shoulderY = 0.15f;
    [Tooltip("Elbow bend (X). 0.5–0.7 ≈ right angle on most rigs.")]
    [Range(-1f, 1f)] public float elbowBend = 0.6f;
    [Tooltip("Small wrist tweak if boxes tip. Keep 0 to stay upright.")]
    [Range(-1f, 1f)] public float handZTwist = 0f;
    [Range(0f, 1f)] public float lockedArmStrength = 1f;

    [Header("Target / Helpers (not used for motion)")]
    public Transform target;

    [Header("Body Parts")]
    public Transform hips;
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

    // Controllers
    OrientationCubeController m_OrientationCube;
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;

    void SetKinematicFrozen(Rigidbody rb, bool frozen)
    {
        if (!rb) return;
        if (frozen)
        {
            rb.useGravity = false;
            rb.isKinematic = true;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.constraints = RigidbodyConstraints.FreezeAll;
        }
        else
        {
            rb.isKinematic = false;
            rb.useGravity = true;
            rb.constraints = RigidbodyConstraints.None;
        }
    }

    void ApplyLowerBodyLock(bool on)
    {
        // Lock pelvis & legs so they are not affected by gravity and won’t move.
        SetKinematicFrozen(m_JdController.bodyPartsDict[hips].rb, on);
        SetKinematicFrozen(m_JdController.bodyPartsDict[thighL].rb, on);
        SetKinematicFrozen(m_JdController.bodyPartsDict[shinL].rb, on);
        SetKinematicFrozen(m_JdController.bodyPartsDict[footL].rb, on);
        SetKinematicFrozen(m_JdController.bodyPartsDict[thighR].rb, on);
        SetKinematicFrozen(m_JdController.bodyPartsDict[shinR].rb, on);
        SetKinematicFrozen(m_JdController.bodyPartsDict[footR].rb, on);
        // Leave spine/chest/head/arms dynamic so they can pose.
    }

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

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

        // Lock lower body immediately
        if (lockLowerBody) ApplyLowerBodyLock(true);

        // Keep speed near-zero to avoid locomotion pressure
        m_TargetWalkingSpeed = 0.0001f;
    }

    public override void OnEpisodeBegin()
    {
        // No respawn / no randomization (intentionally empty).
        // Ensure locks remain if episodes are triggered.
        if (lockLowerBody) ApplyLowerBodyLock(true);
        UpdateOrientationObjects();
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
        if (target) sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(target.transform.position));

        foreach (var bp in m_JdController.bodyPartsList) CollectObservationBodyPart(bp, sensor);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var bp = m_JdController.bodyPartsDict;
        var a = actionBuffers.ContinuousActions;
        var i = -1;

        // Torso/head still receive policy actions (we override to lock after)
        bp[chest].SetJointTargetRotation(a[++i], a[++i], a[++i]);
        bp[spine].SetJointTargetRotation(a[++i], a[++i], a[++i]);
        bp[head].SetJointTargetRotation(a[++i], a[++i], 0);

        // Arms receive actions (overridden by gift pose below)
        bp[armL].SetJointTargetRotation(a[++i], a[++i], 0);
        bp[armR].SetJointTargetRotation(a[++i], a[++i], 0);
        bp[forearmL].SetJointTargetRotation(a[++i], 0, 0);
        bp[forearmR].SetJointTargetRotation(a[++i], 0, 0);

        // Strengths (torso + arms only; lower body is kinematic/frozen)
        bp[chest].SetJointStrength(a[++i]);
        bp[spine].SetJointStrength(a[++i]);
        bp[head].SetJointStrength(a[++i]);
        bp[armL].SetJointStrength(a[++i]);
        bp[forearmL].SetJointStrength(a[++i]);
        bp[armR].SetJointStrength(a[++i]);
        bp[forearmR].SetJointStrength(a[++i]);

        // ---- Gift-carry pose override (arms/hands) ----
        if (lockArmsAndHands)
        {
            bp[armL].SetJointTargetRotation(shoulderX, -Mathf.Abs(shoulderY), 0f);
            bp[armR].SetJointTargetRotation(shoulderX, Mathf.Abs(shoulderY), 0f);

            bp[forearmL].SetJointTargetRotation(elbowBend, 0f, 0f);
            bp[forearmR].SetJointTargetRotation(elbowBend, 0f, 0f);

            bp[handL].SetJointTargetRotation(0f, 0f, -handZTwist);
            bp[handR].SetJointTargetRotation(0f, 0f, handZTwist);

            bp[armL].SetJointStrength(lockedArmStrength);
            bp[forearmL].SetJointStrength(lockedArmStrength);
            bp[handL].SetJointStrength(lockedArmStrength);
            bp[armR].SetJointStrength(lockedArmStrength);
            bp[forearmR].SetJointStrength(lockedArmStrength);
            bp[handR].SetJointStrength(lockedArmStrength);
        }

        // ---- Upright torso lock ----
        if (lockSpineAndChest)
        {
            bp[spine].SetJointTargetRotation(spineX, spineY, 0f);
            bp[chest].SetJointTargetRotation(chestX, chestY, 0f);
            bp[spine].SetJointStrength(lockedTorsoStrength);
            bp[chest].SetJointStrength(lockedTorsoStrength);
        }
    }

    void UpdateOrientationObjects()
    {
        if (target) m_OrientationCube.UpdateOrientation(hips, target);
        if (m_DirectionIndicator) m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        // Keep rewards benign with near-zero speed; add upright bonus.
        var fwd = m_OrientationCube.transform.forward;
        var matchSpeed = GetMatchingVelocityReward(fwd * MTargetWalkingSpeed, GetAvgVelocity());
        var headForward = head.forward; headForward.y = 0;
        var look = (Vector3.Dot(fwd, headForward) + 1f) * 0.5f;
        var upright = Mathf.Clamp01(Vector3.Dot(hips.up, Vector3.up));

        AddReward(matchSpeed * look * 0.25f + upright * 0.75f);
    }

    Vector3 GetAvgVelocity()
    {
        Vector3 sum = Vector3.zero; int n = 0;
        foreach (var p in m_JdController.bodyPartsList) { sum += p.rb.linearVelocity; n++; }
        return n > 0 ? sum / n : Vector3.zero;
    }

    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        float denom = Mathf.Max(MTargetWalkingSpeed, 0.0001f);
        var delta = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, denom);
        return Mathf.Pow(1 - Mathf.Pow(delta / denom, 2), 2);
    }

    public void TouchedTarget() => AddReward(1f);
}
