using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
using Random = UnityEngine.Random;

public class Lan : Agent
{
    [Header("Walk / Stand")]
    [Range(0.1f, 10)]
    [SerializeField]
    private float m_TargetWalkingSpeed = 2;
    public float MTargetWalkingSpeed
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }
    const float m_maxWalkingSpeed = 10;
    public bool randomizeWalkSpeedEachEpisode;

    // Seated-look mode: lock legs into a chair-sit pose, keep torso/arms as configured
    [Header("Pose Mode")]
    [Tooltip("Force a seated-leg pose (looks like the agent is on a chair).")]
    public bool seatedLegPose = true;
    [Range(0f, 1f)] public float seatedPoseStrength = 1f;
    [Tooltip("Flip knee bend if your rig uses opposite sign.")]
    public bool invertKnee = false;
    [Tooltip("Flip hip flexion sign if your rig uses opposite sign.")]
    public bool invertHip = false;
    [Tooltip("Small ankle dorsiflexion so feet don’t look limp (sign may vary by rig).")]
    public bool invertAnkle = false;

    // Angles (normalized [-1,1]) for a typical chair sit (~90° knees / ~80° hips)
    [Range(0f, 1f)] public float hipFlexTarget = 0.80f;
    [Range(0f, 1f)] public float kneeBendTarget = 0.90f;
    [Range(0f, 0.3f)] public float ankleFlexTarget = 0.10f;

    private Vector3 m_WorldDirToWalk = Vector3.right;

    [Header("Target To Walk Towards")]
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

    // === Right-hand mug pose ===
    [Header("Right-hand Mug Pose (normalized [-1,1])")]
    [Tooltip("If on, arms/forearms/hands are forced every step: right hand holds mug, left arm relaxed.")]
    public bool lockArmsAndHands = true;

    [Range(-1f, 1f)] public float rightShoulderX = 0.25f; // lift
    [Range(-1f, 1f)] public float rightShoulderY = 0.35f; // toward chest
    [Range(-1f, 1f)] public float rightElbowBend = 0.70f; // ~100°
    [Range(-1f, 1f)] public float rightWristRollZ = 0.20f;

    [Range(-1f, 1f)] public float leftShoulderX = 0.0f;
    [Range(-1f, 1f)] public float leftShoulderY = -0.05f;
    [Range(-1f, 1f)] public float leftElbowBend = 0.10f;
    [Range(-1f, 1f)] public float leftWristRollZ = 0.0f;

    [Range(0f, 1f)] public float lockedArmStrength = 1f;

    // === Lock spine & chest ===
    [Header("Lock Torso (normalized [-1,1])")]
    public bool lockSpineAndChest = true;
    [Range(-1f, 1f)] public float spineX = 0.03f;  // slight forward looks natural when seated
    [Range(-1f, 1f)] public float spineY = 0.0f;
    [Range(-1f, 1f)] public float chestX = 0.04f;
    [Range(-1f, 1f)] public float chestY = 0.0f;
    [Range(0f, 1f)] public float lockedTorsoStrength = 1f;

    // === Mug attach (optional) ===
    [Header("Mug Attachment (optional)")]
    public Rigidbody mug;
    [Tooltip("Empty child on handR (palm center) for precise attach/orient.")]
    public Transform rightGripSocket;
    public bool gripUseFixedJoint = true;
    [Range(100, 20000)] public float gripLinearDrive = 6000f;
    [Range(100, 20000)] public float gripAngularDrive = 4000f;
    public bool ignoreHandMugCollisions = true;

    // Helpers
    OrientationCubeController m_OrientationCube;
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;
    EnvironmentParameters m_ResetParams;

    // ------- IMPORTANT: do not respawn/reset on new episodes -------
    [Header("Episode Control")]
    [Tooltip("If true, OnEpisodeBegin will not reset or randomize the agent pose/orientation.")]
    public bool disableEpisodeResets = true;

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

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        // Keep speed near-zero so FixedUpdate reward doesn't push locomotion
        m_TargetWalkingSpeed = 0.0001f;
    }

    public override void OnEpisodeBegin()
    {
        // Respect the request: do NOT respawn or randomize.
        if (disableEpisodeResets)
        {
            // Still re-attach mug if assigned and a socket exists (harmless if already attached)
            if (mug && rightGripSocket) AttachMugToRightGrip();

            // Refresh orientation helpers (no transform changes)
            UpdateOrientationObjects();
            return;
        }

        // (Fallback legacy behavior if you ever turn resets back on)
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
            bodyPart.Reset(bodyPart);

        hips.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);
        UpdateOrientationObjects();

        MTargetWalkingSpeed = randomizeWalkSpeedEachEpisode
            ? Random.Range(.1f, m_maxWalkingSpeed)
            : MTargetWalkingSpeed;

        if (mug && rightGripSocket) AttachMugToRightGrip();
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
            CollectObservationBodyPart(bodyPart, sensor);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var bp = m_JdController.bodyPartsDict;
        var i = -1;
        var a = actionBuffers.ContinuousActions;

        // Policy-driven (will be overridden for pose-locked parts)
        bp[chest].SetJointTargetRotation(a[++i], a[++i], a[++i]);
        bp[spine].SetJointTargetRotation(a[++i], a[++i], a[++i]);

        bp[thighL].SetJointTargetRotation(a[++i], a[++i], 0);
        bp[thighR].SetJointTargetRotation(a[++i], a[++i], 0);
        bp[shinL].SetJointTargetRotation(a[++i], 0, 0);
        bp[shinR].SetJointTargetRotation(a[++i], 0, 0);
        bp[footR].SetJointTargetRotation(a[++i], a[++i], a[++i]);
        bp[footL].SetJointTargetRotation(a[++i], a[++i], a[++i]);

        bp[armL].SetJointTargetRotation(a[++i], a[++i], 0);
        bp[armR].SetJointTargetRotation(a[++i], a[++i], 0);
        bp[forearmL].SetJointTargetRotation(a[++i], 0, 0);
        bp[forearmR].SetJointTargetRotation(a[++i], 0, 0);
        bp[head].SetJointTargetRotation(a[++i], a[++i], 0);

        // Strengths
        bp[chest].SetJointStrength(a[++i]);
        bp[spine].SetJointStrength(a[++i]);
        bp[head].SetJointStrength(a[++i]);
        bp[thighL].SetJointStrength(a[++i]);
        bp[shinL].SetJointStrength(a[++i]);
        bp[footL].SetJointStrength(a[++i]);
        bp[thighR].SetJointStrength(a[++i]);
        bp[shinR].SetJointStrength(a[++i]);
        bp[footR].SetJointStrength(a[++i]);
        bp[armL].SetJointStrength(a[++i]);
        bp[forearmL].SetJointStrength(a[++i]);
        bp[armR].SetJointStrength(a[++i]);
        bp[forearmR].SetJointStrength(a[++i]);

        // ===== OVERRIDE: seated legs (looks like sitting on a chair) =====
        if (seatedLegPose)
        {
            float hipFlex = (invertHip ? 1f : -1f) * hipFlexTarget;   // hip flexion (X)
            float kneeBend = (invertKnee ? -1f : 1f) * kneeBendTarget; // knee bend (X)
            float ankleFlex = (invertAnkle ? 1f : -1f) * ankleFlexTarget;

            bp[thighL].SetJointTargetRotation(hipFlex, 0f, 0f);
            bp[thighR].SetJointTargetRotation(hipFlex, 0f, 0f);
            bp[shinL].SetJointTargetRotation(kneeBend, 0f, 0f);
            bp[shinR].SetJointTargetRotation(kneeBend, 0f, 0f);
            bp[footL].SetJointTargetRotation(-ankleFlex, 0f, 0f);
            bp[footR].SetJointTargetRotation(-ankleFlex, 0f, 0f);

            bp[thighL].SetJointStrength(seatedPoseStrength);
            bp[thighR].SetJointStrength(seatedPoseStrength);
            bp[shinL].SetJointStrength(seatedPoseStrength);
            bp[shinR].SetJointStrength(seatedPoseStrength);
            bp[footL].SetJointStrength(seatedPoseStrength);
            bp[footR].SetJointStrength(seatedPoseStrength);
        }

        // ===== OVERRIDE: right-hand holding mug & left arm relaxed =====
        if (lockArmsAndHands)
        {
            // Left arm: relaxed
            bp[armL].SetJointTargetRotation(leftShoulderX, leftShoulderY, 0f);
            bp[forearmL].SetJointTargetRotation(leftElbowBend, 0f, 0f);
            bp[handL].SetJointTargetRotation(0f, 0f, leftWristRollZ);

            // Right arm: hold mug near sternum
            bp[armR].SetJointTargetRotation(rightShoulderX, rightShoulderY, 0f);
            bp[forearmR].SetJointTargetRotation(rightElbowBend, 0f, 0f);
            bp[handR].SetJointTargetRotation(0f, 0f, rightWristRollZ);

            bp[armL].SetJointStrength(lockedArmStrength);
            bp[forearmL].SetJointStrength(lockedArmStrength);
            bp[handL].SetJointStrength(lockedArmStrength);
            bp[armR].SetJointStrength(lockedArmStrength);
            bp[forearmR].SetJointStrength(lockedArmStrength);
            bp[handR].SetJointStrength(1f); // wrist firm for grip
        }

        // ===== OVERRIDE: lock torso =====
        if (lockSpineAndChest)
        {
            bp[spine].SetJointTargetRotation(spineX, spineY, 0f);
            bp[chest].SetJointTargetRotation(chestX, chestY, 0f);

            bp[spine].SetJointStrength(lockedTorsoStrength);
            bp[chest].SetJointStrength(lockedTorsoStrength);
        }
    }

    // Orientation helpers
    void UpdateOrientationObjects()
    {
        m_WorldDirToWalk = target.position - hips.position;
        m_OrientationCube.UpdateOrientation(hips, target);
        if (m_DirectionIndicator)
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        var cubeForward = m_OrientationCube.transform.forward;

        // Keep reward stable even when target speed is ~0
        var matchSpeedReward = GetMatchingVelocityReward(cubeForward * MTargetWalkingSpeed, GetAvgVelocity());
        if (float.IsNaN(matchSpeedReward))
            throw new ArgumentException("NaN in moveTowardsTargetReward.");

        var headForward = head.forward; headForward.y = 0;
        var lookAtTargetReward = (Vector3.Dot(cubeForward, headForward) + 1) * .5F;
        if (float.IsNaN(lookAtTargetReward))
            throw new ArgumentException("NaN in lookAtTargetReward.");

        AddReward(matchSpeedReward * lookAtTargetReward);
    }

    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;
        int n = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            n++;
            velSum += item.rb.linearVelocity;
        }
        return velSum / Mathf.Max(1, n);
    }

    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        float denom = Mathf.Max(MTargetWalkingSpeed, 0.0001f);
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, denom);
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / denom, 2), 2);
    }

    public void TouchedTarget() => AddReward(1f);

    // ===== Mug attach helpers =====
    void AttachMugToRightGrip()
    {
        if (!mug || !rightGripSocket) return;

        // Align mug to socket
        mug.linearVelocity = Vector3.zero;
        mug.angularVelocity = Vector3.zero;
        mug.transform.position = rightGripSocket.position;
        mug.transform.rotation = rightGripSocket.rotation;

        if (ignoreHandMugCollisions) ToggleHandMugCollisions(false);

        // Remove old joints
        var oldFj = mug.GetComponent<FixedJoint>();
        var oldCj = mug.GetComponent<ConfigurableJoint>();
        if (oldFj) Destroy(oldFj);
        if (oldCj) Destroy(oldCj);

        var handRB = m_JdController.bodyPartsDict[handR].rb;

        if (gripUseFixedJoint)
        {
            var fj = mug.gameObject.AddComponent<FixedJoint>();
            fj.connectedBody = handRB;
            fj.enablePreprocessing = false;
            fj.breakForce = Mathf.Infinity;
            fj.breakTorque = Mathf.Infinity;
        }
        else
        {
            var cj = mug.gameObject.AddComponent<ConfigurableJoint>();
            cj.connectedBody = handRB;

            cj.autoConfigureConnectedAnchor = false;
            cj.anchor = Vector3.zero; // mug pivot
            cj.connectedAnchor = handRB.transform.InverseTransformPoint(rightGripSocket.position);

            // Tiny linear & angular freedom to avoid jitter
            cj.xMotion = ConfigurableJointMotion.Limited;
            cj.yMotion = ConfigurableJointMotion.Limited;
            cj.zMotion = ConfigurableJointMotion.Limited;
            var lim = new SoftJointLimit { limit = 0.002f }; // ~2 mm
            cj.linearLimit = lim;

            cj.angularXMotion = ConfigurableJointMotion.Limited;
            cj.angularYMotion = ConfigurableJointMotion.Limited;
            cj.angularZMotion = ConfigurableJointMotion.Limited;
            var low = new SoftJointLimit { limit = -2f };
            var high = new SoftJointLimit { limit = 2f };
            cj.lowAngularXLimit = low;
            cj.highAngularXLimit = high;
            var ayz = new SoftJointLimit { limit = 2f };
            cj.angularYLimit = ayz;
            cj.angularZLimit = ayz;

            var ld = new JointDrive { positionSpring = gripLinearDrive, positionDamper = gripLinearDrive * 0.5f, maximumForce = Mathf.Infinity };
            cj.xDrive = ld; cj.yDrive = ld; cj.zDrive = ld;

            var ad = new JointDrive { positionSpring = gripAngularDrive, positionDamper = gripAngularDrive * 0.5f, maximumForce = Mathf.Infinity };
            cj.angularXDrive = ad; cj.angularYZDrive = ad;

            cj.enablePreprocessing = false;
        }
    }

    void ToggleHandMugCollisions(bool enable)
    {
        if (!mug) return;

        var mugCols = mug.GetComponentsInChildren<Collider>();
        var rightHandParts = new[] { armR, forearmR, handR };
        foreach (var t in rightHandParts)
        {
            if (!t) continue;
            var handCols = t.GetComponentsInChildren<Collider>();
            foreach (var hc in handCols)
                foreach (var mc in mugCols)
                    if (hc && mc) Physics.IgnoreCollision(hc, mc, !enable);
        }
    }
}
