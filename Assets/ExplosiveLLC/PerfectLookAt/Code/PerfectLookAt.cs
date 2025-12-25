using System.Collections.Generic;
using UnityEngine;

public enum TargetMode
{
	TargetObject,
	CameraDirection,
	CustomDirection
}

public class PerfectLookAt:MonoBehaviour
{
	private Vector3 m_gameObjectScale;

	[Header("Target Settings")]
	public TargetMode m_TargetMode = TargetMode.TargetObject;
	public GameObject m_TargetObject;
	public Camera m_Camera;
	public float m_CameraForwardDistance = 10f;
	public Vector3 m_CustomDirection = Vector3.forward;

	[Header("Look At Settings")]
	public Vector3 m_UpVector = Vector3.up;
	private float m_Weight = 1.0f;
	public float m_LookAtBlendSpeed = 5.0f;
	private float m_BlendSpeed = 0.0f;

	[Header("Advanced Settings")]
	public float m_MaxLookAngle = 90f;
	public float m_DeadZoneAngle = 2f;
	public float m_DirectionSmoothTime = 0.1f;
	private Vector3 m_SmoothedTargetPosition;
	private Vector3 m_TargetPositionVelocity;

	[Header("Vertical Angle Control")]
	public bool m_UseVerticalDeadZone = true;
	public float m_VerticalDeadZoneMin = -12f;
	public float m_VerticalDeadZoneMax = 20f;
	public float m_VerticalTransitionRange = 8f;

	[Header("Vertical Angle Limits")]
	public float m_MinVerticalAngle = -40f;
	public float m_MaxVerticalAngle = 50f;
	public float m_VerticalAngleMultiplier = 0.8f;
	public bool m_ShowVerticalAngleDebug = false;

	[Header("Leg Stabilizer Settings")]
	public float m_LegStabilizerMinDistanceToStartSolving = 0.005f;
	public byte LegStabilizerMaxIterations = 20;
	public bool m_DrawDebugLookAtLines = false;
	private bool m_IsValid = true;
	public PerfectLookAtData[] m_LookAtBones;
	private Quaternion[] m_BlendedRotations;
	private Quaternion[] m_LastFrameRotations;
	public PerfectLookAtLegStabilizer[] m_legStabilizers;

	public float GetLookAtWeight()
	{ return m_Weight; }

	public void SetLookAtWeight(float weight)
	{
		m_Weight = Mathf.Clamp(weight, 0.0f, 1.0f);
		m_BlendSpeed = 0.0f;
	}

	public void EnablePerfectLookat(float time, bool cancelCurrentTransition = true)
	{
		if (Mathf.Abs(m_Weight - 1.0f) > Mathf.Epsilon) {
			if (Mathf.Abs(m_BlendSpeed) < Mathf.Epsilon || cancelCurrentTransition) {
				m_BlendSpeed = (1.0f - m_Weight) / time;
			}
		}
		else { m_BlendSpeed = 0.0f; }
	}

	public void DisablePerfectLookat(float time, bool cancelCurrentTransition = true)
	{
		if (Mathf.Abs(m_Weight) > Mathf.Epsilon) {
			if (Mathf.Abs(m_BlendSpeed) < Mathf.Epsilon || cancelCurrentTransition) {
				m_BlendSpeed = -m_Weight / time;
			}
		}
		else { m_BlendSpeed = 0.0f; }
	}

	public bool IsInTransition()
	{
		if (Mathf.Abs(m_BlendSpeed) > Mathf.Epsilon) { return true; }

		return false;
	}

	public float GetTimeToFinishTransition()
	{
		float ret = 0.0f;

		// Multiplied by 10 to avoid numerical errors on division.
		if (Mathf.Abs(m_BlendSpeed) > 10.0f * Mathf.Epsilon) {

			// On enable with positive speed.
			if (m_BlendSpeed > 0.0f) { ret = (1.0f - m_Weight) / m_BlendSpeed; }

			// On disable with negative speed.
			else { ret = m_Weight / m_BlendSpeed; }
		}

		return ret;
	}

	public void SetCameraReference(Camera camera, float forwardDistance = 10f)
	{
		m_Camera = camera;
		m_CameraForwardDistance = forwardDistance;
		m_TargetMode = TargetMode.CameraDirection;
	}

	public void SetCustomDirection(Vector3 direction)
	{
		m_CustomDirection = direction.normalized;
		m_TargetMode = TargetMode.CustomDirection;
	}

	public void SetTargetMode(TargetMode mode)
	{
		m_TargetMode = mode;
	}

	private float CalculateVerticalInfluence(float verticalAngle)
	{
		if (!m_UseVerticalDeadZone)
		{
			return 1.0f;
		}

		// Check if angle is within dead zone
		if (verticalAngle >= m_VerticalDeadZoneMin && verticalAngle <= m_VerticalDeadZoneMax)
		{
			return 0.0f; // No vertical influence in dead zone
		}

		// Calculate smooth transition
		float influence = 0.0f;

		if (verticalAngle < m_VerticalDeadZoneMin)
		{
			// Below dead zone (looking down)
			float transitionStart = m_VerticalDeadZoneMin - m_VerticalTransitionRange;
			if (verticalAngle <= transitionStart)
			{
				influence = 1.0f;
			}
			else
			{
				// Smooth transition from 0 to 1
				float t = (m_VerticalDeadZoneMin - verticalAngle) / m_VerticalTransitionRange;
				influence = Mathf.SmoothStep(0.0f, 1.0f, t);
			}
		}
		else if (verticalAngle > m_VerticalDeadZoneMax)
		{
			// Above dead zone (looking up)
			float transitionEnd = m_VerticalDeadZoneMax + m_VerticalTransitionRange;
			if (verticalAngle >= transitionEnd)
			{
				influence = 1.0f;
			}
			else
			{
				// Smooth transition from 0 to 1
				float t = (verticalAngle - m_VerticalDeadZoneMax) / m_VerticalTransitionRange;
				influence = Mathf.SmoothStep(0.0f, 1.0f, t);
			}
		}

		return influence;
	}

	private Vector3 ApplyVerticalCorrection(Vector3 cameraDirection, Vector3 fromPosition)
	{
		// Calculate horizontal direction (XZ plane)
		Vector3 horizontalDirection = cameraDirection;
		horizontalDirection.y = 0;
		horizontalDirection.Normalize();

		// Calculate vertical angle of camera direction
		float verticalAngle = Mathf.Asin(cameraDirection.y) * Mathf.Rad2Deg;

		// Calculate influence based on dead zone
		float influence = CalculateVerticalInfluence(verticalAngle);

		// Apply multiplier
		influence *= m_VerticalAngleMultiplier;

		// Clamp vertical angle
		verticalAngle = Mathf.Clamp(verticalAngle, m_MinVerticalAngle, m_MaxVerticalAngle);

		// Calculate final vertical component
		float finalVerticalAngle = verticalAngle * influence;

		// Reconstruct direction with corrected vertical angle
		float finalVerticalAngleRad = finalVerticalAngle * Mathf.Deg2Rad;
		Vector3 finalDirection = horizontalDirection * Mathf.Cos(finalVerticalAngleRad);
		finalDirection.y = Mathf.Sin(finalVerticalAngleRad);
		finalDirection.Normalize();

		// Debug visualization
		if (m_ShowVerticalAngleDebug && m_LookAtBones.Length > 0)
		{
			Vector3 startPos = fromPosition;
			float debugLength = 3f;

			// Original camera direction (red)
			Debug.DrawRay(startPos, cameraDirection * debugLength, Color.red);

			// Horizontal direction (blue)
			Debug.DrawRay(startPos, horizontalDirection * debugLength, Color.blue);

			// Final corrected direction (green)
			Debug.DrawRay(startPos, finalDirection * debugLength, Color.green);

			// Angle text would require UnityEditor, so we use simple debug sphere
			Vector3 textPos = startPos + Vector3.up * 0.5f;
			Debug.DrawLine(textPos + Vector3.left * 0.1f, textPos + Vector3.right * 0.1f, Color.yellow);
		}

		return fromPosition + finalDirection * m_CameraForwardDistance;
	}

	private Vector3 CalculateTargetPosition()
	{
		Vector3 rawTargetPosition;

		switch (m_TargetMode)
		{
			case TargetMode.TargetObject:
				if (m_TargetObject != null)
				{
					rawTargetPosition = m_TargetObject.transform.position;
				}
				else
				{
					rawTargetPosition = m_LookAtBones[0].m_Bone.position + m_LookAtBones[0].m_Bone.forward * 10f;
				}
				break;

			case TargetMode.CameraDirection:
				Camera cam = m_Camera != null ? m_Camera : Camera.main;
				if (cam != null)
				{
					// Apply vertical correction to camera direction
					Vector3 bonePosition = m_LookAtBones[0].m_Bone.position;
					Vector3 cameraDirection = cam.transform.forward;
					rawTargetPosition = ApplyVerticalCorrection(cameraDirection, bonePosition);
				}
				else
				{
					rawTargetPosition = m_LookAtBones[0].m_Bone.position + Vector3.forward * 10f;
				}
				break;

			case TargetMode.CustomDirection:
				rawTargetPosition = m_LookAtBones[0].m_Bone.position + m_CustomDirection * 10f;
				break;

			default:
				rawTargetPosition = m_LookAtBones[0].m_Bone.position + Vector3.forward * 10f;
				break;
		}

		// Apply smoothing
		if (m_DirectionSmoothTime > 0.0f)
		{
			m_SmoothedTargetPosition = Vector3.SmoothDamp(m_SmoothedTargetPosition, rawTargetPosition,
				ref m_TargetPositionVelocity, m_DirectionSmoothTime);
			return m_SmoothedTargetPosition;
		}

		return rawTargetPosition;
	}

	private bool IsTargetValid()
	{
		switch (m_TargetMode)
		{
			case TargetMode.TargetObject:
				return m_TargetObject != null;

			case TargetMode.CameraDirection:
				return (m_Camera != null || Camera.main != null);

			case TargetMode.CustomDirection:
				return true;

			default:
				return false;
		}
	}

	public static Quaternion GetWorldLookAtRotation(Vector3 targetVector, Vector3 fwdVectorInWorldSpace)
	{
		Quaternion worldLookAtRotation;
		Vector3 rotationAxis;
		float angle = Vector3.Angle(fwdVectorInWorldSpace, targetVector) * Mathf.Deg2Rad;

		if (Mathf.Abs(angle - Mathf.PI) < Mathf.Epsilon || angle <= Mathf.Epsilon) {
			rotationAxis = Vector3.up;
		}
		else { rotationAxis = Vector3.Cross(fwdVectorInWorldSpace, targetVector); }

		worldLookAtRotation = QuaternionFromAngleAxis(ref rotationAxis, angle);

		return worldLookAtRotation;
	}

	private Quaternion PerfectLookAtSlerp(Quaternion a, Quaternion b, float weight)
	{
		if (Mathf.Abs(weight - 1.0f) < Mathf.Epsilon) { return b; }

		if (weight < Mathf.Epsilon) { return a; }

		return Quaternion.Slerp(a, b, weight);
	}

	public static Quaternion QuaternionFromAngleAxis(ref Vector3 rotationAxis, float angleRad)
	{
		float halfAngleRad = angleRad * 0.5f;
		rotationAxis = rotationAxis.normalized * Mathf.Sin(halfAngleRad);
		float quatW = Mathf.Cos(halfAngleRad);
		return new Quaternion(rotationAxis.x, rotationAxis.y, rotationAxis.z, quatW);
	}

	private void CheckForForceDefaultRotation()
	{
		for (int i = 0; i < m_LookAtBones.Length; i++) {
			PerfectLookAtData lookAtData = m_LookAtBones[i];

			if (lookAtData.m_ResetToDefaultRotation) {
				lookAtData.m_Bone.transform.localRotation = lookAtData.GetDefaultRotation();
			}

			for (int j = 0; j < lookAtData.m_LinkedBones.Length; j++) {
				PerfecLookAtLinkedBones linkedBoneData = lookAtData.m_LinkedBones[j];
				if (linkedBoneData.m_ResetToDefaultRotation) {
					linkedBoneData.m_Bone.transform.localRotation = linkedBoneData.GetDefaultRotation();
				}
			}
		}
	}

	private bool CheckValidity()
	{
		bool ret = true;

		for (int i = 0; i < m_LookAtBones.Length; i++) {
			PerfectLookAtData lookAtData = m_LookAtBones[i];
			if (lookAtData.m_Bone == null) {
				ret = false;
				break;
			}

			for (int j = 0; j < lookAtData.m_LinkedBones.Length; j++) {
				if (lookAtData.m_LinkedBones[j].m_Bone == null) {
					ret = false;
					break;
				}
			}
		}

		return ret;
	}

	private float GetAngleFromQuaternionDeg(ref Quaternion inputQuat)
	{
		float ret = Mathf.Acos(inputQuat.w) * Mathf.Rad2Deg * 2.0f;
		return ret;
	}

	private float GetAngleFromQuaternionDeg(Quaternion inputQuat)
	{
		float ret = Mathf.Acos(inputQuat.w) * Mathf.Rad2Deg * 2.0f;
		return ret;
	}

	private float GetAngleFromQuaternionRad(Quaternion inputQuat)
	{
		float ret = Mathf.Acos(inputQuat.w) * 2.0f;
		return ret;
	}

	private float GetAngleFromQuaternionRad(ref Quaternion inputQuat)
	{
		float ret = Mathf.Acos(inputQuat.w) * 2.0f;
		return ret;
	}

	private Vector3 GetForwardVector(ref Transform inputTr, FwdDirection inputAxis)
	{
		switch (inputAxis) {
			case FwdDirection.X_AXIS:
				return inputTr.right;

			case FwdDirection.Y_AXIS:
				return inputTr.up;

			case FwdDirection.Z_AXIS:
				return inputTr.forward;

			case FwdDirection.MINUS_X_AXIS:
				return -inputTr.right;

			case FwdDirection.MINUS_Y_AXIS:
				return -inputTr.up;

			case FwdDirection.MINUS_Z_AXIS:
				return -inputTr.forward;

			default:
				return inputTr.forward;
		}
	}

	private Vector3 BlendTowardTargetVector(ref Transform inputTr, FwdDirection inputAxis)
	{
		switch (inputAxis) {
			case FwdDirection.X_AXIS:
				return inputTr.right;

			case FwdDirection.Y_AXIS:
				return inputTr.up;

			case FwdDirection.Z_AXIS:
				return inputTr.forward;

			case FwdDirection.MINUS_X_AXIS:
				return -inputTr.right;

			case FwdDirection.MINUS_Y_AXIS:
				return -inputTr.up;

			case FwdDirection.MINUS_Z_AXIS:
				return -inputTr.forward;

			default:
				return inputTr.forward;
		}

	}

	private void CheckIfCharacterScaleChanged()
	{
		if ((transform.localScale - m_gameObjectScale) != Vector3.zero) {
			for (int i = 0; i < m_legStabilizers.Length; i++) {
				m_legStabilizers[i].Initialize();
			}

			m_gameObjectScale = transform.localScale;
		}
	}

	private void Start()
	{
		m_gameObjectScale = gameObject.transform.localScale;
		m_BlendedRotations = new Quaternion[m_LookAtBones.Length];
		m_LastFrameRotations = new Quaternion[m_LookAtBones.Length];
		m_IsValid = CheckValidity();

		// Initialize camera reference if not set
		if (m_TargetMode == TargetMode.CameraDirection && m_Camera == null)
		{
			m_Camera = Camera.main;
		}

		// Initialize smoothed target position
		if (m_LookAtBones.Length > 0)
		{
			m_SmoothedTargetPosition = CalculateTargetPosition();
		}

		for (int i = 0; i < m_LookAtBones.Length; i++) {
			PerfectLookAtData lookAtBoneData = m_LookAtBones[i];
			PerfecLookAtLinkedBones[] currentLinkedBones = lookAtBoneData.m_LinkedBones;
			lookAtBoneData.SetDefaultRotation(lookAtBoneData.m_Bone.localRotation);
			m_LastFrameRotations[i] = lookAtBoneData.m_Bone.localRotation;

			lookAtBoneData.CheckJointRotation();

			for (int j = 0; j < currentLinkedBones.Length; j++) {
				currentLinkedBones[j].SetDefaultRotation(currentLinkedBones[j].m_Bone.localRotation);
				currentLinkedBones[j].SetLastFrameRotation(currentLinkedBones[j].m_Bone.rotation);
			}
		}

		// Initialize Leg Bones for Leg IK.
		for (int i = 0; i < m_legStabilizers.Length; i++) {
			m_legStabilizers[i].Initialize();
		}
	}

	private void Update()
	{
		if (m_LookAtBones.Length > 0) {
			if (m_DrawDebugLookAtLines && IsTargetValid()) {
				Vector3 targetPos = CalculateTargetPosition();
				Vector3 destination = targetPos - m_LookAtBones[0].m_Bone.position;
				Debug.DrawLine(m_LookAtBones[0].m_Bone.position, m_LookAtBones[0].m_Bone.position + destination, Color.cyan);

				// Draw virtual target point in CameraDirection mode
				if (m_TargetMode == TargetMode.CameraDirection)
				{
					Debug.DrawLine(targetPos + Vector3.up * 0.5f, targetPos - Vector3.up * 0.5f, Color.yellow);
					Debug.DrawLine(targetPos + Vector3.right * 0.5f, targetPos - Vector3.right * 0.5f, Color.yellow);
				}
			}

			// If the system already in a transtion.
			if (Mathf.Abs(m_BlendSpeed) > Mathf.Epsilon && m_IsValid) {
				m_Weight += m_BlendSpeed * Time.deltaTime;

				if (m_BlendSpeed > 0.0f) {
					if (m_Weight > 1.0f) {
						m_Weight = 1.0f;
						m_BlendSpeed = 0.0f;
					}
				}
				else if (m_Weight < 0.0f) {
					m_Weight = 0.0f;
					m_BlendSpeed = 0.0f;
				}
			}
			else { m_BlendSpeed = 0.0f; }
		}
	}

	private void LateUpdate()
	{
		if (!IsTargetValid()) {
			string warningMessage = "";
			switch (m_TargetMode)
			{
				case TargetMode.TargetObject:
					warningMessage = "No target object set. Set m_TargetObject or switch to CameraDirection mode.";
					break;
				case TargetMode.CameraDirection:
					warningMessage = "No camera found. Set m_Camera or ensure Camera.main exists.";
					break;
			}
			Debug.LogWarning(warningMessage, this);
			return;
		}

		if (!m_IsValid) {
			Debug.LogWarning("Missing bones in PerfectlookAt component. Component won't work unless all bones are set", this);
			return;
		}

		if (m_Weight < Mathf.Epsilon) {

			// Updating last frame rotation even if the weight is zero.
			for (int i = 0; i < m_LookAtBones.Length; i++) {
				PerfectLookAtData lookAtBoneData = m_LookAtBones[i];
				m_LastFrameRotations[i] = lookAtBoneData.m_Bone.localRotation;

				for (int j = 0; j < lookAtBoneData.m_LinkedBones.Length; j++) {
					lookAtBoneData.m_LinkedBones[j].SetLastFrameRotation(lookAtBoneData.m_LinkedBones[j].m_Bone.rotation);
				}

			}

			return;
		}

		// Updating the bones rotations.
		if (m_LookAtBones.Length > 0) {

			// If scale has changed the leg stabilizers get reinitialized.
			CheckIfCharacterScaleChanged();

			CheckForForceDefaultRotation();

			// Cache Leg Bones for IK Leg Fix.
			for (int i = 0; i < m_legStabilizers.Length; i++) { m_legStabilizers[i].CacheBones(); }

			Dictionary<int, Quaternion> linkedBonesRotations = new Dictionary<int, Quaternion>();
			Vector3 rotatedInitFwdVec = GetForwardVector(ref m_LookAtBones[0].m_Bone, m_LookAtBones[0].m_ForwardAxis);
			Vector3 currentFwdVec;
			Vector3 parentFwdVec;

			// Get target position from the selected mode
			Vector3 targetPosition = CalculateTargetPosition();
			Vector3 targetVector = targetPosition - m_LookAtBones[0].m_Bone.position;

			// Check dead zone
			float targetAngle = Vector3.Angle(rotatedInitFwdVec, targetVector);
			if (targetAngle < m_DeadZoneAngle)
			{
				return;
			}

			// Check max look angle
			if (targetAngle > m_MaxLookAngle)
			{
				// Clamp to max angle
				Vector3 clampedDirection = Vector3.RotateTowards(rotatedInitFwdVec, targetVector,
					m_MaxLookAngle * Mathf.Deg2Rad, 0f);
				targetVector = clampedDirection.normalized * targetVector.magnitude;
			}

			Vector3 firstBoneRotatedInitFwdVec = rotatedInitFwdVec;
			byte numBonesToRotate = 1;

			for (int i = 0; i < m_LookAtBones.Length; i++) {
				Transform currentBone = m_LookAtBones[i].m_Bone;
				Transform parentBone = currentBone.parent;

				float rotationLimit = m_LookAtBones[i].m_RotationLimit;
				bool parentExists = true;

				currentFwdVec = GetForwardVector(ref currentBone, m_LookAtBones[i].m_ForwardAxis);
				parentFwdVec = GetForwardVector(ref parentBone, m_LookAtBones[i].m_ParentBoneForwardAxis);

				if (parentBone != null) {
					currentFwdVec = GetForwardVector(ref currentBone, m_LookAtBones[i].m_ForwardAxis);
					parentFwdVec = GetForwardVector(ref parentBone, m_LookAtBones[i].m_ParentBoneForwardAxis);
					float diffAngleFromAnim = Vector3.Angle(currentFwdVec, parentFwdVec);

					// In case animation already has a bone with relative rotation higher than the limit specified by user.
					if (diffAngleFromAnim > rotationLimit) { rotationLimit = diffAngleFromAnim; }
				}
				else { parentExists = false; }

				Quaternion lookAtRot = GetWorldLookAtRotation(targetVector, rotatedInitFwdVec);

				if (m_LookAtBones[i].m_RotateAroundUpVectorWeight > 0.0f) {
					Vector3 currentRotationAxis;
					currentRotationAxis.x = lookAtRot.x;
					currentRotationAxis.y = lookAtRot.y;
					currentRotationAxis.z = lookAtRot.z;

					// Checking the up vector direction for rotation.
					float rotationSign = Mathf.Sign(Vector3.Cross(firstBoneRotatedInitFwdVec, targetVector).y);
					Vector3 finalUpVector = rotationSign * m_UpVector;
					finalUpVector = Vector3.Lerp(currentRotationAxis, finalUpVector, m_LookAtBones[i].m_RotateAroundUpVectorWeight);

					lookAtRot = QuaternionFromAngleAxis(ref finalUpVector, GetAngleFromQuaternionRad(lookAtRot));
				}

				Quaternion childRotation = lookAtRot * currentBone.rotation;

				if (parentExists) {

					// Checking the angle difference.
					float diffAngle = Mathf.Abs(Vector3.Angle(parentFwdVec, lookAtRot * currentFwdVec)) - rotationLimit;
					if (diffAngle > 0) {
						Vector3 rotationAxis = new Vector3(lookAtRot.x, lookAtRot.y, lookAtRot.z);
						float limitAngleRad = GetAngleFromQuaternionRad(ref lookAtRot) + Mathf.Deg2Rad * (-diffAngle);

						lookAtRot = QuaternionFromAngleAxis(ref rotationAxis, limitAngleRad);
						childRotation = lookAtRot * currentBone.rotation;

						// Blending with the rotation before assigning it to bone so it can have no effect on rotaiton limit checking
						// and we can have updated bone position for calculating the rotated fwd vec and target vector.
						Quaternion currentBoneRotationLS = currentBone.localRotation;
						m_LookAtBones[i].m_Bone.rotation = childRotation;
						m_BlendedRotations[i] = PerfectLookAtSlerp(currentBoneRotationLS, m_LookAtBones[i].m_Bone.localRotation, m_Weight);

						if (m_LookAtBones[i].m_LinkedBones.Length > 0) {
							linkedBonesRotations.Add(i, PerfectLookAtSlerp(Quaternion.identity, lookAtRot, m_Weight));
						}

						if (i != m_LookAtBones.Length - 1) {
							Vector3 currentBoneToParentPosDiff = m_LookAtBones[0].m_Bone.position - m_LookAtBones[i + 1].m_Bone.position;
						Vector3 firstBoneToTargetDiff = targetPosition - m_LookAtBones[0].m_Bone.position;
							rotatedInitFwdVec = GetForwardVector(ref m_LookAtBones[0].m_Bone, m_LookAtBones[0].m_ForwardAxis);
							rotatedInitFwdVec.Normalize();
							rotatedInitFwdVec = firstBoneToTargetDiff.magnitude * rotatedInitFwdVec;
							rotatedInitFwdVec = currentBoneToParentPosDiff + rotatedInitFwdVec;

							targetVector = currentBoneToParentPosDiff + firstBoneToTargetDiff;

							numBonesToRotate++;

							if (m_DrawDebugLookAtLines) {
								Debug.DrawLine(m_LookAtBones[i + 1].m_Bone.position, m_LookAtBones[i + 1].m_Bone.position + rotatedInitFwdVec, Color.green);
								Debug.DrawLine(m_LookAtBones[i + 1].m_Bone.position, m_LookAtBones[i + 1].m_Bone.position + targetVector, Color.red);
							}
						}
					}
					else {
						Quaternion currentBoneRotationLS = currentBone.localRotation;
						m_LookAtBones[i].m_Bone.rotation = childRotation;
						m_BlendedRotations[i] = PerfectLookAtSlerp(currentBoneRotationLS, m_LookAtBones[i].m_Bone.localRotation, m_Weight);

						if (m_LookAtBones[i].m_LinkedBones.Length > 0) {
							linkedBonesRotations.Add(i, PerfectLookAtSlerp(Quaternion.identity, lookAtRot, m_Weight));
						}

						break;
					}
				}
				else {
					Quaternion currentBoneRotationLS = currentBone.localRotation;
					m_LookAtBones[i].m_Bone.rotation = childRotation;
					m_BlendedRotations[i] = PerfectLookAtSlerp(currentBoneRotationLS, m_LookAtBones[i].m_Bone.localRotation, m_Weight);

					if (m_LookAtBones[i].m_LinkedBones.Length > 0) {
						linkedBonesRotations.Add(i, PerfectLookAtSlerp(Quaternion.identity, lookAtRot, m_Weight));
					}

					if (i < m_LookAtBones.Length - 1) {
						Debug.LogWarning("Warning Bone name doesn't have a parent. The rest of the PerfectLookAt bone chain won't work after this bone!", this);
						break;
					}
				}
			}

			// Setting bones rotations final pass. The initial bone rotations are set during the last loop.
			// These two loops are used for blending and smooth movement.

			// Updating last frame rotations. Two loops are separated to have less jumps in memory and be more cache friendly.
			bool isBlending = Mathf.Abs(m_Weight - 1.0f) > Mathf.Epsilon;

			// Calculating the blend weight to blend between the current frame and last frame to achieve smooth rotations in
			// dynamic animations with large range of movements.
			float smoothingWeight = Mathf.Clamp(m_LookAtBlendSpeed * Time.deltaTime, 0.0f, 1.0f);

			// Linearly blend the smoothingWeight to zero and its current frame value to be sure when perfect look at blend
			// weight is zero-the rotation is exactly the same as animation and no last frame is getting blended in.
			smoothingWeight = (smoothingWeight - 1.0f) * m_Weight + 1.0f;

			for (int k = 0; k < m_LookAtBones.Length; k++) {
				PerfectLookAtData lookAtBoneData = m_LookAtBones[k];
				Quaternion boneLocalRotation;
				if (isBlending && k < numBonesToRotate) { boneLocalRotation = m_BlendedRotations[k]; }
				else { boneLocalRotation = lookAtBoneData.m_Bone.localRotation; }

				Quaternion localRotationFromAnim = lookAtBoneData.m_Bone.localRotation;

				lookAtBoneData.m_Bone.localRotation = PerfectLookAtSlerp(m_LastFrameRotations[k], boneLocalRotation, smoothingWeight);

				m_LastFrameRotations[k] = lookAtBoneData.m_Bone.localRotation;
			}

			// Updating linked bones.
			for (int m = 0; m < m_LookAtBones.Length; m++) {
				PerfectLookAtData lookAtBoneData = m_LookAtBones[m];

				if (lookAtBoneData.m_LinkedBones.Length > 0) {
					if (m < numBonesToRotate) {
						Quaternion linkedBoneLookAtQuat = linkedBonesRotations[m];
						for (int n = 0; n < lookAtBoneData.m_LinkedBones.Length; n++) {
							Transform linkedBone = lookAtBoneData.m_LinkedBones[n].m_Bone;

							linkedBone.rotation = PerfectLookAtSlerp(lookAtBoneData.m_LinkedBones[n].GetLastFrameRotation(),
								linkedBoneLookAtQuat * linkedBone.rotation, smoothingWeight);
							lookAtBoneData.m_LinkedBones[n].SetLastFrameRotation(linkedBone.rotation);

						}
					}
					else {
						for (int n = 0; n < lookAtBoneData.m_LinkedBones.Length; n++) {
							lookAtBoneData.m_LinkedBones[n].SetLastFrameRotation(lookAtBoneData.m_LinkedBones[n].m_Bone.rotation);
						}
					}
				}
			}

			// Fix Leg Bones for IK Leg Fix.
			for (int i = 0; i < m_legStabilizers.Length; i++) {
				m_legStabilizers[i].FixLeg(LegStabilizerMaxIterations, m_LegStabilizerMinDistanceToStartSolving);
			}
		}
	}
}