using UnityEngine;


public class IK_Solver : MonoBehaviour
{
    public Transform[] m_BoneList;
    public Transform m_EffectorPoint;
    public Transform m_TargetPoint;
    public int m_MaxInteration = 10;
    public float m_Threshold = 1.0f;

    [SerializeField] float m_Speed;
    [SerializeField] float m_MaxAngle;
    [SerializeField] float m_MinAngle;

    private void Update()
    {
        CCDIk();
    }

    void CCDIk() 
    {
        for (int i = 0; i < m_MaxInteration; i++) 
        {
            for (int j = 3; j < m_BoneList.Length; j++)
            {
                Vector3 bone2Effector = m_EffectorPoint.position - m_BoneList[j].position;
                Vector3 bone2Target = m_TargetPoint.position - m_BoneList[j].position;
                Quaternion rotation = Quaternion.FromToRotation(bone2Effector, bone2Target);
                Vector3 axis = m_BoneList[j].localToWorldMatrix * Vector3.forward;
                Debug.DrawLine(m_BoneList[j].position, m_BoneList[j].position + 10 * axis, Color.black);
                Vector3 quaternionAxis = new Vector3(rotation.x, rotation.y, rotation.z);
                Vector3 projected = Vector3.ProjectOnPlane(quaternionAxis, axis);
                Quaternion twist = new Quaternion(projected.x, projected.y, projected.z, rotation.w).normalized;
                Quaternion swing = rotation * Quaternion.Inverse(twist);

                Quaternion targetRotation = swing * m_BoneList[j].rotation;

                Vector3 boneDir = targetRotation * axis;
                Vector3 parentBoneDir = m_BoneList[j - 1].rotation * axis;
                float angle = Vector3.Angle(boneDir, parentBoneDir);
                Vector3 clampDir = Vector3.Slerp(boneDir, parentBoneDir, (angle - m_MaxAngle) / angle);

                targetRotation = Quaternion.FromToRotation(boneDir, clampDir) * targetRotation;

                m_BoneList[j].rotation = Quaternion.Slerp(
                    m_BoneList[j].rotation, targetRotation,
                    1 - Mathf.Exp(-m_Speed * Time.deltaTime));
            }
            if (Vector3.Distance(m_EffectorPoint.position, m_TargetPoint.position) < m_Threshold)
                break;
        }
    }
}
