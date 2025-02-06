using UnityEngine;


public class IK_Solver : MonoBehaviour
{
    public BoneProperty[] m_BoneList;
    public Transform m_EffectorPoint;
    public Transform m_TargetPoint;
    public int m_MaxInteration = 10;
    public float m_Threshold = 1.0f;

    [SerializeField] float m_Speed;

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
                var boneTransform = m_BoneList[j].m_Transform;

                Vector3 bone2Effector = m_EffectorPoint.position - boneTransform.position;
                Vector3 bone2Target = m_TargetPoint.position - boneTransform.position;
                Quaternion rotation = Quaternion.FromToRotation(bone2Effector, bone2Target);
                Vector3 axis = boneTransform.localToWorldMatrix * Vector3.forward;
                Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * axis, Color.black);

                // 固定在一个方向上旋转
                Vector3 quaternionAxis = new Vector3(rotation.x, rotation.y, rotation.z);
                Vector3 projected = Vector3.ProjectOnPlane(quaternionAxis, axis);
                Quaternion twist = new Quaternion(projected.x, projected.y, projected.z, rotation.w).normalized;
                Quaternion swing = rotation * Quaternion.Inverse(twist);

                Quaternion targetRotation = swing * boneTransform.rotation;

                // 角度控制
                Vector3 boneDir = targetRotation * Vector3.right;
                Vector3 parentBoneDir = boneTransform.parent.rotation * Vector3.right;
                float angle = Vector3.Angle(boneDir, parentBoneDir);
                Vector3 clampDir = Vector3.Slerp(boneDir, parentBoneDir, (angle - m_BoneList[j].m_MaxAngle) / angle);

                Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * boneDir, Color.red);
                Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * parentBoneDir, Color.yellow);

                targetRotation = Quaternion.FromToRotation(boneDir, clampDir) * targetRotation;

                //矫正误差
                Matrix4x4 newLocalToWorldMatrix = Matrix4x4.TRS(boneTransform.position, targetRotation, boneTransform.lossyScale);
                Vector3 newAxis = newLocalToWorldMatrix * Vector3.forward;
                Quaternion offset = Quaternion.FromToRotation(newAxis, axis);
                targetRotation = offset * targetRotation;

                boneTransform.rotation = Quaternion.Slerp(
                    boneTransform.rotation, targetRotation,
                    1 - Mathf.Exp(-m_Speed * Time.deltaTime));
            }
            if (Vector3.Distance(m_EffectorPoint.position, m_TargetPoint.position) < m_Threshold)
                break;
        }
    }
}
