using UnityEditor.PackageManager;
using UnityEngine;


public class IK_Solver : MonoBehaviour
{
    public GameObject[] m_RootGameObjects;
    public int m_MaxInteration = 10;
    public float m_Threshold = 1.0f;

    [SerializeField] float m_Speed;

    private BoneProperty[] m_BoneList;
    private Transform m_EffectorPoint;
    private Transform m_TargetPoint;
    private RaycastHit m_Hit;

    private void Update()
    {
        foreach (var go in m_RootGameObjects) 
        {
            var chain = go.GetComponent<BoneIKChain>();
            m_BoneList = chain.m_BoneList;
            m_EffectorPoint = chain.m_EffectorPoint;
            m_TargetPoint = chain.m_TargetPoint;

            var pos = m_BoneList[m_BoneList.Length - 1].transform.position;
            var length = (pos - m_EffectorPoint.position).magnitude;

            //var ray = new Ray(m_BoneList[m_BoneList.Length - 1].transform.position + Vector3.up * 10, Vector3.down);
            if (Physics.SphereCast(pos, 0.01f, Vector3.down, out m_Hit))
            {
                m_TargetPoint.position = new Vector3(m_Hit.point.x, m_Hit.point.y, m_Hit.point.z);
            }
            Debug.DrawLine(m_TargetPoint.position, m_TargetPoint.position + m_Hit.normal * 10f, Color.yellow);

            CCDIk(go);
        }
    }

    void CCDIk(GameObject go) 
    {
        for (int i = 0; i < m_MaxInteration; i++)
        {

            {
                var bone = m_BoneList[m_BoneList.Length - 1];
                var boneTransform = bone.m_Transform;
                Quaternion rotation = Quaternion.FromToRotation(boneTransform.localToWorldMatrix * Vector3.right, m_Hit.normal);
                Vector3 axis = boneTransform.localToWorldMatrix * bone.m_Axis;
                //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * axis, Color.black);

                // 固定在一个方向上旋转
                Vector3 quaternionAxis = new Vector3(rotation.x, rotation.y, rotation.z);
                Vector3 projected = Vector3.ProjectOnPlane(quaternionAxis, axis);
                Quaternion twist = new Quaternion(projected.x, projected.y, projected.z, rotation.w).normalized;
                Quaternion swing = rotation * Quaternion.Inverse(twist);

                Quaternion targetRotation = swing * boneTransform.rotation;

                //矫正误差
                Matrix4x4 newLocalToWorldMatrix = Matrix4x4.TRS(boneTransform.position, targetRotation, boneTransform.lossyScale);
                Vector3 newAxis = newLocalToWorldMatrix * bone.m_Axis;
                Quaternion offset = Quaternion.FromToRotation(newAxis, axis);
                targetRotation = offset * targetRotation;

                //boneTransform.rotation = targetRotation;

                boneTransform.rotation = Quaternion.Slerp(
                    boneTransform.rotation, targetRotation,
                    1 - Mathf.Exp(-m_Speed * Time.deltaTime));

                // 角度控制
                Vector3 boneDir = (boneTransform.rotation * Vector3.right).normalized;
                Vector3 parentBoneDir = (boneTransform.parent.rotation * Vector3.right).normalized;
                float angle = Vector3.SignedAngle(parentBoneDir, boneDir, axis);
                float lerpAngle = (angle - bone.m_MaxAngle) / angle;
                Vector3 clampDir = Vector3.Slerp(boneDir, parentBoneDir, lerpAngle);

                //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * boneDir, Color.red);
                //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * parentBoneDir, Color.yellow);
                //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * clampDir, Color.green);

                boneTransform.rotation = Quaternion.FromToRotation(boneDir, clampDir) * boneTransform.rotation;
            }

            for (int j = m_BoneList.Length - 2; j >= 0; j--)
            {
                var boneTransform = m_BoneList[j].m_Transform;

                Vector3 bone2Effector = m_EffectorPoint.position - boneTransform.position;
                Vector3 bone2Target = m_TargetPoint.position - boneTransform.position;
                Quaternion rotation = Quaternion.FromToRotation(bone2Effector, bone2Target);
                Vector3 axis = boneTransform.localToWorldMatrix * m_BoneList[j].m_Axis;
                //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * axis, Color.black);

                // 固定在一个方向上旋转
                Vector3 quaternionAxis = new Vector3(rotation.x, rotation.y, rotation.z);
                Vector3 projected = Vector3.ProjectOnPlane(quaternionAxis, axis);
                Quaternion twist = new Quaternion(projected.x, projected.y, projected.z, rotation.w).normalized;
                Quaternion swing = rotation * Quaternion.Inverse(twist);

                Quaternion targetRotation = swing * boneTransform.rotation;

                //矫正误差
                Matrix4x4 newLocalToWorldMatrix = Matrix4x4.TRS(boneTransform.position, targetRotation, boneTransform.lossyScale);
                Vector3 newAxis = newLocalToWorldMatrix * m_BoneList[j].m_Axis;
                Quaternion offset = Quaternion.FromToRotation(newAxis, axis);
                targetRotation = offset * targetRotation;

                //boneTransform.rotation = targetRotation;

                boneTransform.rotation = Quaternion.Slerp(
                    boneTransform.rotation, targetRotation,
                    1 - Mathf.Exp(-m_Speed * Time.deltaTime));

                // 角度控制
                Vector3 boneDir = (boneTransform.rotation * Vector3.right).normalized;
                Vector3 parentBoneDir = (boneTransform.parent.rotation * Vector3.right).normalized;
                float angle = Vector3.SignedAngle(parentBoneDir, boneDir, axis);
                float lerpAngle = (angle - m_BoneList[j].m_MaxAngle) / angle;
                Vector3 clampDir = Vector3.Slerp(boneDir, parentBoneDir, lerpAngle);

                //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * boneDir, Color.red);
                //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * parentBoneDir, Color.yellow);
                //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * clampDir, Color.green);

                boneTransform.rotation = Quaternion.FromToRotation(boneDir, clampDir) * boneTransform.rotation;
            }

            //m_BoneList[m_BoneList.Length - 1].transform.rotation = 

            if (Vector3.Distance(m_EffectorPoint.position, m_TargetPoint.position) < 0.05f)
                break;
        }
    }
}
