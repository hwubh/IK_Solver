using UnityEngine;


public class IK_Solver : MonoBehaviour
{
    public Transform[] m_BoneList;
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
            for (int j = 0; j < m_BoneList.Length; j++)
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
                //m_BoneList[j].rotation = swing * m_BoneList[j].rotation;
                m_BoneList[j].localRotation = Quaternion.Slerp(
                    m_BoneList[j].localRotation, swing * m_BoneList[j].rotation,
                    1 - Mathf.Exp(-m_Speed * Time.deltaTime));
            }
            if (Vector3.Distance(m_EffectorPoint.position, m_TargetPoint.position) < m_Threshold)
                break;
        }
    }
}
