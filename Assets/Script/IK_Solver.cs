using UnityEngine;

public class IK_Solver : MonoBehaviour
{
    public Transform[] m_BoneList;
    public Transform m_EffectorPoint;
    public Transform m_TargetPoint;
    public int m_MaxInteration = 10;
    public float m_Threshold = 1.0f;

    void CCDIk() 
    {
        for (int i = 0; i < m_MaxInteration; i++) 
        {
            for (int j = 0; j < m_BoneList.Length; j++)
            {
                Vector3 bone2Effector = m_EffectorPoint.position - m_BoneList[i].position;
                Vector3 bone2Tartget = m_TargetPoint.position - m_BoneList[i].position;
                Quaternion rotation = Quaternion.FromToRotation(bone2Effector, bone2Tartget);
                m_BoneList[i].rotation *= rotation;
            }
            if (Vector3.Distance(m_EffectorPoint.position, m_TargetPoint.position) < m_Threshold)
                break;
        }
    }
}
