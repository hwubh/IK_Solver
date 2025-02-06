using UnityEngine;

public class BoneProperty : MonoBehaviour
{
    [SerializeField] internal float m_MaxAngle;
    [SerializeField] internal float m_MinAngle;
    internal GameObject m_Go;
    internal Transform m_Transform;

    void Start()
    {
        m_Go = this.gameObject;
        m_Transform = m_Go.transform;
    }
}
