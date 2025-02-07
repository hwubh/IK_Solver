using UnityEngine;

public class BoneProperty : MonoBehaviour
{
    [SerializeField] internal float m_MaxAngle;
    [SerializeField] internal float m_MinAngle;
    [SerializeField] internal Vector3 m_Axis = Vector3.forward;
    internal GameObject m_Go;
    internal Transform m_Transform;

    void Start()
    {
        m_Go = this.gameObject;
        m_Transform = m_Go.transform;
    }
}
