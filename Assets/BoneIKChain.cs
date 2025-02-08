using System.Collections.Generic;
using UnityEngine;

public class BoneIKChain : MonoBehaviour
{
    List<BoneProperty> m_Bones = new List<BoneProperty>();
    internal BoneProperty[] m_BoneList;
    internal Transform m_EffectorPoint;
    public Transform m_TargetPoint;

    void OnEnable()
    {
        var go = this.gameObject;
        while (go && go.GetComponent<BoneProperty>())
        {
            m_Bones.Add(go.GetComponent<BoneProperty>());
            go = go.transform.GetChild(0).gameObject;
        }
        m_EffectorPoint = go.transform;

        m_BoneList = m_Bones.ToArray();
    }
}
