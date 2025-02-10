using System.Collections.Generic;
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

    private Vector3 Test0;
    private Vector3 Test1;

    private void Update()
    //private void Start()
    {
        foreach (var go in m_RootGameObjects) 
        {
            var chain = go.GetComponent<BoneIKChain>();
            m_BoneList = chain.m_BoneList;
            m_EffectorPoint = chain.m_EffectorPoint;
            m_TargetPoint = chain.m_TargetPoint;

            //var pos = m_BoneList[m_BoneList.Length - 1].transform.position;
            //var length = (pos - m_EffectorPoint.position).magnitude;

            ////var ray = new Ray(m_BoneList[m_BoneList.Length - 1].transform.position + Vector3.up * 10, Vector3.down);
            //if (Physics.SphereCast(pos, 0.01f, Vector3.down, out m_Hit))
            //{
            //    m_TargetPoint.position = new Vector3(m_Hit.point.x, m_Hit.point.y, m_Hit.point.z);
            //}
            //Debug.DrawLine(m_TargetPoint.position, m_TargetPoint.position + m_Hit.normal * 10f, Color.yellow);

            FABRIK();
        }
    }

    void CCDIk() 
    {
        for (int i = 0; i < m_MaxInteration; i++)
        {
            for (int j = m_BoneList.Length - 2; j >= 0; j--)
            {
                var boneTransform = m_BoneList[j].m_Transform;

                // 计算旋转的角度
                Vector3 bone2Effector = m_EffectorPoint.position - boneTransform.position;
                Vector3 bone2Target = m_TargetPoint.position - boneTransform.position;
                Quaternion rotation = Quaternion.FromToRotation(bone2Effector, bone2Target);

                // 固定在一个方向上旋转
                Vector3 axis = boneTransform.localToWorldMatrix * m_BoneList[j].m_Axis;
                //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * axis, Color.black);
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

            {
            //    var bone = m_BoneList[m_BoneList.Length - 1];
            //    var boneTransform = bone.m_Transform;
            //    Quaternion rotation = Quaternion.FromToRotation(boneTransform.localToWorldMatrix * Vector3.right, m_Hit.normal);
            //    Vector3 axis = boneTransform.localToWorldMatrix * bone.m_Axis;
            //    //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * axis, Color.black);

            //    // 固定在一个方向上旋转
            //    Vector3 quaternionAxis = new Vector3(rotation.x, rotation.y, rotation.z);
            //    Vector3 projected = Vector3.ProjectOnPlane(quaternionAxis, axis);
            //    Quaternion twist = new Quaternion(projected.x, projected.y, projected.z, rotation.w).normalized;
            //    Quaternion swing = rotation * Quaternion.Inverse(twist);

            //    Quaternion targetRotation = swing * boneTransform.rotation;

            //    //矫正误差
            //    Matrix4x4 newLocalToWorldMatrix = Matrix4x4.TRS(boneTransform.position, targetRotation, boneTransform.lossyScale);
            //    Vector3 newAxis = newLocalToWorldMatrix * bone.m_Axis;
            //    Quaternion offset = Quaternion.FromToRotation(newAxis, axis);
            //    targetRotation = offset * targetRotation;

            //    //boneTransform.rotation = targetRotation;

            //    boneTransform.rotation = Quaternion.Slerp(
            //        boneTransform.rotation, targetRotation,
            //        1 - Mathf.Exp(-m_Speed * Time.deltaTime));

            //    // 角度控制
            //    Vector3 boneDir = (boneTransform.rotation * Vector3.right).normalized;
            //    Vector3 parentBoneDir = (boneTransform.parent.rotation * Vector3.right).normalized;
            //    float angle = Vector3.SignedAngle(parentBoneDir, boneDir, axis);
            //    float lerpAngle = (angle - bone.m_MaxAngle) / angle;
            //    Vector3 clampDir = Vector3.Slerp(boneDir, parentBoneDir, lerpAngle);

            //    //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * boneDir, Color.red);
            //    //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * parentBoneDir, Color.yellow);
            //    //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * clampDir, Color.green);

            //    boneTransform.rotation = Quaternion.FromToRotation(boneDir, clampDir) * boneTransform.rotation;
            }

            if (Vector3.Distance(m_EffectorPoint.position, m_TargetPoint.position) < 0.05f)
                break;
        }
    }

    bool isOverMaxAngle(BoneProperty endBone, BoneProperty startBone) 
    {
        Vector3 endToTarget = m_TargetPoint.position - endBone.transform.position;
        Vector3 startToEnd = endBone.transform.position - startBone.transform.position;
        Vector3 axis = endBone.transform.localToWorldMatrix * endBone.m_Axis;
        float angle = Vector3.SignedAngle(endToTarget, startToEnd, axis);

        return angle > 180 - endBone.m_MaxAngle;
    }

    void FABRIK() 
    {
        int BonesListSize = m_BoneList.Length;

        // 记录各个关节的位置
        Vector3[] bonesPosition = new Vector3[BonesListSize + 1];
        for (int i = 0; i < BonesListSize; i++)
        {
            bonesPosition[i] = m_BoneList[i].transform.position;
        }
        bonesPosition[BonesListSize] = m_EffectorPoint.position;

        // 计算各个关节之间的距离
        float[] bonesLength = new float[BonesListSize];
        for (int i = 0; i < BonesListSize - 1; i++) 
        {
            bonesLength[i] = Vector3.Distance(bonesPosition[i], bonesPosition[i + 1]); // (m_BoneList[i].transform.position - m_BoneList[i + 1].transform.position).magnitude;
        }
        bonesLength[BonesListSize - 1] = Vector3.Distance(bonesPosition[BonesListSize - 1], bonesPosition[BonesListSize]); //  (m_BoneList[BonesListSize - 1].transform.position - m_EffectorPoint.position).magnitude;

        // 检查目标是否可以到达
        float allBonesLength = 0f;
        foreach (var length in bonesLength) 
            allBonesLength += length;
        float rootToTargetLength = (bonesPosition[0] - m_TargetPoint.position).magnitude;
        if (rootToTargetLength > allBonesLength)
            return;

        // 开始迭代
        for (int i = 0; i < 1; i++)
        {
            Quaternion rotation = Quaternion.identity;

            // 逆向遍历: 更新各个关节的空间位置
            bonesPosition[BonesListSize] = m_TargetPoint.position;
            Vector3 normalizedEndToStart = Vector3.one;
            //Vector3 normalizedEndToStart = (bonesPosition[BonesListSize - 1] - bonesPosition[BonesListSize]).normalized;
            //bonesPosition[BonesListSize - 1] = bonesPosition[BonesListSize] + bonesLength[BonesListSize - 1] * normalizedEndToStart;

            bool isSkip = false;
            bool test = true;

            for (int j = BonesListSize - 1; j > 0; j--) 
            {
                // 根据角度限制，规划当前迭代的骨骼信息。
                // 如果存在target目标超出了该关节的最大角度，则将该关节与其父关节一起处理。
                isSkip = isSkip ? isSkip : !isOverMaxAngle(m_BoneList[j], m_BoneList[j - 1]);
                if (!isSkip)
                {
                    if (j == BonesListSize - 1) 
                    {
                        bonesPosition[j] = bonesPosition[j + 1] - m_BoneList[j].transform.rotation * m_EffectorPoint.localPosition;
                        continue;
                    }
                    bonesPosition[j] = bonesPosition[j + 1] + m_BoneList[j].transform.rotation * m_BoneList[j + 1].transform.localPosition;
                    continue;
                }
                if (test) 
                {
                    normalizedEndToStart = (bonesPosition[j] - m_TargetPoint.position).normalized;
                    bonesPosition[j] = m_TargetPoint.position + (bonesPosition[j] - m_EffectorPoint.position).magnitude * normalizedEndToStart;
                    test = false;
                    continue;
                }
                normalizedEndToStart = (bonesPosition[j] - bonesPosition[j + 1]).normalized;
                bonesPosition[j] = bonesPosition[j + 1] + bonesLength[j] * normalizedEndToStart;
            }

            // 正向遍历: 调整各个关节的相对位置(保持关节长度不变)
            //Vector3 normalizedStartToEnd = Vector3.zero;
            //for (int j = 0; j < BonesListSize - 1; j++)
            //{
            //    normalizedStartToEnd = (bonesPosition[j + 1] - bonesPosition[j]).normalized;
            //    bonesPosition[j + 1] = bonesPosition[j] + bonesLength[j] * normalizedStartToEnd;
            //}
            //normalizedStartToEnd = (bonesPosition[BonesListSize] - bonesPosition[BonesListSize - 1]).normalized;
            //bonesPosition[BonesListSize] = bonesPosition[BonesListSize - 1] + bonesLength[BonesListSize - 1] * normalizedStartToEnd;

            // 调整关节朝向
            for (int j = 0; j < BonesListSize; j++)
            {
                //Debug.DrawLine(bonesPosition[j], bonesPosition[j + 1], Color.black);
                Debug.DrawLine(bonesPosition[BonesListSize - 1], bonesPosition[BonesListSize], Color.red);
                Debug.DrawLine(bonesPosition[BonesListSize - 1 - 1], bonesPosition[BonesListSize - 1], Color.yellow);
                Debug.DrawLine(bonesPosition[BonesListSize - 1 - 1 - 1], bonesPosition[BonesListSize - 1 - 1], Color.blue);
                Debug.DrawLine(bonesPosition[BonesListSize - 1 - 1 - 1 - 1], bonesPosition[BonesListSize - 1 - 1 - 1], Color.green);
            }

            //for (int j = 0; j < BonesListSize; j++) 
            //{
            //    var boneTransform = m_BoneList[j].m_Transform;

            //    rotation = Quaternion.FromToRotation(m_BoneList[j].transform.localToWorldMatrix * Vector3.left, bonesPosition[j + 1] - bonesPosition[j]);
            //    //m_BoneList[j].transform.rotation = rotation * m_BoneList[j].transform.rotation;

            //    // 固定在一个方向上旋转
            //    Vector3 axis = boneTransform.localToWorldMatrix * m_BoneList[j].m_Axis;
            //    //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * axis, Color.black);
            //    Vector3 quaternionAxis = new Vector3(rotation.x, rotation.y, rotation.z);
            //    Vector3 projected = Vector3.ProjectOnPlane(quaternionAxis, axis);
            //    Quaternion twist = new Quaternion(projected.x, projected.y, projected.z, rotation.w).normalized;
            //    Quaternion swing = rotation * Quaternion.Inverse(twist);

            //    Quaternion targetRotation = swing * boneTransform.rotation;

            //    //矫正误差
            //    Matrix4x4 newLocalToWorldMatrix = Matrix4x4.TRS(boneTransform.position, targetRotation, boneTransform.lossyScale);
            //    Vector3 newAxis = newLocalToWorldMatrix * m_BoneList[j].m_Axis;
            //    Quaternion offset = Quaternion.FromToRotation(newAxis, axis);
            //    targetRotation = offset * targetRotation;

            //    //boneTransform.rotation = targetRotation;

            //    boneTransform.rotation = Quaternion.Slerp(
            //        boneTransform.rotation, targetRotation,
            //        1 - Mathf.Exp(-m_Speed * Time.deltaTime));

            //    // 角度控制
            //    Vector3 boneDir = (boneTransform.rotation * Vector3.right).normalized;
            //    Vector3 parentBoneDir = (boneTransform.parent.rotation * Vector3.right).normalized;
            //    float angle = Vector3.SignedAngle(parentBoneDir, boneDir, axis);
            //    float lerpAngle = (angle - m_BoneList[j].m_MaxAngle) / angle;
            //    Vector3 clampDir = Vector3.Slerp(boneDir, parentBoneDir, lerpAngle);

            //    //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * boneDir, Color.red);
            //    //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * parentBoneDir, Color.yellow);
            //    //Debug.DrawLine(boneTransform.position, boneTransform.position + 10 * clampDir, Color.green);

            //    boneTransform.rotation = Quaternion.FromToRotation(boneDir, clampDir) * boneTransform.rotation;

            //}

            if (Vector3.Distance(m_EffectorPoint.position, m_TargetPoint.position) < 0.01f)
                break;
        }
    }

    //private void Update()
    //{
    //    Debug.DrawLine(Test0, Test0 + (Test1), Color.yellow);
    //}
}
