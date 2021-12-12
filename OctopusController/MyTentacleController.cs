using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;




namespace OctopusController
{

    
    internal class MyTentacleController

    //MAINTAIN THIS CLASS AS INTERNAL
    {

        TentacleMode tentacleMode;
        Transform[] _bones;
        public Transform _endEffectorSphere;

        public Transform[] Bones { get => _bones; }

        void GetJointLeg(GameObject go, bool first, ref List<Transform> bones)
        {
            if (!first)
                bones.Add(go.transform);
            foreach (Transform t in go.transform)
            {                
                if (t.GetComponent<MeshRenderer>() == null)
                {
                    if (t.childCount > 0)
                        GetJointLeg(t.gameObject, false, ref bones);
                    else
                        _endEffectorSphere = t;
                }
            }
        }

        void GetJointTail(GameObject go, ref List<Transform> bones)
        {
            bones.Add(go.transform);
            //Debug.Log("from: " + go.name);
            foreach(Transform t in go.transform)
            {
                //Debug.Log("childrens: " + t.name);
                if (t.GetComponent<MeshRenderer>() == null)
                {
                    if (t.childCount > 0)
                        GetJointTail(t.gameObject, ref bones);
                    else
                    {
                        _endEffectorSphere = t;
                        bones.Add(t.transform);
                    }
                }
            }
        }

        void GetJointTentacle(GameObject go, ref List<Transform> bones)
        {
            foreach(Transform t in go.transform)
            {
                //if(t.GetComponent<Rigidbody>() == null)
                //{
                    if (t.GetChild(0).GetComponent<Rigidbody>() != null)
                        _endEffectorSphere = t;
                    else
                    {
                        bones.Add(t);
                        GetJointTentacle(t.gameObject, ref bones);
                    }
                //}
            }
        }

        //Exercise 1.
        public Transform[] LoadTentacleJoints(Transform root, TentacleMode mode)
        {
            //TODO: add here whatever is needed to find the bones forming the tentacle for all modes
            //you may want to use a list, and then convert it to an array and save it into _bones
            tentacleMode = mode;
            List<Transform> _boneTmp = new List<Transform>();

            switch (tentacleMode){
                case TentacleMode.LEG:
                    //TODO: in _endEffectorsphere you keep a reference to the base of the leg
                    GetJointLeg(root.gameObject, true, ref _boneTmp);
                    break;
                case TentacleMode.TAIL:
                    GetJointTail(root.gameObject, ref _boneTmp);
                    //TODO: in _endEffectorsphere you keep a reference to the red sphere 
                    break;
                case TentacleMode.TENTACLE:
                    //TODO: in _endEffectorphere you  keep a reference to the sphere with a collider attached to the endEffector
                    Transform child1 = root.GetChild(0).transform;
                    Transform child2 = child1.GetChild(0).transform;
                    GetJointTentacle(child2.gameObject, ref _boneTmp);
                    break;
            }
            _bones = new Transform[_boneTmp.Count];

            for(int i = 0; i < _boneTmp.Count; i++)
            {
                _bones[i] = _boneTmp[i];
            }
            return Bones;
        }
    }
}
