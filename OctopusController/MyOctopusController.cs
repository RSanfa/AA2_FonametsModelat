using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

//This code is created by Roger Sanfeliu & Kevin Elortza

namespace OctopusController
{
    public enum TentacleMode { LEG, TAIL, TENTACLE };

    public class MyOctopusController 
    {
        
        MyTentacleController[] _tentacles =new  MyTentacleController[4];

        Transform _currentRegion;
        Transform _target;

        Transform[] _randomTargets;// = new Transform[4];
        Transform[] _randomTargetsInitPos;

        float _twistMin, _twistMax;
        float _swingMin, _swingMax;

        private int maxIterations = 10;
        private int currentIteration = 0;
        private float threshold = 0.01f;
        private bool shooted = false;

        #region public methods
        //DO NOT CHANGE THE PUBLIC METHODS!!

        public float TwistMin { set => _twistMin = value; }
        public float TwistMax { set => _twistMax = value; }
        public float SwingMin {  set => _swingMin = value; }
        public float SwingMax { set => _swingMax = value; }
        

        public void TestLogging(string objectName)
        {

           
            Debug.Log("hello, I am initializing my Octopus Controller in object "+objectName);
            Debug.Log("aupa");

            
        }

        public void Init(ref Transform[] tentacleRoots, ref Transform[] randomTargets)
        {
            _tentacles = new MyTentacleController[tentacleRoots.Length];
            maxIterations = 10;
            currentIteration = 0;
            threshold = 0.01f;
            // foreach (Transform t in tentacleRoots)
            for(int i = 0;  i  < tentacleRoots.Length; i++)
            {

                _tentacles[i] = new MyTentacleController();
                _tentacles[i].LoadTentacleJoints(tentacleRoots[i],TentacleMode.TENTACLE);
                //TODO: initialize any variables needed in ccd
                //le tenemos que asignar una region a cada uno de los tentaculos para que cuando le demos a 
                //la pelota, unicamente el tentaculo de la region en la que se encuentra el target se desplace
            }

            _randomTargets = randomTargets;
            _randomTargetsInitPos = _randomTargets;
            //TODO: use the regions however you need to make sure each tentacle stays in its region
        }

              
        public void NotifyTarget(Transform target, Transform region)
        {
            _currentRegion = region;
            _target = target;
        }

        public void NotifyShoot() {
            //TODO. what happens here?
            shooted = true;
            Debug.Log("Shoot");
        }


        public void UpdateTentacles()
        {
            //TODO: implement logic for the correct tentacle arm to stop the ball and implement CCD method
            update_ccd();
        }




        #endregion


        #region private and internal methods
        //todo: add here anything that you need

        void update_ccd() {
            currentIteration = 0;
            for(int i = 0; i < _tentacles.Length; i++)
            {
                if(Vector3.Distance(_tentacles[i]._endEffectorSphere.position, _randomTargets[i].position) > threshold && currentIteration < maxIterations)
                {
                    for(int j = _tentacles[i].Bones.Length - 1; j >= 0; j--)
                    {
                        Vector3 vecEndEffector = _tentacles[i]._endEffectorSphere.position - _tentacles[i].Bones[j].position;
                        Vector3 vecTarget = _randomTargets[i].position - _tentacles[i].Bones[j].position;

                        float theta = Mathf.Acos(Vector3.Dot(vecEndEffector, vecTarget) / (vecEndEffector.magnitude * vecTarget.magnitude));
                        theta = theta % (2.0f * Mathf.PI);
                        theta += theta < -Mathf.PI ? 2.0f * Mathf.PI : -2.0f * Mathf.PI;
                        theta *= Mathf.Rad2Deg;

                        Vector3 rotationAxis = Vector3.Cross(vecEndEffector, vecTarget) / (vecEndEffector.magnitude * vecTarget.magnitude);


                        Quaternion R = _tentacles[i].Bones[j].rotation;
                        Vector3 twistAxis = new Vector3(0, 1, 0);
                        Vector3 vT = Vector3.Project(new Vector3(R.x, R.y, R.z), twistAxis);
                        Quaternion T = new Quaternion(vT.x, vT.y, vT.z, R.w);
                        T.Normalize();
                        Quaternion S = R * Quaternion.Inverse(T);

                        float Mtwist = Mathf.Cos(_twistMax / 2);
                        float mtwist = Mathf.Cos(_twistMin / 2);
                        float Mswing = Mathf.Cos(_swingMax / 2);
                        float mswing = Mathf.Cos(_swingMin / 2);

                        if (T.w > Mtwist)
                            T.w = Mtwist;
                        else if (T.w < mtwist)
                            T.w = mtwist;

                        if (S.w > Mswing)
                            S.w = Mswing;
                        else if (S.w < mswing)
                            S.w = mswing;

                        T.Normalize();
                        S.Normalize();

                        Quaternion finalRot = T * S;
                        //Quaternion.Euler(rotationAxis.x, rotationAxis.y, rotationAxis.z);
                        Quaternion.AngleAxis(theta, new Vector3(rotationAxis.x, rotationAxis.y, rotationAxis.z));

                        _tentacles[i].Bones[j].Rotate(rotationAxis, theta, Space.World);
                    }
                    currentIteration++;
                }
                currentIteration = 0;
            }

        }


        

        #endregion






    }
}
