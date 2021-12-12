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
        //Transform[] _randomTargetsInitPos;

        float _twistMin, _twistMax;
        float _swingMin, _swingMax;

        private int maxIterations = 10;
        private int currentIteration = 0;
        private float threshold = 0.01f;
        private bool shooted = false;
        private int closestIndex = 0;

        #region public methods
        //DO NOT CHANGE THE PUBLIC METHODS!!

        public float TwistMin { set => _twistMin = value; }
        public float TwistMax { set => _twistMax = value; }
        public float SwingMin {  set => _swingMin = value; }
        public float SwingMax { set => _swingMax = value; }
        

        public void TestLogging(string objectName)
        {

           
            Debug.Log("hello, I am initializing my Octopus Controller in object "+objectName);

            
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
            //_randomTargetsInitPos = _randomTargets;
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
            closestIndex = GetClosestTentacle();
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

        private int GetClosestTentacle()
        {
            float[] distances = new float[4];
            distances[0] = Vector3.Distance(_tentacles[0]._endEffectorSphere.position, _currentRegion.position);
            distances[1] = Vector3.Distance(_tentacles[1]._endEffectorSphere.position, _currentRegion.position);
            distances[2] = Vector3.Distance(_tentacles[2]._endEffectorSphere.position, _currentRegion.position);
            distances[3] = Vector3.Distance(_tentacles[3]._endEffectorSphere.position, _currentRegion.position);

            float minDistance = distances[0];
            int index = 0;

            for(int i = 1; i < distances.Length; i++)
            {
                if(minDistance > distances[i])
                {
                    minDistance = distances[i];
                    index = i;
                }
            }
            return index;
        }

        private void update_ccd() {
            currentIteration = 0;
            for(int i = 0; i < _tentacles.Length; i++)
            {
                
                //movemos al tentaculo mas cerca de
                //la zona del target en el caso que haya disparado
                if (shooted && closestIndex == i)
                {
                    if(Vector3.Distance(_tentacles[i]._endEffectorSphere.position, _currentRegion.position) > threshold && currentIteration < maxIterations)
                    {
                        for(int j = _tentacles[i].Bones.Length - 1; j >= 0; j--)
                        {
                            Vector3 vecEndEffector = _tentacles[i]._endEffectorSphere.position - _tentacles[i].Bones[j].position;
                            Vector3 vecTarget = _currentRegion.position - _tentacles[i].Bones[j].position;

                            float theta = Mathf.Acos(Vector3.Dot(vecEndEffector, vecTarget) / (vecEndEffector.magnitude * vecTarget.magnitude));
                            Vector3 rotationAxis = Vector3.Cross(vecEndEffector, vecTarget) / (vecEndEffector.magnitude * vecTarget.magnitude);


                            _tentacles[i].Bones[j].Rotate(rotationAxis, theta, Space.World);
                        }
                        currentIteration++;
                    }
                    if(i<_tentacles.Length)
                        i++;
                }


                //movemos los demas tentaculos o a todos en el caso que aun no le haya dado
                if(Vector3.Distance(_tentacles[i]._endEffectorSphere.position, _randomTargets[i].position) > threshold && currentIteration < maxIterations)
                {
                    for(int j = _tentacles[i].Bones.Length - 1; j >= 0; j--)
                    {
                        Vector3 vecEndEffector = _tentacles[i]._endEffectorSphere.position - _tentacles[i].Bones[j].position;
                        Vector3 vecTarget = _randomTargets[i].position - _tentacles[i].Bones[j].position;

                        float theta = Mathf.Acos(Vector3.Dot(vecEndEffector, vecTarget) / (vecEndEffector.magnitude * vecTarget.magnitude));

                        Vector3 rotationAxis = Vector3.Cross(vecEndEffector, vecTarget) / (vecEndEffector.magnitude * vecTarget.magnitude);


                        //float Mtwist = Mathf.Cos(_twistMax / 2);
                        //float mtwist = Mathf.Cos(_twistMin / 2);
                        //float Mswing = Mathf.Cos(_swingMax / 2);
                        //float mswing = Mathf.Cos(_swingMin / 2);
                        _tentacles[i].Bones[j].Rotate(rotationAxis, theta, Space.World);

                        /*
                        Quaternion T, S;

                        DecomposeSwingTwist(_tentacles[i].Bones[j].rotation, new Vector3(0, 1f, 0), out S, out T);

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
                        
                        //Vector3 TR = Vector3.Project(new Vector3(0, 0, 1), _tentacles[i].Bones[j].rotation.eulerAngles);
                        //T.Normalize();
                        //S = Quaternion.Inverse(T) * _tentacles[i].Bones[j].rotation;


                        Quaternion finalRot = S * T;

                        _tentacles[i].Bones[j].rotation = T * S;
                        */
                        /*
                        Quaternion S = new Quaternion(0, 0, 0, 1);
                        Quaternion T = Quaternion.Normalize(new Quaternion(0, _tentacles[i].Bones[j].rotation.y, 0, _tentacles[i].Bones[j].rotation.w));
                        S = Quaternion.Inverse(T) * _tentacles[i].Bones[j].rotation;
                        S.Normalize();

                        float testAngle;
                        Vector3 testAxis;

                        S.ToAngleAxis(out testAngle, out testAxis);
                        //testAngle = Mathf.Clamp(testAngle, minAngle, maxAngle);

                        Vector3 twistAxis;
                        float twistAngle;

                        T.ToAngleAxis(out twistAngle, out twistAxis);

                        if (twistAngle > _twistMax)
                            twistAngle = _twistMax;
                        if (twistAngle < _twistMin)
                            twistAngle = _twistMin;

                        T = Quaternion.AngleAxis(twistAngle, twistAxis);

                        //Debug.Log(T.ToString());
                        _tentacles[i].Bones[j].rotation = T * Quaternion.AngleAxis(testAngle, testAxis);
                        */


                    }
                    currentIteration++;
                }
                currentIteration = 0;
            }

        }

        private void DecomposeSwingTwist(
            Quaternion q,
            Vector3 twistAxis,
            out Quaternion swing,
            out Quaternion twist)
        {
            Vector3 r = new Vector3(q.x, q.y, q.z);

            if(r.sqrMagnitude < Mathf.Epsilon)
            {
                Vector3 rotatedTwistAxis = q * twistAxis;
                Vector3 swingAxis = Vector3.Cross(twistAxis, rotatedTwistAxis);

                if(swingAxis.sqrMagnitude < Mathf.Epsilon)
                {
                    float swingAngle = Vector3.Angle(twistAxis, rotatedTwistAxis);
                    swing = Quaternion.AngleAxis(swingAngle, swingAxis);
                }
                else
                {
                    //si el eje de rotacion es paralelo al eje de twist
                    swing = Quaternion.identity;
                }

                twist = Quaternion.AngleAxis(180f, twistAxis);
                return;
            }

            Vector3 p = Vector3.Project(r, twistAxis);
            twist = new Quaternion(p.x, p.y, p.z, q.w);
            twist.Normalize();// = Quaternion.Normalize(twist);
            swing = q * Quaternion.Inverse(twist);
        }        

        #endregion






    }
}
