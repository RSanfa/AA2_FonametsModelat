using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


namespace OctopusController
{

    public class MyScorpionController
    {
        //TAIL
        Transform tailTarget;
        Transform tailEndEffector;
        MyTentacleController _tail;
        float animationRange = 0;
        float currentDistance = 0;
        bool closeEnough = false;

        //Gradiant descent Tail
        Vector3[] TailJointAxis;
        Vector3[] TailJointInitPos;
        Vector3[] TailJointInitRot;
        float[] TailJointAngles;
        float[] TailJointMinAngle;
        float[] TailJointMaxAngle;

        //Inverse Kinematics
        float deltaGradient = 0.1f;
        float learningRate = 25f;
        float stopThreshold = 0.1f;

        //LEGS
        Transform[] legTargets;
        Transform[] legFutureBases;
        MyTentacleController[] _legs = new MyTentacleController[6];


        #region public

        public void InitLegs(ref Transform[] LegRoots, ref Transform[] LegFutureBases, ref Transform[] LegTargets)
        {
            _legs = new MyTentacleController[LegRoots.Length];
            //Legs init
            for (int i = 0; i < LegRoots.Length; i++)
            {
                _legs[i] = new MyTentacleController();
                _legs[i].LoadTentacleJoints(LegRoots[i], TentacleMode.LEG);
                //TODO: initialize anything needed for the FABRIK implementation
            }

        }

        public void InitTail(ref Transform TailBase)
        {
            _tail = new MyTentacleController();
            _tail.LoadTentacleJoints(TailBase, TentacleMode.TAIL);
            tailEndEffector = _tail._endEffectorSphere;
            //TODO: Initialize anything needed for the Gradient Descent implementation
            //we calculate the total length of the tail.
            for (int i = 0; i < _tail.Bones.Length - 1; i++)
            {
                Vector3 tmpVec = _tail.Bones[i].localPosition - _tail.Bones[i + 1].localPosition;
                animationRange += tmpVec.magnitude;
            }
            //Vector3 tmp = _tail.Bones[_tail.Bones.Length - 1].localPosition - _tail._endEffectorSphere.localPosition;
            //animationRange += tmp.magnitude;

            TailJointAxis = new Vector3[6];
            TailJointAxis[0] = new Vector3(0, 0, 1);
            TailJointAxis[1] = new Vector3(1, 0, 0);
            TailJointAxis[2] = new Vector3(1, 0, 0);
            TailJointAxis[3] = new Vector3(1, 0, 0);
            TailJointAxis[4] = new Vector3(1, 0, 0);
            TailJointAxis[5] = new Vector3(0, 0, 0);

            TailJointInitPos = new Vector3[6];
            TailJointInitPos[0] = _tail.Bones[0].localPosition;
            TailJointInitPos[1] = _tail.Bones[1].localPosition;
            TailJointInitPos[2] = _tail.Bones[2].localPosition;
            TailJointInitPos[3] = _tail.Bones[3].localPosition;
            TailJointInitPos[4] = _tail.Bones[4].localPosition;
            TailJointInitPos[5] = _tail.Bones[5].localPosition;

            TailJointInitRot = new Vector3[6];
            TailJointInitRot[0] = _tail.Bones[0].localEulerAngles;
            TailJointInitRot[1] = _tail.Bones[1].localEulerAngles;
            TailJointInitRot[2] = _tail.Bones[2].localEulerAngles;
            TailJointInitRot[3] = _tail.Bones[3].localEulerAngles;
            TailJointInitRot[4] = _tail.Bones[4].localEulerAngles;
            TailJointInitRot[5] = _tail.Bones[5].localEulerAngles;

            TailJointMinAngle = new float[6];
            TailJointMinAngle[0] = -180f;
            TailJointMinAngle[1] = -160f;
            TailJointMinAngle[2] = -100f;
            TailJointMinAngle[3] = -100f;
            TailJointMinAngle[4] = -20f;
            TailJointMinAngle[5] = -0f;

            TailJointMaxAngle = new float[6];
            TailJointMaxAngle[0] = 180f;
            TailJointMaxAngle[1] = 160f;
            TailJointMaxAngle[2] = 100f;
            TailJointMaxAngle[3] = 100f;
            TailJointMaxAngle[4] = 20f;
            TailJointMaxAngle[5] = 0f;

            TailJointAngles = new float[6];

        }

        //TODO: Check when to start the animation towards target and implement Gradient Descent method to move the joints.
        public void NotifyTailTarget(Transform target)
        {
            tailEndEffector = _tail._endEffectorSphere;
            tailTarget = target;
            currentDistance = Vector3.Distance(tailTarget.position, tailEndEffector.position);
            if (currentDistance < animationRange && currentDistance > stopThreshold)
            {
                //empezar la animacion
                //Debug.Log("se empieaza la animacion de la cola");
                //UpdateIK();
                closeEnough = true;
            }
        }

        //TODO: Notifies the start of the walking animation
        public void NotifyStartWalk()
        {

        }

        //TODO: create the apropiate animations and update the IK from the legs and tail

        public void UpdateIK()
        {
            //por ahora solo queremos mover la cola: ejercicio 3
            if (closeEnough)
            {
                //Debug.Log("angles: ");
                //Debug.Log("of: " + _tail.Bones[0].name + " " + TailJointInitRot[0]);
                //Debug.Log("of: " + _tail.Bones[1].name + " " + TailJointInitRot[1]);
                //Debug.Log("of: " + _tail.Bones[2].name + " " + TailJointInitRot[2]);
                //Debug.Log("of: " + _tail.Bones[3].name + " " + TailJointInitRot[3]);
                //Debug.Log("of: " + _tail.Bones[4].name + " " + TailJointInitRot[4]);
                //Debug.Log("current distance: " + currentDistance);
                //Debug.Log("target: " + tailTarget.localPosition);
                //Debug.Log("end effector: " + _tail._endEffectorSphere.localPosition);
                //Debug.Log("end effector: " + tailEndEffector.position);


                //Debug.Break();
                updateTail();
            }
        }
        #endregion


        #region private
        //TODO: Implement the leg base animations and logic
        private void updateLegPos()
        {
            //check for the distance to the futureBase, then if it's too far away start moving the leg towards the future base position
            //
        }
        //TODO: implement Gradient Descent method to move tail if necessary
        private void updateTail()
        {
            for (int i = _tail.Bones.Length - 1; i >= 0; i--)
            {
                float gradient = CalculateGradient(TailJointAngles, i, deltaGradient);
                TailJointAngles[i] -= learningRate * gradient;

                //puede que esto este mal
                //TailJointAngles[i] = Mathf.Clamp(TailJointAngles[i] + deltaGradient, TailJointMinAngle[i], TailJointMaxAngle[i]);
                _tail.Bones[i].eulerAngles = TailJointAngles[i] * TailJointAxis[i];
            }
        }

        private float CalculateGradient(float[] angles, int i, float delta)
        {
            float angle = angles[i];
            float f_x = DistanceFromTarget(angles);
            angles[i] += delta;
            float f_x_plus_d = DistanceFromTarget(angles);
            float gradient = (f_x_plus_d - f_x) / delta;

            angles[i] = angle;
            return gradient;
        }

        private float DistanceFromTarget(float[] angles)
        {
            Vector3 point = ForwardKinematics(angles);
            //Debug.Log("current endeffector pos: " + tailEndEffector.position);
            //Debug.Log("endeff from func: " + point);
            //Debug.Log(Vector3.Distance(point, tailTarget.position));
            //Debug.Break();
            return Vector3.Distance(point, tailTarget.position);
        }

        //en teoria esto funciona bien
        private Vector3 ForwardKinematics(float[] angles)
        {
            Vector3 prevPoint = _tail.Bones[0].position;

            Quaternion rotation = new Quaternion(0, 0, 0, 1);


            for (int i = 1; i < _tail.Bones.Length - 1; i++)
            {
                rotation *= Quaternion.AngleAxis(TailJointAngles[i - 1], TailJointAxis[i - 1]);
                Vector3 nextPoint = prevPoint + rotation * TailJointInitPos[i];
                prevPoint = nextPoint;
            }

            return prevPoint;
        }

        //TODO: implement fabrik method to move legs 
        private void updateLegs()
        {

        }
        #endregion
    }
}
