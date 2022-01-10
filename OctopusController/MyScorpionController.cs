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
        bool walkAnimation = false;

        //Gradiant descent Tail
        #region Gradiant descent
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
        #endregion

        //LEGS
        Transform[] legTargets;
        Transform[] legFutureBases; 
        MyTentacleController[] _legs = new MyTentacleController[6];

        //Ejercicio 3
        float threshold = 0.0001f;
        float minAnimTreshold = 0.8f;
        float animDuration = 0.5f;//esto se tendra que ajustar para que la animacion se vea bien
        //guardamos la posicion de la futureBase en cuanto alcanzamos el umbral
        Vector3[] FixedFutureBase = new Vector3[6];
        //se guarda la posicion inicial del leg para poder resetear su pos enttre la fase forward y backward
        Vector3[] legsInitPos = new Vector3[6];
        //la posicion inicial que se le da en cuanto se hace el lerp entre currentPos y futureBase
        Vector3[] lerpInitPos = new Vector3[6];
        //la posicion final que se le da en cuanto se hace el lerp entre currentPos y futureBase
        Vector3[] lerpFinalPos = new Vector3[6];
        bool[] legsMoving = new bool[6];
        float[] legsLength = new float[6];
        float[] legsCurrentDuration = new float[6];

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
                legsMoving[i] = false;
                //aqui se inizializa la longitud de cada pata
                for(int j = 0; j < _legs[i].Bones.Length-1; j++)
                {
                    legsLength[i] += Vector3.Distance(_legs[i].Bones[j].position, _legs[i].Bones[j + 1].position);
                }
            }
            legFutureBases = LegFutureBases;
            legTargets = LegTargets;
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
            TailJointAxis[0] = new Vector3(0, 1, 0);
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
            walkAnimation = true;
        }

        //TODO: create the apropiate animations and update the IK from the legs and tail

        public void UpdateIK()
        {
            //por ahora solo queremos mover la cola: ejercicio 3
            if (closeEnough)
            {

                //Debug.Break();
                updateTail();
            }
            if (walkAnimation)
            {
                updateLegPos();

                //todo lo relacionado con el lerp
                for(int i = 0; i < _legs.Length; i++)
                {
                    //miramos si la pata ha alcanzado el umbral 
                    if(minAnimTreshold < Vector3.Distance(_legs[i].Bones[0].position, legFutureBases[i].position) && !legsMoving[i])
                    {
                        FixedFutureBase[i] = legFutureBases[i].position;
                        legsMoving[i] = true;
                        //no se si hay que poner el endEffector o la base pero diria que le tengo que pasar la base
                        lerpInitPos[i] = _legs[i].Bones[0].position;
                        legsCurrentDuration[i] = 0;
                    }

                    if (legsMoving[i])
                    {
                        //aqui se tiene que hacer el lerp.
                        _legs[i].Bones[0].position = Vector3.Lerp(lerpInitPos[i], FixedFutureBase[i], legsCurrentDuration[i]/animDuration);

                        //despues del lerp hay que mirar si la animacion ha terminado o no (mirando la distancia de la posicion actual y la de 
                        //la future base guardada
                        //en el caso que haya acabado se tiene que cambiar el valor de legsMoving[i]

                        legsCurrentDuration[i] += Time.fixedDeltaTime;
                        if(legsCurrentDuration[i] >= animDuration)
                        {
                            legsCurrentDuration[i] = 0;
                            legsMoving[i] = false;
                        }
                    }
                }
            }
        }
        #endregion


        #region private
        //TODO: implement fabrik method to move legs 

        private void updateLegPos()
        {
            //check for the distance to the futureBase, then if it's too far away start moving the leg towards the future base position
            //
            for (int i = 0; i < _legs.Length; i++)
            {
                //se calcula la distancia entre el origen y el target
                float dist = Vector3.Distance(_legs[i].Bones[0].position, legTargets[i].position);
                //se mira si la distancia es mas grande que la longitud del le

                if (dist > legsLength[i])
                {
                    //el target no se puede alcanzar
                    for (int j = 0; j < _legs[i].Bones.Length - 1; j++)
                    {
                        //se busca la distancia entre el joint j y el target i
                        float r = Vector3.Distance(_legs[i].Bones[j].position, legTargets[i].position);
                        float l = legsLength[j] / r;

                        //se calcula la nueva posicion del joint
                        _legs[i].Bones[j + 1].position = ((1 - l) * _legs[i].Bones[j].position) + (l * legTargets[i].position);
                    }
                }
                else
                {
                    //se puede alcanzar el target
                    /*legsInitPos[i] = _legs[i].Bones[0].position;
                    updateLegs(i);*/
                }
            }
        }

        private void updateLegs(int i)
        {
            int count = 15;
            float dist = threshold * 2;

            while (dist > threshold && count > 0)
            {
                //en primer lugar se le aplica la parte de forward
                forward(i);
                //en segundo lugar se le aplica la parte de backward
                backward(i);
                dist = Vector3.Distance(_legs[i].Bones[_legs[i].Bones.Length - 1].position, legTargets[i].position);
                count--;
            }
            Debug.Log("Ha salido del while");
            legsMoving[i] = false;
        }

        private void backward(int i)
        {
            _legs[i].Bones[0].position = legsInitPos[i];

            for(int j = 0; j < _legs[i].Bones.Length-1; j++)
            {
                //se calcula la distancia entre el nuevo joint j y el joint j+1
                float r = Vector3.Distance(_legs[i].Bones[j].position, _legs[i].Bones[j + 1].position);
                float l = legsLength[j] / r;

                //se calcula la nueva posicion del joint
                _legs[i].Bones[j + 1].position = (1 - l) * _legs[i].Bones[j].position + l * _legs[i].Bones[j + 1].position;
            }
        }

        private void forward(int i)
        {
            _legs[i].Bones[_legs[i].Bones.Length - 1].position = legTargets[i].position;

            for(int j = _legs[i].Bones.Length-2; j >= 0; j--)
            {
                //buscamos la distancia entre el nuevo joint j+1 y el joint j
                float r = Vector3.Distance(_legs[i].Bones[j + 1].position, _legs[i].Bones[j].position);
                float l = legsLength[j] / r;
                //se calcula la nueva posicion 
                _legs[i].Bones[j].position = (1 - l) * _legs[i].Bones[j + 1].position + l * _legs[i].Bones[j].position;
            }
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
            //Debug.Log("distance: " + DistanceFromTarget(TailJointAngles));
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


            for (int i = 1; i < _tail.Bones.Length; i++)
            {
                rotation *= Quaternion.AngleAxis(TailJointAngles[i - 1], TailJointAxis[i - 1]);
                Vector3 nextPoint = prevPoint + rotation * _tail.Bones[i].localPosition;//TailJointInitPos[i];
                prevPoint = nextPoint;
            }

            return prevPoint;
        }
        #endregion
    }
}
