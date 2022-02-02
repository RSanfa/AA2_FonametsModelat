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
        Vector3[] TailJointStartOffset;
        float[] TailJointAngles;
        float[] TailJointMinAngle;
        float[] TailJointMaxAngle;
        float[] tailJointPrevAngle;

        //Inverse Kinematics
        float deltaGradient = 0.1f;
        float learningRate = 10f;
        float stopThreshold = 0.01f;
        #endregion

        #region ejercicio 3
        //LEGS
        Transform[] legTargets;
        Transform[] legFutureBases;
        MyTentacleController[] _legs = new MyTentacleController[6];

        //Ejercicio 3
        float threshold = 0.0001f;
        float minAnimTreshold = 1f;
        float animDuration = 0.5f;//esto se tendra que ajustar para que la animacion se vea bien
        //guardamos la posicion de la futureBase en cuanto alcanzamos el umbral
        Vector3[] FixedFutureBase = new Vector3[6];
        //se guarda la posicion inicial del leg para poder resetear su pos enttre la fase forward y backward
        Vector3[] legsInitPos = new Vector3[6];
        //la posicion inicial que se le da en cuanto se hace el lerp entre currentPos y futureBase
        Vector3[] lerpInitPos = new Vector3[6];
        //la posicion final que se le da en cuanto se hace el lerp entre currentPos y futureBase
        Vector3[] lerpFinalPos = new Vector3[6];
        Vector3[] copy = new Vector3[4];
        bool[] legsMoving = new bool[6];
        float[] legsLength = new float[6];
        float[,] bonesLength = new float[6,3];
        float[] legsCurrentDuration = new float[6];
        #endregion

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
                //for (int j = 0; j < _legs[i].Bones.Length - 1; j++)
                //{                    
                //    legsLength[i] += Vector3.Distance(_legs[i].Bones[j].position, _legs[i].Bones[j + 1].position);
                //}

                for(int j = 0; j < _legs[i].Bones.Length - 1; j++)
                {
                    if (j < _legs[i].Bones.Length - 1)
                        bonesLength[i,j] = Vector3.Distance(_legs[i].Bones[j].position, _legs[i].Bones[j + 1].position);
                    else
                        bonesLength[i,j] = 0.0f;
                }
                for(int j = 0; j < bonesLength.GetLength(1); j++)
                {
                    legsLength[i] += bonesLength[i,j];
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

            TailJointStartOffset = new Vector3[6];
            TailJointStartOffset[0] = _tail.Bones[0].localPosition;
            TailJointStartOffset[1] = _tail.Bones[1].localPosition;
            TailJointStartOffset[2] = _tail.Bones[2].localPosition;
            TailJointStartOffset[3] = _tail.Bones[3].localPosition;
            TailJointStartOffset[4] = _tail.Bones[4].localPosition;
            TailJointStartOffset[5] = _tail.Bones[5].localPosition;

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
            TailJointAngles[0] = _tail.Bones[0].localEulerAngles.y;
            TailJointAngles[1] = _tail.Bones[1].localEulerAngles.x;
            TailJointAngles[2] = _tail.Bones[2].localEulerAngles.x;
            TailJointAngles[3] = _tail.Bones[3].localEulerAngles.x;
            TailJointAngles[4] = _tail.Bones[4].localEulerAngles.x;
            TailJointAngles[5] = _tail.Bones[5].localEulerAngles.x;

            tailJointPrevAngle = new float[6];
            tailJointPrevAngle[0] = TailJointAngles[0];
            tailJointPrevAngle[1] = TailJointAngles[1];
            tailJointPrevAngle[2] = TailJointAngles[2];
            tailJointPrevAngle[3] = TailJointAngles[3];
            tailJointPrevAngle[4] = TailJointAngles[4];
            tailJointPrevAngle[5] = TailJointAngles[5];

            ForwardKinematics();
        }

        //TODO: Check when to start the animation towards target and implement Gradient Descent method to move the joints.
        public void NotifyTailTarget(Transform target)
        {
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
            /*
            Vector3 forward = _tail.Bones[5].TransformDirection(Vector3.forward) * 2;
            Debug.DrawRay(_tail.Bones[5].position, forward, Color.cyan);
            forward = _tail.Bones[4].TransformDirection(Vector3.forward) * 2;
            Debug.DrawRay(_tail.Bones[4].position, forward, Color.black);
            forward = _tail.Bones[3].TransformDirection(Vector3.forward) * 2;
            Debug.DrawRay(_tail.Bones[3].position, forward, Color.gray);
            forward = _tail.Bones[2].TransformDirection(Vector3.forward) * 2;
            Debug.DrawRay(_tail.Bones[2].position, forward, Color.magenta);
            forward = _tail.Bones[1].TransformDirection(Vector3.forward) * 2;
            Debug.DrawRay(_tail.Bones[1].position, forward, Color.white);
            forward = _tail.Bones[0].TransformDirection(Vector3.forward) * 2;
            Debug.DrawRay(_tail.Bones[0].position, forward, Color.yellow);
            */

            //por ahora solo queremos mover la cola: ejercicio 3
            if (closeEnough)
            {
                updateTail(tailTarget.position);
                //ForwardKinematics();
            }
            if (walkAnimation)
            {
                updateLegPos();
                //todo lo relacionado con el lerp

                #region pornoborrarlo
                /*
                
                for (int i = 0; i < _legs.Length; i++)
                {
                    //miramos si la pata ha alcanzado el umbral 
                    if (minAnimTreshold < Vector3.Distance(_legs[i].Bones[0].position, legFutureBases[i].position) && !legsMoving[i])
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
                        _legs[i].Bones[0].position = Vector3.Lerp(lerpInitPos[i], FixedFutureBase[i], legsCurrentDuration[i] / animDuration);

                        //despues del lerp hay que mirar si la animacion ha terminado o no (mirando la distancia de la posicion actual y la de 
                        //la future base guardada
                        //en el caso que haya acabado se tiene que cambiar el valor de legsMoving[i]

                        legsCurrentDuration[i] += Time.fixedDeltaTime;
                        if (legsCurrentDuration[i] >= animDuration)
                        {
                            legsCurrentDuration[i] = 0;
                            legsMoving[i] = false;
                        }
                    }
                }

                */
                #endregion

            }
        }
        #endregion


        #region private
        #region FABRIK
        //TODO: implement fabrik method to move legs 
        private void updateLegPos()
        {
            //check for the distance to the futureBase, then if it's too far away start moving the leg towards the future base position
            //
            for (int i = 0; i < _legs.Length; i++)
            {
                for(int j = 0; j < _legs[i].Bones.Length; j++)
                {
                    copy[j] = _legs[i].Bones[j].position;
                }
                //se calcula la distancia entre el origen y el target
                float dist = Vector3.Distance(_legs[i].Bones[0].position, legTargets[i].position);
                //se mira si la distancia es mas grande que la longitud del le

                if (dist > legsLength[i])
                {
                    //el target no se puede alcanzar
                    for (int j = 0; j < _legs[i].Bones.Length - 1; j++)
                    {
                        //se busca la distancia entre el joint j y el target i
                        //float r = Vector3.Distance(_legs[i].Bones[j].position, legTargets[i].position);
                        float r = Vector3.Distance(copy[j], legTargets[i].position);
                        float l = bonesLength[i,j] / r;//legsLength[j] / r;

                        //se calcula la nueva posicion del joint
                        //_legs[i].Bones[j + 1].position = ((1 - l) * _legs[i].Bones[j].position) + (l * legTargets[i].position);
                        copy[j + 1] = ((1 - l) * copy[j]) + (l * legTargets[i].position);
                    }
                }
                else
                {
                    //se puede alcanzar el target
                    legsInitPos[i] = copy[0];//_legs[i].Bones[0].position;
                    updateLegs(i);
                }

                for(int j = 0; j < _legs[i].Bones.Length; j++)
                {
                    _legs[i].Bones[j].position = copy[j];
                }

                FABRIKLerp(i);
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
                //dist = Vector3.Distance(_legs[i].Bones[_legs[i].Bones.Length - 1].position, legTargets[i].position);
                dist = Vector3.Distance(copy[copy.Length - 1], legTargets[i].position);
                count--;
            }
            //Debug.Log("Ha salido del while");
        }

        private void backward(int i)
        {
            //_legs[i].Bones[0].position = legsInitPos[i];
            copy[0] = legsInitPos[i];

            for (int j = 1; j < /*_legs[i].Bones.Length*/ copy.Length - 2; j++)
            {
                /*
                 
                //se calcula la distancia entre el nuevo joint j y el joint j+1
                float r = Vector3.Distance(_legs[i].Bones[j].position, _legs[i].Bones[j + 1].position);
                float l = bonesLength[i,j] / r;//legsLength[i] / r;

                //se calcula la nueva posicion del joint
                _legs[i].Bones[j + 1].position = (1 - l) * _legs[i].Bones[j].position + l * _legs[i].Bones[j + 1].position;

                */
                Vector3 currToNext = copy[j + 1] - copy[j];//_legs[i].Bones[j + 1].position - _legs[i].Bones[j].position;
                currToNext.Normalize();
                //_legs[i].Bones[j].position = _legs[i].Bones[j - 1].position + currToNext * bonesLength[i, j];
                copy[j] = copy[j - 1] + currToNext * bonesLength[i, j];
            }
        }

        private void forward(int i)
        {
            //_legs[i].Bones[_legs[i].Bones.Length - 1].position = legTargets[i].position;
            copy[copy.Length - 1] = legTargets[i].position;
            //Debug.Log("target[" + i + "]: " + legTargets[i].localPosition);

            for (int j = /*_legs[i].Bones.Length - 2*/copy.Length-2; j >= 0; j--)
            {
                /*
                 
                //buscamos la distancia entre el nuevo joint j+1 y el joint j
                float r = Vector3.Distance(_legs[i].Bones[j + 1].position, _legs[i].Bones[j].position);
                float l = bonesLength[i,j] / r;// legsLength[i] / r;
                Debug.Log("[" + i + "," + j + "]:" + bonesLength[i, j]);
                //se calcula la nueva posicion 
                _legs[i].Bones[j].position = (1 - l) * _legs[i].Bones[j + 1].position + l * _legs[i].Bones[j].position;

                */

                Vector3 currToNex = copy[j] - copy[j + 1];//_legs[i].Bones[j].position - _legs[i].Bones[j + 1].position;
                currToNex.Normalize();
                //_legs[i].Bones[j].position = _legs[i].Bones[j + 1].position + currToNex * bonesLength[i, j];
                copy[j] = copy[j + 1] + currToNex * bonesLength[i, j];
            }
        }

        private void FABRIKLerp(int i)
        {
            //miramos si la pata ha alcanzado el umbral 
            if (minAnimTreshold < Vector3.Distance(_legs[i].Bones[0].position, legFutureBases[i].position) && !legsMoving[i])
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
                _legs[i].Bones[0].position = Vector3.Lerp(lerpInitPos[i], FixedFutureBase[i], legsCurrentDuration[i] / animDuration);

                //despues del lerp hay que mirar si la animacion ha terminado o no (mirando la distancia de la posicion actual y la de 
                //la future base guardada
                //en el caso que haya acabado se tiene que cambiar el valor de legsMoving[i]

                legsCurrentDuration[i] += Time.fixedDeltaTime;
                if (legsCurrentDuration[i] >= animDuration)
                {
                    legsCurrentDuration[i] = 0;
                    legsMoving[i] = false;
                }
            }
        }
        #endregion
        #region Gradient descent
        //TODO: implement Gradient Descent method to move tail if necessary
        void updateTail(Vector3 target)
        {
            for(int i = _tail.Bones.Length-2; i >=0; i--)
            {
                //calculamos el gradiente
                float angle = TailJointAngles[i];

                float f_x = DistanceFromTarget(target, true);

                TailJointAngles[i] += deltaGradient;

                float f_x_plus_d = DistanceFromTarget(target, false);

                float gradient = (f_x_plus_d - f_x) / deltaGradient;

                TailJointAngles[i] = angle;

                TailJointAngles[i] -= learningRate * gradient;

                float angleOffset = TailJointAngles[i] - tailJointPrevAngle[i];
                tailJointPrevAngle[i] = TailJointAngles[i];

               // Vector3 rotation = TailJointAxis[i] * angleOffset;

                //_tail.Bones[i].Rotate(rotation, Space.Self);

                _tail.Bones[i].localEulerAngles = TailJointAxis[i] * TailJointAngles[i];
            }
        }

        float DistanceFromTarget(Vector3 target, bool norm)
        {
            Vector3 point = ForwardKinematics();
            if (norm)
            {
               Debug.Log("end effector global pos: " + point);
               Debug.Log("posicion real del end effector" + _tail.Bones[5].position);

                for(int i = 0; i < _tail.Bones.Length; i++)
                {
                   Debug.Log(_tail.Bones[i].name + " local position: " + _tail.Bones[i].localPosition);
                }
                //Debug.Break();
            }
            return Vector3.Distance(point, target);
        }

        Vector3 ForwardKinematics()
        {
            //global position
            Vector3 prevPoint = _tail.Bones[0].position;
            // Takes object initial rotation into account
            Quaternion rotation = Quaternion.identity;//transform.rotation;
            //TODO
            for (int i = 1; i < _tail.Bones.Length; i++)
            {                                   
                rotation *= Quaternion.AngleAxis(TailJointAngles[i - 1], TailJointAxis[i - 1]);

                Vector3 nextPoint = prevPoint + rotation * TailJointStartOffset[i];
                prevPoint = nextPoint;
            }
            return prevPoint;
        }
        #endregion
        #endregion
    }
}