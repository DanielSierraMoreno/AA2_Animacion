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

        public Transform[] Bones { get => _bones; }

        //Exercise 1.
        public Transform[] LoadTentacleJoints(Transform root, TentacleMode mode)
        {
            //TODO: add here whatever is needed to find the bones forming the tentacle for all modes
            //you may want to use a list, and then convert it to an array and save it into _bones
            List<Transform> joints = new List<Transform>();
            tentacleMode = mode;
            Transform auxiliar = root;
            switch (tentacleMode)
            {
                case TentacleMode.LEG:
                    //TODO: in _endEffectorsphere you keep a reference to the base of the leg
                    auxiliar = root.GetChild(0);
                    joints.Add(auxiliar);
                    while (auxiliar.name != "Joint2")
                    {
                        auxiliar = auxiliar.GetChild(1);
                        joints.Add(auxiliar);
                    }
                    // this add is to obtain endEfector
                    auxiliar = auxiliar.GetChild(1);
                    joints.Add(auxiliar);
                    break;
                case TentacleMode.TAIL:
                    //TODO: in _endEffectorsphere you keep a reference to the red sphere 
                    // As we recive joint 0 we can use the auxiliar and add it to joints directly
                    joints.Add(auxiliar);
                    while (auxiliar.name != "EndEffector")
                    {
                        auxiliar = auxiliar.GetChild(1);
                        joints.Add(auxiliar);
                    }
                    break;
                case TentacleMode.TENTACLE:
                    //TODO: in _endEffectorphere you  keep a reference to the sphere with a collider attached to the endEffector
                    // To avoid armaments and bones we search 2 time in getchild
                    auxiliar = auxiliar.GetChild(0);
                    auxiliar = auxiliar.GetChild(0);

                    while (auxiliar.name != "Bone.001_end")
                    {
                        auxiliar = auxiliar.GetChild(0);
                        joints.Add(auxiliar);
                    }
                    break;
            }
            _bones = joints.ToArray();
            return Bones;
        }
    }
}
