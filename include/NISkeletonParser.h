#ifndef NISKELETONPARSER
#define NISKELETONPARSER

#include "NISkeletonType.h"
#include "XmlRpc.h"

//faire un parser inverse aussi car utilisation final va utiliser un pretraitement sur ce qui a etait trouver par openni le resultats brut sera donc stocker sous forme de NISkeletonUserJoints vector

//utilisation faire une fonction pour extrair le nombre de personne detectee
//fair un parser pour un seul user
//et ensuit faire une fonction ou on fourni un pointeur vers un vecteur de  NISkeletonUserJoints de la taille correspondant au user detecte

    void parseToUserDetectedNbr(XmlRpc::XmlRpcValue & dataIN, int & userDetectedNumber ); 

    void parseToNISkeletonUser(XmlRpc::XmlRpcValue & dataIN, NISkeleton::NISkeletonUserJoints & niSkeletonUserJoints);

    void parseToNISkeletonAllUser(XmlRpc::XmlRpcValue & dataIN, NISkeleton::NISkeletonUserJoints* niSkeletonUserJoints);

    void parseToXmlRpcValueUser(NISkeleton::NISkeletonUserJoints & niSkeletonUserJointsIN, int userIN, XmlRpc::XmlRpcValue & dataOUT );

    void parseToXmlRpcValueAllUser(NISkeleton::NISkeletonUserJoints & niSkeletonUserJointsIN, int userDetectedNbrIN, XmlRpc::XmlRpcValue & resultINOUT );


    void parseToUserDetectedNbr(XmlRpc::XmlRpcValue & dataIN, int & userDetectedNumber )
    {

    }

    void parseToNISkeletonUser(XmlRpc::XmlRpcValue & dataIN, NISkeleton::NISkeletonUserJoints &  niSkeletonUserJoints); 
    {


    }

    void parseToNISkeletonAllUser(XmlRpc::XmlRpcValue & dataIN, int userDetectedNbrIN, NISkeleton::NISkeletonUserJoints* niSkeletonUserJoints)
    {


    }


    void parseToXmlRpcValueUser(NISkeleton::NISkeletonUserJoints & niSkeletonUserJointsIN, int userIN, XmlRpc::XmlRpcValue & dataOUT )
    {

    }


    {


    }
//.head.3DPos
//.head.3DPos
//.head.3DPos

    
/*
namespace NISkeleton
{
    struct NISkeleton3DPos
    {
        double x;
        double y;
        double z;
    };

    struct NISkeletonPixelPos
    {   
        double x;
        double y;
    };

    struct NISkeletonJoint
    {
        Skeleton3DPos 3DPos;
        SkeletonPixelPos PixelPos;
        double confidencePos;
    };

    struct NISkeletonUserJoints
    {
        NISkeletonJoint head;
        NISkeletonJoint neck;
        NISkeletonJoint leftShoulder;
        NISkeletonJoint leftElbow;
        NISkeletonJoint leftHand;
        NISkeletonJoint rightShoulder;
        NISkeletonJoint rightElbow;
        NISkeletonJoint rightHand;
        NISkeletonJoint torso;
        NISkeletonJoint leftHip;
        NISkeletonJoint leftKnee;
        NISkeletonJoint leftFoot;
        NISkeletonJoint rightHip;
        NISkeletonJoint rightKnee;
        NISkeletonJoint rightFoot;
        NISkeletonJoint COM;
    };
*/        
//detected user

    void parseToXmlRpcValueAllUser(NISkeleton::NISkeletonUserJoints & niSkeletonUserJointsIN, int userDetectedNbrIN, XmlRpc::XmlRpcValue & resultINOUT )
    {

        resultINOUT["NISkeleton"]["userDetectedNbr"] = userDetectedNbrIN;

//faire un structure contenant une position x,y en pixel
//faire une structure cotenant une position x,y,z en torso

// faire une structure par user contenant une structure pour les joints dans  le real world et dans le pixel world preparer un structure pour les orientation

        for(int iUserDetected =0; iUserDetected < userDetectedNbrIN; userDetectedNbrIN++ )
        {
            resultINOUT["NISkeleton"]["user"][iUserDetected]["Head"]["T"][0] = NISkeletonposInTorsoTranslation_[0].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["Head"]["T"][1] = NISkeletonposInTorsoTranslation_[0].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["Head"]["T"][2] = NISkeletonposInTorsoTranslation_[0].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["Neck"]["T"][0] = NISkeletonposInTorsoTranslation_[1].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["Neck"]["T"][1] = NISkeletonposInTorsoTranslation_[1].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["Neck"]["T"][2] = NISkeletonposInTorsoTranslation_[1].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftShoulder"]["T"][0] = NISkeletonposInTorsoTranslation_[2].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftShoulder"]["T"][1] = NISkeletonposInTorsoTranslation_[2].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftShoulder"]["T"][2] = NISkeletonposInTorsoTranslation_[2].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftElbow"]["T"][0] = NISkeletonposInTorsoTranslation_[3].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftElbow"]["T"][1] = NISkeletonposInTorsoTranslation_[3].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftElbow"]["T"][2] = NISkeletonposInTorsoTranslation_[3].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHand"]["T"][0] = NISkeletonposInTorsoTranslation_[4].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHand"]["T"][1] = NISkeletonposInTorsoTranslation_[4].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHand"]["T"][2] = NISkeletonposInTorsoTranslation_[4].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightShoulder"]["T"][0] = NISkeletonposInTorsoTranslation_[5].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightShoulder"]["T"][1] = NISkeletonposInTorsoTranslation_[5].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightShoulder"]["T"][2] = NISkeletonposInTorsoTranslation_[5].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightElbow"]["T"][0] = NISkeletonposInTorsoTranslation_[6].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightElbow"]["T"][1] = NISkeletonposInTorsoTranslation_[6].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightElbow"]["T"][2] = NISkeletonposInTorsoTranslation_[6].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHand"]["T"][0] = NISkeletonposInTorsoTranslation_[7].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHand"]["T"][1] = NISkeletonposInTorsoTranslation_[7].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHand"]["T"][2] = NISkeletonposInTorsoTranslation_[7].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["Torso"]["T"][0] = NISkeletonposInTorsoTranslation_[8].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["Torso"]["T"][1] = NISkeletonposInTorsoTranslation_[8].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["Torso"]["T"][2] = NISkeletonposInTorsoTranslation_[8].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHip"]["T"][0] = NISkeletonposInTorsoTranslation_[9].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHip"]["T"][1] = NISkeletonposInTorsoTranslation_[9].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHip"]["T"][2] = NISkeletonposInTorsoTranslation_[9].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftKnee"]["T"][0] = NISkeletonposInTorsoTranslation_[10].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftKnee"]["T"][1] = NISkeletonposInTorsoTranslation_[10].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftKnee"]["T"][2] = NISkeletonposInTorsoTranslation_[10].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftFoot"]["T"][0] = NISkeletonposInTorsoTranslation_[11].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftFoot"]["T"][1] = NISkeletonposInTorsoTranslation_[11].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftFoot"]["T"][2] = NISkeletonposInTorsoTranslation_[11].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHip"]["T"][0] = NISkeletonposInTorsoTranslation_[12].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHip"]["T"][1] = NISkeletonposInTorsoTranslation_[12].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHip"]["T"][2] = NISkeletonposInTorsoTranslation_[12].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightKnee"]["T"][0] = NISkeletonposInTorsoTranslation_[13].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightKnee"]["T"][1] = NISkeletonposInTorsoTranslation_[13].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightKnee"]["T"][2] = NISkeletonposInTorsoTranslation_[13].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightFoot"]["T"][0] = NISkeletonposInTorsoTranslation_[14].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightFoot"]["T"][1] = NISkeletonposInTorsoTranslation_[14].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightFoot"]["T"][2] = NISkeletonposInTorsoTranslation_[14].at<float>(2,0);

            resultINOUT["NISkeleton"]["user"][iUserDetected]["COM"]["T"][0] = NISkeletonposInTorsoTranslation_[15].at<float>(0,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["COM"]["T"][1] = NISkeletonposInTorsoTranslation_[15].at<float>(1,0);
            resultINOUT["NISkeleton"]["user"][iUserDetected]["COM"]["T"][2] = NISkeletonposInTorsoTranslation_[15].at<float>(2,0);

//pixell position

            resultINOUT["NISkeleton"]["user"][iUserDetected]["Head"]["CenterX"] = ptJointPos[0].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["Head"]["CenterY"] = ptJointPos[0].Y;

            resultINOUT["NISkeleton"]["user"][iUserDetected]["Neck"]["CenterX"] = ptJointPos[1].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["Neck"]["CenterY"] = ptJointPos[1].Y;

            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftShoulder"]["CenterX"] = ptJointPos[2].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftShoulder"]["CenterY"] = ptJointPos[2].Y;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftElbow"]["CenterX"] = ptJointPos[3].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftElbow"]["CenterY"] = ptJointPos[3].Y;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHand"]["CenterX"] = ptJointPos[4].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHand"]["CenterY"] = ptJointPos[4].Y;

            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightShoulder"]["CenterX"] = ptJointPos[5].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightShoulder"]["CenterY"] = ptJointPos[5].Y;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightElbow"]["CenterX"] = ptJointPos[6].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightElbow"]["CenterY"] = ptJointPos[6].Y;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHand"]["CenterX"] = ptJointPos[7].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHand"]["CenterY"] = ptJointPos[7].Y;

            resultINOUT["NISkeleton"]["user"][iUserDetected]["Torso"]["CenterX"] = ptJointPos[8].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["Torso"]["CenterY"] = ptJointPos[8].Y;

            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHip"]["CenterX"] = ptJointPos[9].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHip"]["CenterY"] = ptJointPos[9].Y;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftKnee"]["CenterX"] = ptJointPos[10].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftKnee"]["CenterY"] = ptJointPos[10].Y;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftFoot"]["CenterX"] = ptJointPos[11].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftFoot"]["CenterY"] = ptJointPos[11].Y;

            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHip"]["CenterX"] = ptJointPos[12].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHip"]["CenterY"] = ptJointPos[12].Y;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightKnee"]["CenterX"] = ptJointPos[13].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightKnee"]["CenterY"] = ptJointPos[13].Y;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightFoot"]["CenterX"] = ptJointPos[14].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["RightFoot"]["CenterY"] = ptJointPos[14].Y;

            resultINOUT["NISkeleton"]["user"][iUserDetected]["COM"]["CenterX"] = ptJointPos[15].X;
            resultINOUT["NISkeleton"]["user"][iUserDetected]["COM"]["CenterY"] = ptJointPos[15].Y;
        }
}

#endif
