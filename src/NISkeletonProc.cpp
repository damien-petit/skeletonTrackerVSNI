#include "NISkeletonProc.h"

namespace NISkeleton
{
    NISkeletonProc * me_ = 0;

    NISkeletonProc::NISkeletonProc()
    {
        me_ = this;
    }
    
    NISkeletonProc::~NISkeletonProc()
    {
    }
    
    void NISkeletonProc::initNISkeleton()
    {
        XnStatus nRetVal = XN_STATUS_OK;
    
        userSkel_.Create(*context_);
    
    //dans maim
        if (!userSkel_.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
        {
            printf("Supplied user generator doesn't support skeleton\n");
            //return 1;
        }
        else
        {
            printf("Supplied user generator support skeleton\n");
        }
    
        nRetVal = userSkel_.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
    //    CHECK_RC(nRetVal, "Register to user callbacks");
    
        userSkel_.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);
        printf("initSkeleton\n");
    
    //need this to ensure that the userGenerator generate data since he is not created at the  same time that the note in the vs_core openni controller
    //    context_->StartGeneratingAll();i
    
        userSkel_.StartGenerating();
    
        XnBool checkGenerate = userSkel_.IsGenerating();
        if(!checkGenerate)
        {
            std::cout<<"userGenerator failed to start"<<std::endl;     
        }
        else
        {
            std::cout<<"userGenerator succeed to start"<<std::endl;
        }
    }
    
    // Callback: New user was detected
    void NISkeletonProc::User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
    {
        XnUInt32 epochTime = 0;
        xnOSGetEpochTime(&epochTime);
        printf("%d New User %d\n", epochTime, nId);
        // New user found
        // Load user's calibration from file
        std::string nameUserCalibrationFile("UserCalibration.bin");
        std::stringstream userCalibrationFilenPath;
        userCalibrationFilenPath << me_->sandbox_;
        userCalibrationFilenPath << nameUserCalibrationFile;
    
        XnStatus rc = me_->userSkel_.GetSkeletonCap().LoadCalibrationDataFromFile(nId, userCalibrationFilenPath.str().c_str());
        if (rc == XN_STATUS_OK)
        {
            // Make sure state is coherent
            me_->userSkel_.GetPoseDetectionCap().StopPoseDetection(nId);
            me_->userSkel_.GetSkeletonCap().StartTracking(nId);
    
            if (me_->userSkel_.GetSkeletonCap().IsTracking(nId))
            {
                printf(" tracked!\n");
                //    return;
            }
            else
            {
                printf("not tracked!\n");                
            }
        }
        else
        {
            std::cout<<"LoadCalibrationDataFromFile fail check the configuration in feature_1_5_2.ini or launch the init script"<<std::endl;
            std::cout<<"other possible source of error:"<<std::endl;
            std::cout<<"LoadCalibrationDataFromFile check if there is a calibration file at :"<<std::endl;
            std::cout<<" "<<me_->sandbox_<<"UserCalibration.bin"<<std::endl;
        }
    }
    
    // Callback: An existing user was lost
    void NISkeletonProc::User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
    {
        XnUInt32 epochTime = 0;
        xnOSGetEpochTime(&epochTime);
        printf("%d Lost user %d\n", epochTime, nId);
    }
    
    void NISkeletonProc::initData()
    {
        cv::FileStorage dataToLoadOpenCV;
        
        if(!dataToLoadOpenCV.open(sandbox_ +"dataToLoadOpenCVFile.yml", cv::FileStorage::READ))
        {
            std::cout<<"ERROR during the opening of the file dataCalibCameraLeftOpenCV.yml"<<std::endl;
        }

        //initialisation of the different camera matrix calibration

        dataToLoadOpenCV["cameraMatrixLeft"] >> cameraMatrixLeft_;
        dataToLoadOpenCV["distorsionMatrixLeft"] >> distorsionMatrixLeft_;
        dataToLoadOpenCV["headToCamMatrixLeft"] >> headToCamMatrixLeft_;

        dataToLoadOpenCV["cameraMatrixRight"] >> cameraMatrixRight_;
        dataToLoadOpenCV["distorsionMatrixRight"] >> distorsionMatrixRight_;
        dataToLoadOpenCV["headToCamMatrixRight"] >> headToCamMatrixRight_;

        dataToLoadOpenCV["cameraMatrixXtionRGB"] >> cameraMatrixXtionRGB_;
        dataToLoadOpenCV["distorsionMatrixXtionRGB"] >> distorsionMatrixXtionRGB_;
        dataToLoadOpenCV["headToCamMatrixXtionRGB"] >> headToCamMatrixXtionRGB_;

        dataToLoadOpenCV["NISkeletonJointNbrMax_"] >> NISkeletonJointNbrMax_;

        dataToLoadOpenCV["coshellName_"] >> coshellName_;
        std::cout<<"coshellName_ "<<coshellName_<<std::endl;


//used for detect faces in NISkeleton
        dataToLoadOpenCV["NISkeletonFaceCascadeName_"] >> NISkeletonFacesCascadeName_;

        //load different parameters for body parts detections
        dataToLoadOpenCV["NISkeletonScaleFactorFace_"] >> NISkeletonScaleFactorFace_;
        dataToLoadOpenCV["NISkeletonMinNeighborsFace_"] >> NISkeletonMinNeighborsFace_;
        dataToLoadOpenCV["NISkeletonFlagsFace_"] >> NISkeletonFlagsFace_;
        dataToLoadOpenCV["NISkeletonFaceCascadeName_"] >> NISkeletonFacesCascadeName_;
        int NISkeletonMinSizeXFace_;
        int NISkeletonMaxSizeXFace_;
        int NISkeletonMinSizeYFace_;
        int NISkeletonMaxSizeYFace_;

        dataToLoadOpenCV["NISkeletonMinSizeXFace_"] >> NISkeletonMinSizeXFace_;
        dataToLoadOpenCV["NISkeletonMaxSizeXFace_"] >> NISkeletonMaxSizeXFace_;
        dataToLoadOpenCV["NISkeletonMinSizeYFace_"] >> NISkeletonMinSizeYFace_;
        dataToLoadOpenCV["NISkeletonMaxSizeYFace_"] >> NISkeletonMaxSizeYFace_;

        NISkeletonMinSizeFace_ = cv::Size(NISkeletonMinSizeXFace_,NISkeletonMinSizeYFace_);
        NISkeletonMaxSizeFace_ = cv::Size(NISkeletonMaxSizeXFace_,NISkeletonMaxSizeYFace_);

        dataToLoadOpenCV["NISkeletonROIWidth_"] >> NISkeletonROIWidth_;
        dataToLoadOpenCV["NISkeletonROIHeight_"] >> NISkeletonROIHeight_;

//end used for detect faces in NISkeleton
        dataToLoadOpenCV.release();

        NISkeletonRefToHeadTemp_ = cv::Mat::zeros(4,4,CV_32F);
        NISkeletonPosInCamTranslationTemp_ = cv::Mat::zeros(4,1,CV_32F);

//the +1 is for the com of the user detected
        NISkeletonPosInCamRotation_.resize(NISkeletonJointNbrMax_+1);
        NISkeletonPosInCamEuler_.resize(NISkeletonJointNbrMax_+1);
        NISkeletonPosInCam_.resize(NISkeletonJointNbrMax_+1);
        NISkeletonPosInCamTranslation_.resize(NISkeletonJointNbrMax_+1);
        NISkeletonposInTorsoTranslation_.resize(NISkeletonJointNbrMax_+1);

        for(int i = 0; i < NISkeletonJointNbrMax_+1; i++)
        {
            NISkeletonPosInCamRotation_[i] = cv::Mat::zeros(3, 3, CV_32F);
            NISkeletonPosInCamEuler_[i] = cv::Mat::zeros(3, 1,CV_32F);
            NISkeletonPosInCam_[i] = cv::Mat::zeros(4, 4, CV_32F);
            NISkeletonPosInCamTranslation_[i] = cv::Mat::zeros(4, 1, CV_32F);
            NISkeletonposInTorsoTranslation_[i] = cv::Mat::zeros(4, 1, CV_32F);
        }

        coshellNISkeleton_ = new coshell::CoshellClient(coshellName_, 2809, false);
        //We have to  initialize the coshell since we dont use coshell interpreter
        coshellNISkeleton_->Initialize();

        //in case another plugin is already send this command
        if( coshellNISkeleton_->ExecuteACommand("dyn2.createOpPoint head 16").size() == 0 )
        {
            coshellNISkeleton_->ExecuteACommand("OpenHRP.periodicCall addSignal dyn2.head");
        }
//        NISkeletonResult_ = new XmlRpc::XmlRpcValue;
    }

    bool NISkeletonProc::checkFace()
    {
//NISkeletonImageRGBPtr_

        {
            boost::mutex::scoped_lock lock(*NISkeletonMatMonoMutex_);

//histogram is already equalize
//       cv::equalizeHist( NISkeletonMatMono_, NISkeletonMatMono_ );

//-- Detect faces
//        facesCascade_.detectMultiScale( imageGrayIN, faces_, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
        
            NISkeletonFacesCascade_.detectMultiScale( *NISkeletonMatMonoPtr_, NISkeletonFaces_, NISkeletonScaleFactorFace_, NISkeletonMinNeighborsFace_, NISkeletonFlagsFace_, NISkeletonMinSizeFace_ , NISkeletonMaxSizeFace_ );
        }
        
        int facesNbr = NISkeletonFaces_.size();

        if(facesNbr >= 1)
        {
            return 1;   
        }
        else
        {
            return 0;
        }
    }

    void NISkeletonProc::GetNISkeletonResult( XmlRpc::XmlRpcValue & resultINOUT)
    {

        XnSkeletonJointPosition jointPos[15];

//a XnSkeletonJointOrientation has this public attribute:
// XnMatrix3X3    orientation
// XnConfidence    fConfidence

//        XnSkeletonJointOrientation JointOr[15];
//16 position : 15 joints position and 1 COM
/*
00444 typedef struct XnVector3D
00445 {
00446     XnFloat X;
00447     XnFloat Y;
00448     XnFloat Z;
00449 } XnVector3D;
00450 
00451 typedef XnVector3D XnPoint3D;
*/

        XnPoint3D ptJointPos[16];

        XnUserID aUsers[15];
        XnUInt16 nUsers = 15;
        userSkel_.GetUsers(aUsers, nUsers);

//different XnSkeletonJoint
/*
        XN_SKEL_HEAD 
        XN_SKEL_NECK

        XN_SKEL_LEFT_SHOULDER
        XN_SKEL_LEFT_ELBOW
        XN_SKEL_LEFT_HAND

        XN_SKEL_RIGHT_SHOULDER
        XN_SKEL_RIGHT_ELBOW
        XN_SKEL_RIGHT_HAND

        XN_SKEL_TORSO
   
        XN_SKEL_LEFT_HIP
        XN_SKEL_LEFT_KNEE
        XN_SKEL_LEFT_FOOT

  
        XN_SKEL_RIGHT_HIP
        XN_SKEL_RIGHT_KNEE
        XN_SKEL_RIGHT_FOOT
*/

/*
    XnPoint3D pt[2];
    pt[0] = joint1.position;
    pt[1] = joint2.position;

*/
        resultINOUT["NISkeleton"]["userDetectedNbr"] = nUsers;
//        std::cout<<"nUsers "<<nUsers<<std::endl;
//       int NISkeletonUserDetectedNbrTMP = (int)resultINOUT["NISkeleton"]["userDetectedNbr"];

//        std::cout<<"NISkeletonUserDetectedNbrTMP"<<NISkeletonUserDetectedNbrTMP<<std::endl;

        for (int iUserDetected = 0; iUserDetected < nUsers; ++iUserDetected)
        {
            if (!userSkel_.GetSkeletonCap().IsTracking(aUsers[iUserDetected]))
            {
//                printf("not tracked!\n");
            //    return;
            }

//get the translation of the joints

            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_HEAD, jointPos[0]);
            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_NECK, jointPos[1]);

            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_SHOULDER, jointPos[2]);
            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_ELBOW, jointPos[3]);
            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_HAND, jointPos[4]);


userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_SHOULDER, jointPos[5]);
            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_ELBOW, jointPos[6]);
            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_HAND, jointPos[7]);

            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_TORSO, jointPos[8]);

            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_HIP, jointPos[9]);
            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_KNEE, jointPos[10]);
            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_FOOT, jointPos[11]);

            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_HIP, jointPos[12]);
            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_KNEE, jointPos[13]);
            userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_FOOT, jointPos[14]);

//get  the orientation from the joints
//          XnStatus    GetSkeletonJointOrientation (XnUserID user, XnSkeletonJoint eJoint, XnSkeletonJointOrientation &Joint) const 
/*
            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_HEAD, JointOr[0]);
            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_NECK, JointOr[1]);

            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_SHOULDER, JointOr[2]);
            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_ELBOW, JointOr[3]);
            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_HAND, JointOr[4]);

            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_SHOULDER, JointOr[5]);
            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_ELBOW, JointOr[6]);
            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_HAND, JointOr[7]);

            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_TORSO, JointOr[8]);

            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_HIP, JointOr[9]);
            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_KNEE, JointOr[10]);
            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_FOOT, JointOr[11]);

            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_HIP, JointOr[12]);
            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_KNEE, JointOr[13]);
            userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_FOOT, JointOr[14]);
*/

//check the confidence of the measure for translation
            for(int iBodyPart = 0; iBodyPart < 15; iBodyPart++)
            {
                if (jointPos[iBodyPart].fConfidence < 0.5)
                {
                //   return;
                }
//ptJointPos is usefull container because we can use the openNI function for the projection after            
                ptJointPos[iBodyPart] = jointPos[iBodyPart].position;


                NISkeletonPosInCamTranslation_[iBodyPart].at<float>(0,0) =  ptJointPos[iBodyPart].X;
                NISkeletonPosInCamTranslation_[iBodyPart].at<float>(1,0) =  ptJointPos[iBodyPart].Y;
                NISkeletonPosInCamTranslation_[iBodyPart].at<float>(2,0) =  ptJointPos[iBodyPart].Z;
                NISkeletonPosInCamTranslation_[iBodyPart].at<float>(3,0) = 1;
//a examiner le calcul effectuei
//regader la doc file:///home/petit/Downloads/OpenNI-Bin-ev-Linux-x86-v1.5.4.0/Documentation/html/conc_coord.html
//converti les donnees en pixel pour x et y z est tou jours en millimetre les origin en x et y son change du centre de fov a upper-left corner of the FOV
            }

            userSkel_.GetCoM(aUsers[iUserDetected], ptJointPos[15]);

//compute the joint detected in the waist position

            NISkeletonRefToHeadTemp_.at<float>(3,0) = 0;
            NISkeletonRefToHeadTemp_.at<float>(3,1) = 0;
            NISkeletonRefToHeadTemp_.at<float>(3,2) = 0;
            NISkeletonRefToHeadTemp_.at<float>(3,3) = 1;

            {
                 std::stringstream ss;
                 ss << coshellNISkeleton_->ExecuteACommand("dyn2.head");
                 //std::cout<<"VISION"<<std::endl;
                 int i = 0;
                 int j = 0;
                 double value;
                 char tmp;
                 while(ss >> tmp)
                 {
                     if( tmp == ';' )
                     {
                         ++i;
                         j = 0;
                     }
                     if( tmp == '[' || ( tmp == ',' && j != 4) || tmp == ';' )
                     {
                         ss >> value;
                         NISkeletonRefToHeadTemp_.at<float>(i,j) = value;
                         ++j;
                     }
                     if( tmp == ']' )
                     {
                         break;
                     }
                 }
             }


             for(int iBodyPart = 0; iBodyPart < 15; iBodyPart++ )
             {
                 NISkeletonposInTorsoTranslation_[iBodyPart] = (NISkeletonRefToHeadTemp_*headToCamMatrixLeft_)*NISkeletonPosInCamTranslation_[iBodyPart];
             }
//                ARCodeposInTorsoTranslation_[iSize] = (ARCodeRefToHeadTemp_*headToCamMatrixLeft_)*ARCodePosInCamTranslationTemp_;

//RealWorld Position in torsoreference

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

            depth_->ConvertRealWorldToProjective(16, ptJointPos, ptJointPos);

//Pixel Position

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

//TODO hard copy of the data if we want to speed up the process we van write directly in *resultINOUT because the pointer has been allocated  in bci-self-interact
            //resultINOUT = resultINOUT;

        }
    }

}   //namespace NISkeleton
