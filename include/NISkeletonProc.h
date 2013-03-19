#ifndef NISKELETONPROC
#define NISKELETONPROC

#include <vision/vision.h>

#include <opencv2/opencv.hpp>

#include <boost/thread.hpp>

#include <XnOS.h>
//#include <XnCppWrapper.h>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#include "XmlRpc.h"

//CoshellClient is used to obtain the head position
#include <coshell-client/CoshellClient.h>

using namespace xn;

namespace NISkeleton
{
    class NISkeletonProc
    {
        public:
    
            NISkeletonProc();
            ~NISkeletonProc();
    
    //used for NISkeleton
            void SetContext( Context* contextIN ){ context_ = contextIN; };
            void SetDepthGenerator( DepthGenerator* depthIN){ depth_ = depthIN; };
            void SetDepthMetaData( DepthMetaData* depthMDIN){ depthMD_ = depthMDIN; };
    
            Context* GetContext(){ return context_; };
            DepthGenerator* GetDepthGenerator(){ return depth_; };
            DepthMetaData* GetDepthMetaData(){ return depthMD_; };
            UserGenerator* GetUserGenerator(){ return &userSkel_; };
    
            void initNISkeleton();
            static void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
            static void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
    
            void SetSandbox(std::string sandbox ){ sandbox_ = sandbox; };
            void initData();
            
            void GetNISkeletonResult(XmlRpc::XmlRpcValue & resultINOUT );

//used to detect the face in a ROI
            boost::mutex* GetNISkeletonMatMonoMutexPtr(){ return NISkeletonMatMonoMutex_; };

            void SetNISkeletonMatMonoMutexPtr( boost::mutex* NISkeletonMatMonoMutex){ NISkeletonMatMonoMutex_ = NISkeletonMatMonoMutex; };
            void SetNISkeletonMatMonoPtr( cv::Mat* NISkeletonMatMono ){ NISkeletonMatMonoPtr_ = NISkeletonMatMono; };

            bool checkFace();
                
        private:
    
            std::string sandbox_;
            Context* context_;
            DepthGenerator* depth_;
            DepthMetaData* depthMD_;
            UserGenerator userSkel_;
            XnCallbackHandle hUserCallbacks;
    
            cv::Mat cameraMatrixLeft_;
            cv::Mat distorsionMatrixLeft_;
            cv::Mat headToCamMatrixLeft_;
    
            cv::Mat cameraMatrixRight_;
            cv::Mat distorsionMatrixRight_;
            cv::Mat headToCamMatrixRight_;
    
            cv::Mat cameraMatrixXtionRGB_;
            cv::Mat distorsionMatrixXtionRGB_;
            cv::Mat headToCamMatrixXtionRGB_;
    
    // used in NIskeleton
            std::vector<cv::Mat> NISkeletonPosInCam_;
            std::vector<cv::Mat> NISkeletonPosInCamTranslation_;
            std::vector<cv::Mat> NISkeletonPosInCamRotation_;
            std::vector<cv::Mat> NISkeletonPosInCamEuler_;
            std::vector<cv::Mat> NISkeletonposInTorsoTranslation_;
     
            cv::Mat NISkeletonRefToHeadTemp_;
            cv::Mat NISkeletonPosInCamTranslationTemp_;
     
            int NISkeletonJointNbrMax_;

            XmlRpc::XmlRpcValue NISkeletonResult_;

            boost::mutex* NISkeletonResultMutexPtr_;
    
    //end used in NISkeleton
            std::string coshellName_;
            coshell::CoshellClient * coshellNISkeleton_;

    //used to detect the face in a ROI
            boost::mutex* NISkeletonMatMonoMutex_;

            cv::Mat * NISkeletonMatMonoPtr_;

            double NISkeletonScaleFactorFace_;
            int NISkeletonMinNeighborsFace_;
            int NISkeletonFlagsFace_;
            cv::Size NISkeletonMinSizeFace_;
            cv::Size NISkeletonMaxSizeFace_;
            std::string NISkeletonFacesCascadeName_;
            int NISkeletonROIWidth_;
            int NISkeletonROIHeight_;

            std::vector<cv::Rect> NISkeletonFaces_;

            cv::CascadeClassifier NISkeletonFacesCascade_;

            cv::RNG rng;
    
    };
}   //namespace NISkeleton
#endif
