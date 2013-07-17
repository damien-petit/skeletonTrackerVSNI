#ifndef SKELETON_TRACKER_VSNI
#define SKELETON_TRACKER_VSNI

#include <deque>

#include <configparser/configparser.h>
#include <vision/vision.h>
#include <visionsystem/plugin.h>
#include <visionsystem/viewer.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include "XmlRpc.h"

//CoshellClient is used to obtain the head position
#include <coshell-client/CoshellClient.h>

//openni

#include <XnOS.h>
//#include <XnCppWrapper.h>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#include "NISkeletonType.h"

#include <opencv2/opencv.hpp>

namespace skeletonTrackerVSNI
{

    class SkeletonTrackerVSNI : public visionsystem::Plugin, public visionsystem::WithViewer, public configparser::WithConfigFile, public XmlRpc::XmlRpcServerMethod {

        public:

            SkeletonTrackerVSNI( visionsystem::VisionSystem *vs, string sandbox ) ;
            ~SkeletonTrackerVSNI() ;

            bool pre_fct()  ;
            void preloop_fct()  ;
            void loop_fct() ;
            bool post_fct() ;
            void calculs();
            /*virtual function of configparser used by  read_config_file of configparser::WithConfigFile*/
            void parse_config_line( vector<string> &line );
            void callback(visionsystem::Camera* cam, XEvent event);
            void glfunc(visionsystem::Camera* cam);

            /* Allow XML-RPC remote call */
            /*
                params should be a string with object name
                result will be this way: { "left" : { x, y }, "right" : { x, y } }
            */
            void execute(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result);

            bool GetObjectPositionNISkeleton(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result);
            bool GetObjectPositionNISkeleton2(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result);

            void initData();

            void initUserGen();
            void startUserGen();
            void stopUserGen();
            
            void initHandsGen();
            void startHandsGen();
            void stopHandsGen();

            void faceDetection();


//OpenNI callback
            static void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
            static void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);

            static void XN_CALLBACK_TYPE Hand_Create(xn::HandsGenerator& generator, XnUserID nId, const XnPoint3D* pPosition, XnFloat fTime, void* pCookie);
            static void XN_CALLBACK_TYPE Hand_Update(xn::HandsGenerator& generator, XnUserID nId, const XnPoint3D* pPosition, XnFloat fTime, void* pCookie);
            static void XN_CALLBACK_TYPE Hand_Destroy(xn::HandsGenerator& generator, XnUserID nId, XnFloat fTime, void* pCookie);

            bool checkDeque(int xFacePos, int yFacePos);

            bool updateSkeletonData();
        private:

            std::string sandbox_;

            vision::Image<uint32_t, vision::RGB> * imgXi_;
            vision::Image<uint8_t, vision::MONO> * imgXiMONO_;
            vision::Image<uint32_t, vision::RGB> * imgDispXi_;
            vision::Image<uint16_t, vision::DEPTH> * imgXd_;
            vision::Image<uint16_t, vision::DEPTH> * imgDispXd_;

            bool finish_;

            visionsystem::Camera * camXi_;
            visionsystem::Camera * camXd_;

//TODO thread used for calcul it may be unnecessary
            boost::thread t_;

            std::string camNameXi_;
            std::string camNameXd_;

            cv::Mat cameraMatrixXtionRGB_;
            cv::Mat distorsionMatrixXtionRGB_;
            cv::Mat headToCamMatrixXtionRGB_;

// used by the openni skeleton tracker function
            xn::Context* context_;
            xn::DepthGenerator* depth_;
            xn::ImageGenerator* image_;
            xn::DepthMetaData* depthMD_;
            xn::UserGenerator userGen_;
            XnCallbackHandle hUserCallbacks_;
            bool userGenStarted_;
            bool trackChecked_;

            XnCallbackHandle hHandCallbacks_;
            xn::HandsGenerator handsGen_;

// end used by the openni skeleton tracker function

//used to check if the user detected is real
            
            boost::mutex NISkeletonUserDetectedVecMutex_;
            std::vector<NISkeleton::NISkeletonUserDetected> userDetectedVec_;
            int maxUserDetected_;

            std::string coshellName_;
            coshell::CoshellClient * coshellVision_;

//used for face detection
            cv::Mat NISkeletonMatMono_;

            double NISkeletonScaleFactorFace_;
            int NISkeletonMinNeighborsFace_;
            int NISkeletonFlagsFace_;
            cv::Size NISkeletonMinSizeFace_;
            cv::Size NISkeletonMaxSizeFace_;
            std::string NISkeletonFacesCascadeName_;
            int NISkeletonROIWidth_;
            int NISkeletonROIHeight_;

            std::vector<cv::Rect> NISkeletonFaces_;

            boost::mutex NISkeletonFacesResultMutex_;
            std::vector<cv::Rect> NISkeletonFacesResult_;

            int maxSizeDeque_;
            std::deque< std::vector<cv::Rect> > NISkeletonFacesResultDeque_;
            boost::mutex NISkeletonFacesResultDequeMutex_;
            bool fullDeque_;

            cv::CascadeClassifier NISkeletonFacesCascade_;

            cv::RNG rng;

//end used in face detection

//used in COM detection
            struct COMResult
            {
                int nId;
                int x;
                int y;    
            };

            std::vector< COMResult > NISkeletonCOMResult_;
            boost::mutex NISkeletonCOMResultMutex_;
//end used in COM detection

//used in updateSkeletonData
// 15 joints + 1 COM
//float XnPoint3D 
//            XnPoint3D ptJointPosReal_[16];
//            XnPoint3D ptJointPosProjective_[16];
//            XnSkeletonJointPosition jointPos_[15];
//            XnSkeletonJointOrientation JointOr_[15];

//            XnUserID aUsers_[15];
//            XnUInt16 nUsers_;
            std::vector<NISkeleton::NISkeletonUserDetected> skeletonUserDetectedVec_;           
            boost::mutex NISkeletonUserDetectedMutex_;

//end used in updateSkeletonData

//used in result transmission by xmlrpc
            int NISkeletonJointNbrMax_;
            
            cv::Mat NISkeletonRefToHeadTemp_;

            std::vector<cv::Mat> NISkeletonPosInTorsoTranslation_;
            std::vector<cv::Mat> NISkeletonPosInCamTranslation_;

//end used in result transmission by xmlrpc
    };
} // namespace skeletonTrackerVSNI

#endif
