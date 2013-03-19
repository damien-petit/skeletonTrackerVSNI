#ifndef SKELETON_TRACKER_VSNI
#define SKELETON_TRACKER_VSNI

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

#include "NISkeletonProc.h"

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


        private:

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
            xn::DepthMetaData* depthMD_;

// end used by the openni skeleton tracker function

            std::string coshellName_;
            coshell::CoshellClient * coshellVision_;
    };
} // namespace skeletonTrackerVSNI

#endif
