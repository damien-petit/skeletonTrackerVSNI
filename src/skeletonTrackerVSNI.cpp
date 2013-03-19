#include "skeletonTrackerVSNI.h"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <map>
#include <fstream>
#include <GL/gl.h>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <ctime>

#include <visionsystem/vs_plugins/xmlrpc/xmlrpc-server.h>

#include <time.h>

#include <visionsystem/vs_controllers/openni/cameraopenni.h>

inline void time_diff(const timeval & tv_in, const timeval & tv_out, timeval & tv_diff)
{
    if( tv_out.tv_sec < tv_in.tv_sec || (tv_out.tv_sec == tv_in.tv_sec && tv_out.tv_usec < tv_in.tv_usec) )
    {
        time_diff(tv_out, tv_in, tv_diff);
    }
    else
    {
        if(tv_out.tv_usec < tv_in.tv_usec)
        {
            tv_diff.tv_sec = tv_out.tv_sec - tv_in.tv_sec - 1;
            tv_diff.tv_usec = 1000000 - tv_in.tv_usec + tv_out.tv_usec;
        }
        else
        {
            tv_diff.tv_sec = tv_out.tv_sec - tv_in.tv_sec;
            tv_diff.tv_usec = tv_out.tv_usec - tv_in.tv_usec;
        }
    }
}

#define TIME_CALL(x) \
{ \
timeval tv_in;\
timeval tv_out;\
timeval tv_diff;\
gettimeofday(&tv_in, 0);\
x;\
gettimeofday(&tv_out, 0);\
time_diff(tv_in, tv_out, tv_diff);\
std::cout << "Call " << #x << " took: " << (tv_diff.tv_sec + ((float)tv_diff.tv_usec/1000000)) << "s" << std::endl;\
}

namespace skeletonTrackerVSNI
{
    SkeletonTrackerVSNI::SkeletonTrackerVSNI( visionsystem::VisionSystem *vs, string sandbox )
    : Plugin( vs, "skeletonTrackerVSNI", sandbox ), WithViewer(vs),
    XmlRpcServerMethod("GetObjectPosition", 0),
    imgXi_(0), imgXd_(0),imgDispXi_(0), imgDispXd_(0)
    {
        cv::FileStorage dataToLoadOpenCV;

        if(!dataToLoadOpenCV.open(sandbox +"dataToLoadOpenCVFile.yml", cv::FileStorage::READ))
        {
            std::cout<<"ERROR during the opening of the file dataCalibCameraLeftOpenCV.yml"<<std::endl;
        }

        //initialisation of the different camera matrix calibration        

        dataToLoadOpenCV["cameraMatrixXtionRGB"] >> cameraMatrixXtionRGB_;
        dataToLoadOpenCV["distorsionMatrixXtionRGB"] >> distorsionMatrixXtionRGB_;
        dataToLoadOpenCV["headToCamMatrixXtionRGB"] >> headToCamMatrixXtionRGB_;

        dataToLoadOpenCV["coshellName_"] >> coshellName_;

        dataToLoadOpenCV.release();

        std::cout<<"coshellName_ "<<coshellName_<<std::endl;

        coshellVision_ = new coshell::CoshellClient(coshellName_, 2809, false);
        //We have to  initialize the coshell since we dont use coshell interpreter
        coshellVision_->Initialize();
    
    }

    SkeletonTrackerVSNI::~SkeletonTrackerVSNI()
    {
        delete imgXi_;
        delete imgDispXi_;
        delete imgXd_;
        delete imgDispXd_;
        delete coshellVision_;
    }

    inline void rgb_to_nb(const vision::Image<uint32_t, vision::RGB> & image, vision::Image<unsigned char, vision::MONO> & imageNB)
    {
        uint32_t rgb = 0;
        for(unsigned int i=0; i< image.pixels; ++i)
        {
            rgb = image.raw_data[i];
            imageNB.raw_data[i] =  (unsigned char)( ( ((rgb & 0xFF0000) >> 16)+ ((rgb & 0x00FF00) >> 8) + (rgb & 0x0000FF) )/3 ) ;
        }
    }

    void SkeletonTrackerVSNI::parse_config_line( vector<string> &line )
    {
        if( fill_member(line, "xtion-image", camNameXi_) )
            return;
        if( fill_member(line, "xtion-depth", camNameXd_) )
            return;

    }

    bool SkeletonTrackerVSNI::pre_fct()
    {
        cout << "[skeletonTrackerVSNI] pre_fct()" << endl ;
        string filename = get_sandbox() + string("/skeletonTrackerVSNI.conf") ;

        try 
        {
            read_config_file ( filename.c_str() ) ;
        } 
        catch ( string msg ) 
        {
            throw(std::string("skeletonTrackerVSNI will not work without a correct skeletonTrackerVSNI.conf config file"));
        }

        std::cout<<" camNameXi_ "<<camNameXi_<<std::endl;
        std::cout<<" camNameXd_ "<<camNameXd_<<std::endl;

        camXi_ = get_camera(camNameXi_);
        camXd_ = get_camera(camNameXd_);

        if(camXi_ == 0 || camXd_ == 0)
        {
            throw(std::string("skeletonTrackerVSNI expect one image camera and one depth camera"));
        }

        register_to_cam< vision::Image<uint32_t, vision::RGB> >(camXi_, 100);
        imgXi_ = new vision::Image<uint32_t, vision::RGB>(camXi_->get_size());
        imgDispXi_ = new vision::Image<uint32_t, vision::RGB>(camXi_->get_size());

        imgXiMONO_ = new vision::Image<uint8_t, vision::MONO>(camXi_->get_size());
//        NiSkeletonMatImgMono_ = cvCreateImage(cvSize(imgXi_->width,imgXi_->height), IPL_DEPTH_8U, 1);

        register_to_cam< vision::Image<uint16_t, vision::DEPTH> >(camXd_, 100);
        imgXd_ = new vision::Image<uint16_t, vision::DEPTH>(camXd_->get_size());
        imgDispXd_ = new vision::Image<uint16_t, vision::DEPTH>(camXd_->get_size());

        visionsystem::CameraDepthOpenNI* camXd = 0;
        camXd = dynamic_cast<visionsystem::CameraDepthOpenNI*>(camXd_);
        {
            context_ = camXd->get_Context();
            depth_ = camXd->get_DepthGenerator();
            depthMD_ =  camXd->get_DepthMetaData();
        }


        register_glfunc();

        finish_ = true;

        whiteboard_write<SkeletonTrackerVSNI *>("plugin_skeletonTrackerVSNI", this);

        return true ;
    }

    void SkeletonTrackerVSNI::preloop_fct()
    {
        try
        {
            visionsystem::XMLRPCServer * server = whiteboard_read<visionsystem::XMLRPCServer *>("plugin_xmlrpc-server");
            if(server)
            {
                server->AddMethod(this);
            }
        }
        catch(...)
        {
            std::cout << "[skeletonTrackerVSNI] No XML-RPC server plugin registered to the server, no biggies" << std::endl;
        }
    }

    void SkeletonTrackerVSNI::loop_fct()
    {
        vision::Image<uint32_t, vision::RGB> * imgXi = this->dequeue_image< vision::Image<uint32_t, vision::RGB> > (camXi_);
        vision::Image<uint16_t, vision::DEPTH> * imgXd = this->dequeue_image< vision::Image<uint16_t, vision::DEPTH> > (camXd_);
        if(finish_){
            finish_ = false;
            t_.join();
            vision::Image<uint32_t, vision::RGB> * tmpXi = imgDispXi_;
            vision::Image<uint16_t, vision::DEPTH> * tmpXd = imgDispXd_;
            imgDispXi_ = imgXi_;
            imgDispXd_ = imgXd_;
            imgXi_ = tmpXi;
            imgXd_ = tmpXd;

            imgXi_->copy(imgXi);
            imgXd_->copy(imgXd);

            {   
            rgb_to_nb(*imgXi_, *imgXiMONO_);
//raw copy from Image to IplImage    

                //std::memcpy(TheInputImage.data,img_->raw_data, img_->width * img_->height);
                {
//  //                    std::memcpy(NiSkeletonMatImgMono_->imageData,imgXiMONO_->raw_data, imgXiMONO_->width * imgXiMONO_->height);
//                      boost::mutex::scoped_lock lock(NISkeletonMatMonoMutex_);
//                      std::memcpy(NISkeletonMatMono_.data,imgXiMONO_->raw_data, imgXiMONO_->width * imgXiMONO_->height);
//                      cv::equalizeHist( NISkeletonMatMono_, NISkeletonMatMono_ );
//  //                    imwrite( "/home/petit/test_mat_image.png", NISkeletonMatMono_ );
//  //
//  //                    std::vector<int> compression_params;
//  //                    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//  ////                    compression_params.push_back(CV_IMWRITE_PXM_BINARY);
//  //                    compression_params.push_back(0);
//  //
//  //                    try 
//  //                    {
//  //                        imwrite("/home/petit/test_mat_image2.png", NISkeletonMatMono_, compression_params);
//  //                    }
//  //                    catch (runtime_error& ex) 
//  //                    {
//  //                        fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
//  //                        //return 1;
//  //                    }
//  //
//  //                    exit(1);
//  //                    Mat NISkeletonMatMono(NiSkeletonMatImgMono_);
//  
//  //                    imgProc_.detectFace(imgXi_);
                }
            }

            t_ = boost::thread(boost::bind(&SkeletonTrackerVSNI::calculs, this));
        }
        enqueue_image< vision::Image<uint32_t, vision::RGB> >(camXi_, imgXi);
        enqueue_image< vision::Image<uint16_t, vision::DEPTH> >(camXd_, imgXd);
    }

    void SkeletonTrackerVSNI::calculs()
    {

        finish_ = true;

    }

    void SkeletonTrackerVSNI::callback(visionsystem::Camera* cam, XEvent event)
    {
    }

    void SkeletonTrackerVSNI::glfunc(visionsystem::Camera* cam)
    {
//    //glDrawPixels( imgDispL_->width, imgDispL_->height, GL_RGBA, GL_UNSIGNED_BYTE, imgDispL_->raw_data );
//        std::vector<cv::Rect> facesResult;
////        {
////            boost::mutex::scoped_lock lock(*FaceDetectResultMutexPtrVision_);
//
//            imgProc_.FaceDetectProc_.GetFacesResult( facesResult );
////            imgProc_.FaceDetectProc_.GetEyesResult( eyesResult, eyesCenterResult );
////        }
//
//        for(int i = 0; i < facesResult.size(); i++)
//        {
//            glColor3f(1,0,0);
//            glLineWidth(6.0);
//            glBegin(GL_LINE_LOOP);
//            glVertex2d(facesResult[i].x, facesResult[i].y);
//            glVertex2d(facesResult[i].x + facesResult[i].width, facesResult[i].y);
//            glVertex2d(facesResult[i].x + facesResult[i].width, facesResult[i].y + facesResult[i].height );
//            glVertex2d(facesResult[i].x, facesResult[i].y + facesResult[i].height);
//            glEnd();
//         }
//        //std::cout<<"facesResult.size()"<<facesResult.size()<< std::endl;
    }

    bool SkeletonTrackerVSNI::post_fct()
    {
        cout << "[bci-self-interact-vision] post_fct()" << endl ;
        unregister_to_cam< vision::Image<uint32_t, vision::RGB> >( camXi_ );
        unregister_to_cam< vision::Image<uint16_t, vision::DEPTH> >( camXd_ );
        return true ;
    }

    void SkeletonTrackerVSNI::execute(XmlRpcValue & params, XmlRpcValue & result)
    {
////TODO selection de fonction dans params
////TODO creation d un evenement par etat de visionFSM
//
//        //std::cout<<"[SkeletonTrackerVSNI] XmlRpc execute"<<std::endl;
//
//        //std::cout <<"[SkeletonTrackerVSNI]"<<" XmlRpcValue &  params "<<params[0]["MethodName"] << std::endl;
//
//        std::string methodName(params[0]["MethodName"]);
//
//        //std::cout<<"[SkeletonTrackerVSNI] params[0][\"MethodName\"]"<<params[0]["MethodName"]<<std::endl;
//
//        if(methodName == "StateVisionFSM")
//        {
////Select the state desired
//            std::string stateVisionFSM(params[0]["StateVisionFSM"]);
//
////TODO tester si l'etat dans lequel on est different de l etat souhaite
////TODO tester si on est dans l'etat StateCoice avent de changer d' etat dans lequel on est different de l etat souhaite ou faire comme l onfait actuellement et revenir automatiquement a StateChoice
//
//            if(stateVisionFSM == "StateARCode")
//            {
///*use an other thread for the process_event. If not the process_event will block this scope because of the while loop in the state of the state machine*/
///*If this scope is block the execute method of xmplrpc will never return a value and so the xmlrpc server will be block and no other request from the client will be treated*/
//                nextImgProcMethod_ = 1;
//            }
//            else if(stateVisionFSM == "StateImageProcessing1")
//            {
//                nextImgProcMethod_ = 2;
//            }
//            else if(stateVisionFSM == "StateImageProcessing2")
//            {
//                nextImgProcMethod_ = 3;
//            }
//            else if(stateVisionFSM == "StateNISkeleton")
//            {
//                nextImgProcMethod_ = 4;
//            }
//        }
//        if(methodName == "GetObjectPositionARCode")
//        {
//            GetObjectPositionARCode(params, result);
//        }
//        else if(methodName == "GetObjectPositionNISkeleton")
//        {
////            std::cout<<"GetObjectPositionNISkeleton(params, result) called in execute function"<<std::endl;
//            GetObjectPositionNISkeleton(params, result);
//        }
    }

    bool SkeletonTrackerVSNI::GetObjectPositionNISkeleton(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result)
    {
//        //result is already allocated inn bci-self-interact
//        //thats why we use it directly
//        imgProc_.GetNISkeletonResult( result );
    }

} //end namespace skeletonTrackerVSNI

PLUGIN(skeletonTrackerVSNI::SkeletonTrackerVSNI)
