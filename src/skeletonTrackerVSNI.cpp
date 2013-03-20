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
//used to access the member of the class skeletonTrackerVSNI inside the static callback function of OpenNI
    SkeletonTrackerVSNI* me_ = 0;

    SkeletonTrackerVSNI::SkeletonTrackerVSNI( visionsystem::VisionSystem *vs, string sandbox )
    : Plugin( vs, "skeletonTrackerVSNI", sandbox ), WithViewer(vs),
    XmlRpcServerMethod("GetObjectPosition", 0),
    imgXi_(0), imgXd_(0),imgDispXi_(0), imgDispXd_(0),
    sandbox_(sandbox)
    {
        me_ = this;

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

        dataToLoadOpenCV["maxUserDetected_"] >> maxUserDetected_;

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

        dataToLoadOpenCV.release();

        if( !NISkeletonFacesCascade_.load( sandbox_ + NISkeletonFacesCascadeName_ ) )
        { 
            printf("--(!)Error loading NISkeletonFacesCascade_\n"); 
        }

//end used for detect faces in NISkeleton

        std::cout<<"coshellName_ "<<coshellName_<<std::endl;

        coshellVision_ = new coshell::CoshellClient(coshellName_, 2809, false);
        //We have to  initialize the coshell since we dont use coshell interpreter
        coshellVision_->Initialize();

        NISkeletonMatMono_.create(480,640, CV_8UC1);

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

        initUserGen();

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

    void SkeletonTrackerVSNI::initUserGen()
    {
        XnStatus nRetVal = XN_STATUS_OK;

        userGen_.Create(*context_);

    //dans maim
        if (!userGen_.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
        {
            printf("Supplied user generator doesn't support skeleton\n");
            //return 1;
        }
        else
        {
            printf("Supplied user generator support skeleton\n");
        }

        nRetVal = userGen_.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks_);
    //    CHECK_RC(nRetVal, "Register to user callbacks");

        userGen_.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);
        printf("user generetor created and initialized but not started\n");

    //need this to ensure that the userGenerator generate data since he is not created at the  same time that the note in the vs_core openni controller
    //    context_->StartGeneratingAll();i

   } 

    void SkeletonTrackerVSNI::startUserGen()
    {
        userGen_.StartGenerating();

        XnBool checkGenerate = userGen_.IsGenerating();
        if(!checkGenerate)
        {
            std::cout<<"userGenerator failed to start"<<std::endl;
        }
        else
        {
            std::cout<<"userGenerator succeed to start"<<std::endl;
        }
    }

    void SkeletonTrackerVSNI::stopUserGen()
    {
    }

    // Callback: New user was detected
    void SkeletonTrackerVSNI::User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
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

        XnStatus rc = me_->userGen_.GetSkeletonCap().LoadCalibrationDataFromFile(nId, userCalibrationFilenPath.str().c_str());
        if (rc == XN_STATUS_OK)
        {
            // Make sure state is coherent
            me_->userGen_.GetPoseDetectionCap().StopPoseDetection(nId);
            me_->userGen_.GetSkeletonCap().StartTracking(nId);

            if (me_->userGen_.GetSkeletonCap().IsTracking(nId))
            {
                printf(" tracked! and put in the check list\n");
////                struct NISkeletonUserDetected
////                {
////                    int nId;
////                    bool toCheck;
////                    int nbrOfDetection;
////                    int frameChecked;
////                    int toTrack;
////                };                
//                //    return;
                
                {
                    boost::mutex::scoped_lock lock(me_->NISkeletonUserDetectedVecMutex_);
        
                    NISkeleton::NISkeletonUserDetected userDetected;
    
                    userDetected.nId = nId;
                    userDetected.toCheck = 1;
                    userDetected.nbrOfDetection = 0;
                    userDetected.frameChecked = 0;
                    userDetected.toTrack = 0;
    
                    me_->userDetectedVec_[nId] = userDetected;
                }
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
    void SkeletonTrackerVSNI::User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
    {
        XnUInt32 epochTime = 0;
        xnOSGetEpochTime(&epochTime);
        printf("%d Lost user %d\n", epochTime, nId);
     
        {
            boost::mutex::scoped_lock lock(me_->NISkeletonUserDetectedVecMutex_);
            
            int sizeVec = me_->userDetectedVec_.size(); 
        }
    }

    bool checkFace();

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
//                    std::memcpy(NiSkeletonMatImgMono_->imageData,imgXiMONO_->raw_data, imgXiMONO_->width * imgXiMONO_->height);
//                    boost::mutex::scoped_lock lock(NISkeletonMatMonoMutex_);
                    std::memcpy(NISkeletonMatMono_.data,imgXiMONO_->raw_data, imgXiMONO_->width * imgXiMONO_->height);
                    cv::equalizeHist( NISkeletonMatMono_, NISkeletonMatMono_ );
//                    imwrite( "/home/petit/test_mat_image.png", NISkeletonMatMono_ );
//
//                    std::vector<int> compression_params;
//                    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
////                    compression_params.push_back(CV_IMWRITE_PXM_BINARY);
//                    compression_params.push_back(0);
//
//                    try 
//                    {
//                        imwrite("/home/petit/test_mat_image2.png", NISkeletonMatMono_, compression_params);
//                    }
//                    catch (runtime_error& ex) 
//                    {
//                        fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
//                        //return 1;
//                    }
//
//                    exit(1);
//                    Mat NISkeletonMatMono(NiSkeletonMatImgMono_);

//                    imgProc_.detectFace(imgXi_);
                    faceDetection();

                }
            }

            t_ = boost::thread(boost::bind(&SkeletonTrackerVSNI::calculs, this));
        }
        enqueue_image< vision::Image<uint32_t, vision::RGB> >(camXi_, imgXi);
        enqueue_image< vision::Image<uint16_t, vision::DEPTH> >(camXd_, imgXd);
    }

    void SkeletonTrackerVSNI::calculs()
    {
//          faceDetection();

//        XnSkeletonJointPosition jointPos[2];
//        XnPoint3D ptJointPos[2];
//
//        XnUserID aUsers[15];
//        XnUInt16 nUsers = 15;
//        userGen_.GetUsers(aUsers, nUsers);
//
//        std::vector<NISkeleton::NISkeletonUserJoints> skeletonDetected;
//
//        for (int iUserDetected = 0; iUserDetected < nUsers; ++iUserDetected)
//        {
//            userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_HEAD, jointPos[0]);
//            userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_NECK, jointPos[1]);
////            ptJointPos[iBodyPart] = jointPos[iBodyPart].position;
//        }
//        
////        std::cout<<"jointPos[0] "<<(int)(jointPos[0])<<"jointPos[1]"<<(int)(jointPos[1])<<std::endl;

        finish_ = true;

    }

    void SkeletonTrackerVSNI::faceDetection()
    {
        NISkeletonFacesCascade_.detectMultiScale( NISkeletonMatMono_, NISkeletonFaces_, NISkeletonScaleFactorFace_, NISkeletonMinNeighborsFace_, NISkeletonFlagsFace_, NISkeletonMinSizeFace_ , NISkeletonMaxSizeFace_ );

        {
            boost::mutex::scoped_lock lock(NISkeletonFacesResultMutex_);
            NISkeletonFacesResult_ = NISkeletonFaces_;
        }

        int facesNbr = NISkeletonFaces_.size();

std::cout<<"NISkeletonFaces_"<<facesNbr<<std::endl;
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
        std::vector<cv::Rect> NISkeletonFacesResult;

        {
            boost::mutex::scoped_lock lock(NISkeletonFacesResultMutex_);
            NISkeletonFacesResult = NISkeletonFacesResult_;
        }


        for(int i = 0; i < NISkeletonFacesResult.size(); i++)
        {
            if(NISkeletonFacesResult.size() != 0)
            {
                 glColor3f(1,0,0);
                 glLineWidth(6.0);
                 glBegin(GL_LINE_LOOP);
                 glVertex2d(NISkeletonFacesResult[i].x, NISkeletonFacesResult[i].y);
                 glVertex2d(NISkeletonFacesResult[i].x + NISkeletonFacesResult[i].width, NISkeletonFacesResult[i].y);
                 glVertex2d(NISkeletonFacesResult[i].x + NISkeletonFacesResult[i].width, NISkeletonFacesResult[i].y + NISkeletonFacesResult[i].height );
                 glVertex2d(NISkeletonFacesResult[i].x, NISkeletonFacesResult[i].y + NISkeletonFacesResult[i].height);
                 glEnd();
            }           
         }
        //std::cout<<"facesResult.size()"<<facesResult.size()<< std::endl;
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
