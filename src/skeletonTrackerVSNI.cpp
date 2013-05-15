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

//        std::cout<<cameraMatrixXtionRGB_<<std::endl;
//        std::cout<<headToCamMatrixXtionRGB_<<std::endl;

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

        dataToLoadOpenCV["maxSizeDeque_"] >> maxSizeDeque_;
        fullDeque_ = false;
//end used for detect faces in NISkeleton

        dataToLoadOpenCV["NISkeletonJointNbrMax_"] >> NISkeletonJointNbrMax_;
//RISK
        if(maxSizeDeque_ < 1)
        {
            std::cout<<"[skeletonTrackerVSNI] problem maxSizeDeque_ value should be at least 1"<<std::endl;
        }

        dataToLoadOpenCV.release();

// used in the resul;t transmission function by xmlrpc

//the +1 is for the com of the user detected
        NISkeletonPosInCamTranslation_.resize(NISkeletonJointNbrMax_+1);
        NISkeletonposInTorsoTranslation_.resize(NISkeletonJointNbrMax_+1);

        NISkeletonRefToHeadTemp_ = cv::Mat::zeros(4,4,CV_32F);

        for(int i = 0; i < NISkeletonJointNbrMax_+1; i++)
        {
            NISkeletonPosInCamTranslation_[i] = cv::Mat::zeros(4, 1, CV_32F);
            NISkeletonposInTorsoTranslation_[i] = cv::Mat::zeros(4, 1, CV_32F);
        }

        if( !NISkeletonFacesCascade_.load( sandbox_ + NISkeletonFacesCascadeName_ ) )
        { 
            printf("--(!)Error loading NISkeletonFacesCascade_\n"); 
        }

        std::cout<<"coshellName_ "<<coshellName_<<std::endl;

        coshellVision_ = new coshell::CoshellClient(coshellName_, 2809, false);
        //We have to  initialize the coshell since we dont use coshell interpreter
        coshellVision_->Initialize();

        NISkeletonMatMono_.create(480,640, CV_8UC1);

//used to corespond the nId to the index of the vector
        skeletonUserDetectedVec_.resize(maxUserDetected_);

//        nUsers_ = 15;
        userIdSelected_ = -1;
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
        std::cout << "[skeletonTrackerVSNI] pre_fct()" << endl ;
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

//        XnInt32 nMin;
//        XnInt32 nMax;
//        XnInt32 nStep;
//        int nStep;
//        XnInt32 nDefault;
//        XnBool autoSupported;

////        if(image_->IsCapabilitySupported(XN_CAPABILITY_LOW_LIGHT_COMPENSATION))
//        if(image_->IsCapabilitySupported(XN_CAPABILITY_HUE))
//        {
//            printf("Supplied image generator support LOW_LIGHT_COMPENSATION\n");
//            //return 1;
//        }
//        else
//        {
//            printf("Supplied image generator doesn't support LOW_LIGHT_COMPENSATION\n");
//        }
//
////for(int iCamIter = 0; iCamIter < 10; iCamIter++)
////{
////        image_->GetLowLightCompensationCap().GetRange(nMin, nMax, nStep, nDefault, autoSupported);
////
////        std::cout<<"nMin "<<(int)nMin<<" nMax "<<(int)nMax<<" nStep "<< (int)nStep <<" nDefault "<<(int)nDefault<<" autoSupported "<<(bool)autoSupported<<std::endl;
////
////sleep(1);
////} 
//
//        exit(-1);

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

        visionsystem::CameraDepthOpenNI* camXd = 0;
        camXd = dynamic_cast<visionsystem::CameraDepthOpenNI*>(camXd_);

        visionsystem::CameraImageOpenNI* camXi = 0;
        camXi = dynamic_cast<visionsystem::CameraImageOpenNI*>(camXi_);

        context_ = camXd->get_Context();
        depth_ = camXd->get_DepthGenerator();
        depthMD_ =  camXd->get_DepthMetaData();

        image_ = camXi->get_ImageGenerator();

        XnStatus rc = depth_->GetAlternativeViewPointCap().SetViewPoint(*image_); 

        if(rc != XN_STATUS_OK)
        {
             printf("Failed to set depth map mode \n");
        }
        else
        {
             printf("Succeed to set depth map mode \n");
        }

        initUserGen();
        userGenStarted_ = false;
        trackChecked_ = false;

        initHandsGen();
        startHandsGen();
    }

    bool SkeletonTrackerVSNI::SetUserIdSelectedFromInterface(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result)
    {
        int userIdSelected(params[0]["userIdSelected"]);

        userIdSelected_ = userIdSelected; 
    }

    bool SkeletonTrackerVSNI::StartHandTracking(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result)
    {
        XnUserID aUsers[maxUserDetected_ - 1];
        XnUInt16 nUsers = maxUserDetected_ - 1;

        userGen_.GetUsers(aUsers, nUsers);

        XnSkeletonJointPosition jointHandPos;
        XnPoint3D ptHandPos;
    
        if(userGen_.GetSkeletonCap().IsTracking(aUsers[iUserDetected_]))
        {
            userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected_], XN_SKEL_LEFT_HAND, jointHandPos);
    
            if(jointHandPos.fConfidence > 0.5)
            {
                std::cout<<"ask track hand"<<std::endl;
    
                ptHandPos = jointHandPos.position;
    
                handsGen_.StartTracking(ptHandPos);
            }
            else
            {
                std::cout<<"bad hand confidence dont ask track hand"<<std::endl;
            }
        }
        else
        {
    
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

    void SkeletonTrackerVSNI::initHandsGen()
    {
        XnStatus nRetVal = XN_STATUS_OK;

        handsGen_.Create(*context_);

        nRetVal = handsGen_.Create(*context_);

        if(nRetVal != XN_STATUS_OK)
        {
             printf("Failed to create the hands generator \n");
        }
        else
        {
             printf("Succeed to create the hands generator \n");
        }

        nRetVal = handsGen_.RegisterHandCallbacks(Hand_Create, Hand_Update, Hand_Destroy, NULL, hHandCallbacks_);

        if(nRetVal != XN_STATUS_OK)
        {
             printf("Failed to register the callbacks of the hands generator \n");
        }
        else
        {
             printf("Succeed to register the callbacks of the hands generator \n");
        }

        printf("hands generetor created and initialized but not started\n");

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

    void SkeletonTrackerVSNI::startHandsGen()
    {
        handsGen_.StartGenerating();

        XnBool checkGenerate = handsGen_.IsGenerating();
        if(!checkGenerate)
        {
            std::cout<<"handsGenerator failed to start"<<std::endl;
        }
        else
        {
            std::cout<<"handsGenerator succeed to start"<<std::endl;
        }
    }

    void SkeletonTrackerVSNI::stopUserGen()
    {
    }

    void SkeletonTrackerVSNI::stopHandsGen()
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

        XnSkeletonJointPosition jointHeadPos;
        XnPoint3D ptHeadPos;

        int posX = 0;
        int posY = 0;

        XnStatus rc = me_->userGen_.GetSkeletonCap().LoadCalibrationDataFromFile(nId, userCalibrationFilenPath.str().c_str());
        if (rc == XN_STATUS_OK)
        {
//            // Make sure state is coherent
            me_->userGen_.GetPoseDetectionCap().StopPoseDetection(nId);
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

    void SkeletonTrackerVSNI::Hand_Create(xn::HandsGenerator& generator, XnUserID nId, const XnPoint3D* pPosition, XnFloat fTime, void* pCookie)
    {
        printf("New Hand: %d @ (%f,%f,%f)\n", nId, pPosition->X, pPosition->Y, pPosition->Z);
    }

    void SkeletonTrackerVSNI::Hand_Update( xn::HandsGenerator& generator, XnUserID nId, const XnPoint3D* pPosition, XnFloat fTime, void* pCookie)
    {
        printf("Hand Moving: %d @ (%f,%f,%f)\n", nId, pPosition->X, pPosition->Y, pPosition->Z);
    }

    void SkeletonTrackerVSNI::Hand_Destroy(xn::HandsGenerator& generator, XnUserID nId, XnFloat fTime, void* pCookie)
    {
        printf("Lost Hand: %d\n", nId);
        //g_GestureGenerator.AddGesture(GESTURE_TO_USE, NULL);
    }

    void SkeletonTrackerVSNI::loop_fct()
    {
        vision::Image<uint32_t, vision::RGB> * imgXi = this->dequeue_image< vision::Image<uint32_t, vision::RGB> > (camXi_);
        vision::Image<uint16_t, vision::DEPTH> * imgXd = this->dequeue_image< vision::Image<uint16_t, vision::DEPTH> > (camXd_);
//        if(finish_)
        {
//            finish_ = false;
//            t_.join();
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
//                    bool check;
//                    if(checkDeque(320,240))
//                    {
//                        std::cout<<"!!!!!!!!!!!!!!!!!!!1check OK"<<std::endl;
//                    } 
//                    std::cout<<std::endl;
//                    std::cout<<"begin of visual loop"<<std::endl;

                    {
                        XnSkeletonJointPosition jointHeadPos;
                        XnPoint3D ptHeadPos;
                        XnPoint3D ptCOMPos;               
 
                        int posX = 0;
                        int posY = 0;

                        int posCoMX = 0;
                        int posCoMY = 0;
                
                        XnUserID aUsers[maxUserDetected_ - 1];
                        XnUInt16 nUsers = maxUserDetected_ - 1;

                        XnSkeletonJointPosition jointHandPos;
                        XnPoint3D ptHandPos;

                        if(userGen_.IsGenerating())
                        {

                            userGen_.GetUsers(aUsers, nUsers);

                            NISkeletonCOMResult_.resize(nUsers);
//
//        std::vector<NISkeleton::NISkeletonUserJoints> skeletonDetected;
//
                            for (int iUserDetected = 0; iUserDetected < nUsers; ++iUserDetected)
                            {
//use th COM of the user
//                                std::cout<<std::endl;
//                                std::cout<<"userNumber "<<iUserDetected<<std::endl;

                                userGen_.GetCoM(aUsers[iUserDetected], ptCOMPos);

                                depth_->ConvertRealWorldToProjective(1, &ptCOMPos, &ptCOMPos);

                                posCoMX = ptCOMPos.X;
                                posCoMY = ptCOMPos.Y;
        
                                COMResult comResultPerUser;

                                comResultPerUser.nId = aUsers[iUserDetected];
                                comResultPerUser.x = posCoMX;
                                comResultPerUser.y = posCoMY;

                                {
                                    boost::mutex::scoped_lock lock(NISkeletonCOMResultMutex_);
                               
                                    NISkeletonCOMResult_[iUserDetected] = comResultPerUser;
                                }

                                if(userGen_.GetSkeletonCap().IsTracking(aUsers[iUserDetected]))
                                {
                                }
                                else
                                {
                                    if(checkDeque( posCoMX, posCoMY))
                                    {
    //                                    std::cout<<"[from Check COM]check ok"<<std::endl;
                                        me_->trackChecked_ = true;
    //                                    std::cout<<"posCoMX "<<posCoMX<<" posCoMY "<<posCoMY<<std::endl;
                                    //    std::cout<<"trackChecked_ "<<me_->trackChecked_<<std::endl;
    
                                        userGen_.GetSkeletonCap().StartTracking(aUsers[iUserDetected]);
                                    }
                                    else
                                    {
    //                                    std::cout<<"not checked"<<std::endl;
        //                                me_->userGen_.GetSkeletonCap().StopTracking(aUsers[iUserDetected]);
                                    }
                                }
                            } 
                        }
                    }
                }
            }

//            t_ = boost::thread(boost::bind(&SkeletonTrackerVSNI::calculs, this));
        }
        enqueue_image< vision::Image<uint32_t, vision::RGB> >(camXi_, imgXi);
        enqueue_image< vision::Image<uint16_t, vision::DEPTH> >(camXd_, imgXd);
    }

    void SkeletonTrackerVSNI::calculs()
    {
        finish_ = true;
    }

    void SkeletonTrackerVSNI::faceDetection()
    {
        NISkeletonFacesCascade_.detectMultiScale( NISkeletonMatMono_, NISkeletonFaces_, NISkeletonScaleFactorFace_, NISkeletonMinNeighborsFace_, NISkeletonFlagsFace_, NISkeletonMinSizeFace_ , NISkeletonMaxSizeFace_ );

        std::vector<cv::Rect> NISkeletonFaces;

        NISkeletonFaces = NISkeletonFaces_;

        {
            boost::mutex::scoped_lock lock(NISkeletonFacesResultMutex_);
            NISkeletonFacesResult_ = NISkeletonFaces;
        }

        int dequeSize = NISkeletonFacesResultDeque_.size();
        
        if(dequeSize == maxSizeDeque_)
        {
            fullDeque_ = true;
        }

        if(fullDeque_)
        {
            if(!userGenStarted_)
            {
                startUserGen();
                userGenStarted_ = true;  

//                startHandsGen();
            }
            else
            {
            }
        }
        else
        {

        }
//        startUserGen(); 

//        std::cout<<"NISkeletonFacesResultDeque_before"<<dequeSize<<std::endl;
//        std::cout<<"maxSizeDeque_ "<<maxSizeDeque_<<std::endl;       
        
        {
            boost::mutex::scoped_lock lock(NISkeletonFacesResultDequeMutex_);

            if(dequeSize < maxSizeDeque_)
            {
                NISkeletonFacesResultDeque_.push_back(NISkeletonFaces);    
            }
            else
            {
                NISkeletonFacesResultDeque_.pop_front();            
                NISkeletonFacesResultDeque_.push_back(NISkeletonFaces);
            }
        }
    }

    bool SkeletonTrackerVSNI::checkDeque(int xFacePos, int yFacePos)
    {
        std::deque<std::vector< cv::Rect> > NISkeletonFacesResultDeque;
        std::vector< cv::Rect> NISkeletonFacesResult;
        cv::Rect faceRect;

        int leftXFace = 0;
        int rightXFace = 0;

        int topYFace = 0;
        int bottomYFace = 0;

        {
            boost::mutex::scoped_lock lock(NISkeletonFacesResultDequeMutex_);        
        
            NISkeletonFacesResultDeque = NISkeletonFacesResultDeque_;
        }
        
        int sizeDeque = NISkeletonFacesResultDeque.size();

//        std::cout<<"sizeDeque "<<sizeDeque<<std::endl;

        int sizeFaceResult = 0;        
        
//        std::cout<<"OpenNI result"<<std::endl;
//        std::cout<<"X "<< xFacePos<<" Y "<<yFacePos<<std::endl;

//        std::cout<<"OpenCV result"<<std::endl;

        for(int i = 0; i < sizeDeque; i++) 
        {
            NISkeletonFacesResult = NISkeletonFacesResultDeque[i];
        
            sizeFaceResult = NISkeletonFacesResult.size();

//            std::cout<<"nbr of faces detected for deque nbr: "<<i<<" faces "<<sizeFaceResult<<std::endl;

            for(int faceIter = 0; faceIter < sizeFaceResult; faceIter++)
            {

//                std::cout<<"faces nbr: "<<faceIter<<sizeDeque<<std::endl;
                faceRect = NISkeletonFacesResult[faceIter];
            
                leftXFace = faceRect.x;
                rightXFace = faceRect.x + faceRect.width;

                topYFace = faceRect.y;
                bottomYFace = faceRect.y + faceRect.height;     

//                std::cout<<"leftXFace"<< leftXFace << std::endl;
//                std::cout<<"rightXFace"<< rightXFace << std::endl;
//                std::cout<<"topYFace"<< topYFace << std::endl;
//                std::cout<<"bottomYFace"<< bottomYFace << std::endl;

                if(  leftXFace <= xFacePos && xFacePos <= rightXFace )
                {
//                    if( topYFace <= yFacePos && yFacePos <= bottomYFace )
//adapted for COM
                    if( yFacePos >= bottomYFace )
                    {
//                        std::cout<<"[check function]check OK"<<std::endl;
                        return 1;
                    }
                }
            } 
        } 
        return 0;
    }

    void SkeletonTrackerVSNI::callback(visionsystem::Camera* cam, XEvent event)
    {
    }

    void SkeletonTrackerVSNI::glfunc(visionsystem::Camera* cam)
    {
//    //glDrawPixels( imgDispL_->width, imgDispL_->height, GL_RGBA, GL_UNSIGNED_BYTE, imgDispL_->raw_data );

        std::vector<cv::Rect> NISkeletonFacesResult;
        std::vector< COMResult > comResultVec;

        {
            boost::mutex::scoped_lock lock(NISkeletonFacesResultMutex_);
            NISkeletonFacesResult = NISkeletonFacesResult_;
        }

        {
            boost::mutex::scoped_lock lock(NISkeletonCOMResultMutex_);
            comResultVec = NISkeletonCOMResult_;
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

        for(int i = 0; i < comResultVec.size(); i++)
        {
            if(comResultVec.size() != 0)
            {
                glColor3f(0,1,0);
                glLineWidth(6.0);
                glBegin(GL_LINE_LOOP);

                int radius = 10;
                float angle;                

                for(int iAngle = 0; iAngle < 100; iAngle++) 
                { 
                   angle = iAngle*2*M_PI/100; 
                   glVertex2f(comResultVec[i].x + (cos(angle) * radius), comResultVec[i].x + (sin(angle) * radius)); 
                }
                glEnd();
            } 
         }
        //std::cout<<"facesResult.size()"<<facesResult.size()<< std::endl;
    }

    bool SkeletonTrackerVSNI::post_fct()
    {
        std::cout << "[bci-self-interact-vision] post_fct()" << endl ;
        unregister_to_cam< vision::Image<uint32_t, vision::RGB> >( camXi_ );
        unregister_to_cam< vision::Image<uint16_t, vision::DEPTH> >( camXd_ );
        return true ;
    }

    void SkeletonTrackerVSNI::execute(XmlRpcValue & params, XmlRpcValue & result)
    {
//TODO selection de fonction dans params
//TODO creation d un evenement par etat de visionFSM

        std::string methodName(params[0]["MethodName"]);
//FIXME
/*use an other thread for the process_event. If not the process_event will block this scope because of the while loop in the state of the state machine*/
/*If this scope is block the execute method of xmplrpc will never return a value and so the xmlrpc server will be block and no other request from the client will be treated*/
        if(methodName == "GetObjectPositionNISkeleton")
        {
//            std::cout<<"GetObjectPositionNISkeleton(params, result) called in execute function"<<std::endl;
            GetObjectPositionNISkeleton(params, result);
//            std::cout<<"end GetObjectPositionNISkeleton(params, result) called in execute function"<<std::endl;
        }
        else if(methodName == "StartHandTracking")
        {
//            std::cout<<"GetObjectPositionNISkeleton(params, result) called in execute function"<<std::endl;
            StartHandTracking(params, result);
//            std::cout<<"end GetObjectPositionNISkeleton(params, result) called in execute function"<<std::endl;
        }
        else if(methodName == "SetUserIdSelectedFromInterface")
        {
            SetUserIdSelectedFromInterface(params, result);
        }
        else
        {
        }
    }

    bool SkeletonTrackerVSNI::updateSkeletonData(void)
    {
// 15 joints + 1 COM 
//float XnPoint3D
        XnPoint3D ptJointPosReal[16];
        XnPoint3D ptJointPosProjective[16];
        XnSkeletonJointPosition jointPos[15];
        XnSkeletonJointOrientation JointOr[15];

        XnUserID aUsers[maxUserDetected_ - 1];
        XnUInt16 nUsers = maxUserDetected_ - 1;

        userGen_.GetUsers(aUsers, nUsers);

        int userTrackedNbr = 0;

        std::vector<NISkeleton::NISkeletonUserDetected> skeletonUserDetectedVec;
// faire un system comme le deque pour ajouter/supprimer des user tracker et garder un historique des donnees des differents joints des users traquer avt qu il ne soit plus traqué
//        skeletonUserDetectedVec_;
        {
            boost::mutex::scoped_lock lock(NISkeletonUserDetectedMutex_);

            skeletonUserDetectedVec = skeletonUserDetectedVec_;
        } 

        if(nUsers > 0)
        {
            std::cout<<"nUsers "<<(int)nUsers<<std::endl;
            for (int iUserDetected = 0; iUserDetected < nUsers; ++iUserDetected)                
            {
//check if the user has already some track data
                if (userGen_.GetSkeletonCap().IsTracking(aUsers[iUserDetected]))
                {
                    userTrackedNbr++;
                    skeletonUserDetectedVec[iUserDetected].tracked = 1;

                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_HEAD, jointPos[0]);
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_NECK, jointPos[1]);
    
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_SHOULDER, jointPos[2]);
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_ELBOW, jointPos[3]);
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_HAND, jointPos[4]);
    
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_SHOULDER, jointPos[5]);
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_ELBOW, jointPos[6]);
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_HAND, jointPos[7]);
    
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_TORSO, jointPos[8]);
    
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_HIP, jointPos[9]);
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_KNEE, jointPos[10]);
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_FOOT, jointPos[11]);
    
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_HIP, jointPos[12]);
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_KNEE, jointPos[13]);
                    userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_FOOT, jointPos[14]);

//check the confidence of the measure for translation
                    for(int iBodyPart = 0; iBodyPart < 15; iBodyPart++)
                    {
                        if (jointPos[iBodyPart].fConfidence > 0.5)
                        {
        //ptJointPosReal is usefull container because we can use the openNI function for the projection after            
                            ptJointPosReal[iBodyPart] = jointPos[iBodyPart].position;
        //ptJointPos return float
                            NISkeletonPosInCamTranslation_[iBodyPart].at<float>(0,0) = ptJointPosReal[iBodyPart].X;
                            NISkeletonPosInCamTranslation_[iBodyPart].at<float>(1,0) = ptJointPosReal[iBodyPart].Y;
                            NISkeletonPosInCamTranslation_[iBodyPart].at<float>(2,0) = ptJointPosReal[iBodyPart].Z;
                            NISkeletonPosInCamTranslation_[iBodyPart].at<float>(3,0) = 1;
        //a examiner le calcul effectuei
        //regader la doc file:///home/petit/Downloads/OpenNI-Bin-ev-Linux-x86-v1.5.4.0/Documentation/html/conc_coord.html
        //converti les donnees en pixel pour x et y z est tou jours en millimetre les origin en x et y son change du centre de fov a upper-left corner of the FOV
                        }
                        else
                        {
                            NISkeletonPosInCamTranslation_[iBodyPart].at<float>(0,0) = 0;
                            NISkeletonPosInCamTranslation_[iBodyPart].at<float>(1,0) = 0;
                            NISkeletonPosInCamTranslation_[iBodyPart].at<float>(2,0) = 0;
                            NISkeletonPosInCamTranslation_[iBodyPart].at<float>(3,0) = 1; 
                        }
                    }

                    userGen_.GetCoM(aUsers[iUserDetected], ptJointPosReal[15]);
        
        //compute the joint detected in the waist position
        
                    NISkeletonRefToHeadTemp_.at<float>(3,0) = 0;
                    NISkeletonRefToHeadTemp_.at<float>(3,1) = 0;
                    NISkeletonRefToHeadTemp_.at<float>(3,2) = 0;
                    NISkeletonRefToHeadTemp_.at<float>(3,3) = 1;
        
                    {
                         std::stringstream ss;
                         ss << coshellVision_->ExecuteACommand("dyn2.head");
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
        //                std::cout<<"iBodyPart "<<iBodyPart<<std::endl;
        //                std::cout<<"NISkeletonRefToHeadTemp_"<<iBodyPart<<std::endl;
        //                std::cout<<NISkeletonRefToHeadTemp_<<std::endl;
        //                std::cout<<"headToCamMatrixXtionRGB_"<<std::endl;
        //                std::cout<<headToCamMatrixXtionRGB_<<std::endl;
        //                std::cout<<"NISkeletonPosInCamTranslation_"<<std::endl;
        //                std::cout<<NISkeletonPosInCamTranslation_[iBodyPart]<<std::endl;
                        NISkeletonposInTorsoTranslation_[iBodyPart] = (NISkeletonRefToHeadTemp_*headToCamMatrixXtionRGB_)*NISkeletonPosInCamTranslation_[iBodyPart];
                     }

//RealWorld Position in torsoreference
                    {
//                        std::vector<NISkeleton::NISkeletonUserJoints> skeletonUserJointsVec_;
//                        boost::mutex NISkeletonUserJointsMutex_;
                        skeletonUserDetectedVec[iUserDetected].joints.head.waistPos.x = (double)NISkeletonposInTorsoTranslation_[0].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.head.waistPos.y = (double)NISkeletonposInTorsoTranslation_[0].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.head.waistPos.z = (double)NISkeletonposInTorsoTranslation_[0].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.neck.waistPos.x = (double)NISkeletonposInTorsoTranslation_[1].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.neck.waistPos.y = (double)NISkeletonposInTorsoTranslation_[1].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.neck.waistPos.z = (double)NISkeletonposInTorsoTranslation_[1].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.leftShoulder.waistPos.x = (double)NISkeletonposInTorsoTranslation_[2].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftShoulder.waistPos.y = (double)NISkeletonposInTorsoTranslation_[2].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftShoulder.waistPos.z = (double)NISkeletonposInTorsoTranslation_[2].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.leftElbow.waistPos.x = (double)NISkeletonposInTorsoTranslation_[3].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftElbow.waistPos.y = (double)NISkeletonposInTorsoTranslation_[3].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftElbow.waistPos.z = (double)NISkeletonposInTorsoTranslation_[3].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.leftHand.waistPos.x = (double)NISkeletonposInTorsoTranslation_[4].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftHand.waistPos.y = (double)NISkeletonposInTorsoTranslation_[4].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftHand.waistPos.z = (double)NISkeletonposInTorsoTranslation_[4].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.rightShoulder.waistPos.x = (double)NISkeletonposInTorsoTranslation_[5].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightShoulder.waistPos.y = (double)NISkeletonposInTorsoTranslation_[5].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightShoulder.waistPos.z = (double)NISkeletonposInTorsoTranslation_[5].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.rightElbow.waistPos.x = (double)NISkeletonposInTorsoTranslation_[6].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightElbow.waistPos.y = (double)NISkeletonposInTorsoTranslation_[6].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightElbow.waistPos.z = (double)NISkeletonposInTorsoTranslation_[6].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.rightHand.waistPos.x = (double)NISkeletonposInTorsoTranslation_[7].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightHand.waistPos.y = (double)NISkeletonposInTorsoTranslation_[7].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightHand.waistPos.z = (double)NISkeletonposInTorsoTranslation_[7].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.torso.waistPos.x = (double)NISkeletonposInTorsoTranslation_[8].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.torso.waistPos.y = (double)NISkeletonposInTorsoTranslation_[8].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.torso.waistPos.z = (double)NISkeletonposInTorsoTranslation_[8].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.leftHip.waistPos.x = (double)NISkeletonposInTorsoTranslation_[9].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftHip.waistPos.y = (double)NISkeletonposInTorsoTranslation_[9].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftHip.waistPos.z = (double)NISkeletonposInTorsoTranslation_[9].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.leftKnee.waistPos.x = (double)NISkeletonposInTorsoTranslation_[10].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftKnee.waistPos.y = (double)NISkeletonposInTorsoTranslation_[10].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftKnee.waistPos.z = (double)NISkeletonposInTorsoTranslation_[10].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.leftFoot.waistPos.x = (double)NISkeletonposInTorsoTranslation_[11].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftFoot.waistPos.y = (double)NISkeletonposInTorsoTranslation_[11].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.leftFoot.waistPos.z = (double)NISkeletonposInTorsoTranslation_[11].at<float>(2,0);
        
        
                        skeletonUserDetectedVec[iUserDetected].joints.rightHip.waistPos.x = (double)NISkeletonposInTorsoTranslation_[12].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightHip.waistPos.y = (double)NISkeletonposInTorsoTranslation_[12].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightHip.waistPos.z = (double)NISkeletonposInTorsoTranslation_[12].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.rightKnee.waistPos.x = (double)NISkeletonposInTorsoTranslation_[13].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightKnee.waistPos.y = (double)NISkeletonposInTorsoTranslation_[13].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightKnee.waistPos.z = (double)NISkeletonposInTorsoTranslation_[13].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.rightFoot.waistPos.x = (double)NISkeletonposInTorsoTranslation_[14].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightFoot.waistPos.y = (double)NISkeletonposInTorsoTranslation_[14].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.rightFoot.waistPos.z = (double)NISkeletonposInTorsoTranslation_[14].at<float>(2,0);
        
                        skeletonUserDetectedVec[iUserDetected].joints.COM.waistPos.x = (double)NISkeletonposInTorsoTranslation_[15].at<float>(0,0);
                        skeletonUserDetectedVec[iUserDetected].joints.COM.waistPos.y = (double)NISkeletonposInTorsoTranslation_[15].at<float>(1,0);
                        skeletonUserDetectedVec[iUserDetected].joints.COM.waistPos.z = (double)NISkeletonposInTorsoTranslation_[15].at<float>(2,0);
        
                        depth_->ConvertRealWorldToProjective(16, ptJointPosReal, ptJointPosProjective);
        
            //Pixel Position
        
                        skeletonUserDetectedVec[iUserDetected].joints.head.pixelPos.x = (int)ptJointPosProjective[0].X;
                        skeletonUserDetectedVec[iUserDetected].joints.head.pixelPos.y = (int)ptJointPosProjective[0].Y;
        
                        skeletonUserDetectedVec[iUserDetected].joints.neck.pixelPos.x = (int)ptJointPosProjective[1].X;
                        skeletonUserDetectedVec[iUserDetected].joints.neck.pixelPos.y = (int)ptJointPosProjective[1].Y;
        
                        skeletonUserDetectedVec[iUserDetected].joints.leftShoulder.pixelPos.x = (int)ptJointPosProjective[2].X;
                        skeletonUserDetectedVec[iUserDetected].joints.leftShoulder.pixelPos.y = (int)ptJointPosProjective[2].Y;
                        skeletonUserDetectedVec[iUserDetected].joints.leftElbow.pixelPos.x = (int)ptJointPosProjective[3].X;
                        skeletonUserDetectedVec[iUserDetected].joints.leftElbow.pixelPos.y = (int)ptJointPosProjective[3].Y;
                        skeletonUserDetectedVec[iUserDetected].joints.leftHand.pixelPos.x = (int)ptJointPosProjective[4].X;
                        skeletonUserDetectedVec[iUserDetected].joints.leftHand.pixelPos.y = (int)ptJointPosProjective[4].Y;
        
                        skeletonUserDetectedVec[iUserDetected].joints.rightShoulder.pixelPos.x = (int)ptJointPosProjective[5].X;
                        skeletonUserDetectedVec[iUserDetected].joints.rightShoulder.pixelPos.y = (int)ptJointPosProjective[5].Y;
                        skeletonUserDetectedVec[iUserDetected].joints.rightElbow.pixelPos.x = (int)ptJointPosProjective[6].X;
                        skeletonUserDetectedVec[iUserDetected].joints.rightElbow.pixelPos.y = (int)ptJointPosProjective[6].Y;
                        skeletonUserDetectedVec[iUserDetected].joints.rightHand.pixelPos.x = (int)ptJointPosProjective[7].X;
                        skeletonUserDetectedVec[iUserDetected].joints.rightHand.pixelPos.y = (int)ptJointPosProjective[7].Y;
        
                        skeletonUserDetectedVec[iUserDetected].joints.torso.pixelPos.x = (int)ptJointPosProjective[8].X;
                        skeletonUserDetectedVec[iUserDetected].joints.torso.pixelPos.y = (int)ptJointPosProjective[8].Y;
        
                        skeletonUserDetectedVec[iUserDetected].joints.leftHip.pixelPos.x = (int)ptJointPosProjective[9].X;
                        skeletonUserDetectedVec[iUserDetected].joints.leftHip.pixelPos.y = (int)ptJointPosProjective[9].Y;
                        skeletonUserDetectedVec[iUserDetected].joints.leftKnee.pixelPos.x = (int)ptJointPosProjective[10].X;
                        skeletonUserDetectedVec[iUserDetected].joints.leftKnee.pixelPos.y = (int)ptJointPosProjective[10].Y;
                        skeletonUserDetectedVec[iUserDetected].joints.leftFoot.pixelPos.x = (int)ptJointPosProjective[11].X;
                        skeletonUserDetectedVec[iUserDetected].joints.leftFoot.pixelPos.y = (int)ptJointPosProjective[11].Y;
        
                        skeletonUserDetectedVec[iUserDetected].joints.rightHip.pixelPos.x = (int)ptJointPosProjective[12].X;
                        skeletonUserDetectedVec[iUserDetected].joints.rightHip.pixelPos.y = (int)ptJointPosProjective[12].Y;
                        skeletonUserDetectedVec[iUserDetected].joints.rightKnee.pixelPos.x = (int)ptJointPosProjective[13].X;
                        skeletonUserDetectedVec[iUserDetected].joints.rightKnee.pixelPos.y = (int)ptJointPosProjective[13].Y;
                        skeletonUserDetectedVec[iUserDetected].joints.rightFoot.pixelPos.x = (int)ptJointPosProjective[14].X;
                        skeletonUserDetectedVec[iUserDetected].joints.rightFoot.pixelPos.y = (int)ptJointPosProjective[14].Y;
        
                        skeletonUserDetectedVec[iUserDetected].joints.COM.pixelPos.x = (int)ptJointPosProjective[15].X;
                        skeletonUserDetectedVec[iUserDetected].joints.COM.pixelPos.y = (int)ptJointPosProjective[15].Y;
                        }
                }
                else
                {
                }
//check if there is a new user tracked
            }
        }
        else
        {
        }
    }    

    bool SkeletonTrackerVSNI::GetObjectPositionNISkeleton2(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result)
    {
    }

    bool SkeletonTrackerVSNI::GetObjectPositionNISkeleton(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result)
    {
// 15 joints + 1 COM 
//float XnPoint3D
        XnPoint3D ptJointPosReal[16];
        XnPoint3D ptJointPosProjective[16];
        XnSkeletonJointPosition jointPos[15];
        XnSkeletonJointOrientation JointOr[15];

        XnUserID aUsers_[15];
        XnUInt16 nUsers = 15;
        userGen_.GetUsers(aUsers_, nUsers);

        result["userDetectedNbr"] = (int)nUsers;
//        std::cout<<"nUsers "<<(int)nUsers<<std::endl;
        for (int iUserDetected = 0; iUserDetected < nUsers; ++iUserDetected)
        {
            if (userGen_.GetSkeletonCap().IsTracking(aUsers_[iUserDetected]))
            {
//get the translation of the joints
                int isTrack = 1;
                result["user"][iUserDetected]["isTrack"] = (int)isTrack;
                result["user"][iUserDetected]["nId"] = (int)(aUsers_[iUserDetected]) ;
//                std::cout<<"test is track VSNI"<< (bool)result["user"][iUserDetected]["isTrack"]<<std::endl;

                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_HEAD, jointPos[0]);
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_NECK, jointPos[1]);
    
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_LEFT_SHOULDER, jointPos[2]);
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_LEFT_ELBOW, jointPos[3]);
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_LEFT_HAND, jointPos[4]);
    
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_RIGHT_SHOULDER, jointPos[5]);
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_RIGHT_ELBOW, jointPos[6]);
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_RIGHT_HAND, jointPos[7]);
    
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_TORSO, jointPos[8]);
    
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_LEFT_HIP, jointPos[9]);
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_LEFT_KNEE, jointPos[10]);
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_LEFT_FOOT, jointPos[11]);
    
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_RIGHT_HIP, jointPos[12]);
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_RIGHT_KNEE, jointPos[13]);
                userGen_.GetSkeletonCap().GetSkeletonJointPosition(aUsers_[iUserDetected], XN_SKEL_RIGHT_FOOT, jointPos[14]);
            }
            else
            {
                int isTrack = 0;
                result["user"][iUserDetected]["isTrack"] = (int)isTrack;
                result["user"][iUserDetected]["nId"] = (int)(aUsers_[iUserDetected]);
                int test = 1;
                test = (int)result["user"][iUserDetected]["isTrack"];
//                std::cout<<"not tracked and test"<< test<<std::endl;
//                std::cout<<"test is track VSNI"<< (bool)result["user"][iUserDetected]["isTrack"]<<std::endl;
                //we are only interested with the users tracked
                continue;
            }
//get  the orientation from the joints

//check the confidence of the measure for translation
            for(int iBodyPart = 0; iBodyPart < 15; iBodyPart++)
            {
                result["pos"][iUserDetected][iBodyPart]["C"] = (double)jointPos[iBodyPart].fConfidence;

                if (jointPos[iBodyPart].fConfidence > 0.5)
                {
//ptJointPosReal is usefull container because we can use the openNI function for the projection after            
                    ptJointPosReal[iBodyPart] = jointPos[iBodyPart].position;
//ptJointPos return float
                    NISkeletonPosInCamTranslation_[iBodyPart].at<float>(0,0) =  ptJointPosReal[iBodyPart].X;
                    NISkeletonPosInCamTranslation_[iBodyPart].at<float>(1,0) =  ptJointPosReal[iBodyPart].Y;
                    NISkeletonPosInCamTranslation_[iBodyPart].at<float>(2,0) =  ptJointPosReal[iBodyPart].Z;
                    NISkeletonPosInCamTranslation_[iBodyPart].at<float>(3,0) = 1;
//a examiner le calcul effectuei
//regader la doc file:///home/petit/Downloads/OpenNI-Bin-ev-Linux-x86-v1.5.4.0/Documentation/html/conc_coord.html
//converti les donnees en pixel pour x et y z est tou jours en millimetre les origin en x et y son change du centre de fov a upper-left corner of the FOV
                }
                else
                {
                    if(iBodyPart == 0)
                    {
                    }
                } 
            }

            userGen_.GetCoM(aUsers_[iUserDetected], ptJointPosReal[15]);

//compute the joint detected in the waist position

            NISkeletonRefToHeadTemp_.at<float>(3,0) = 0;
            NISkeletonRefToHeadTemp_.at<float>(3,1) = 0;
            NISkeletonRefToHeadTemp_.at<float>(3,2) = 0;
            NISkeletonRefToHeadTemp_.at<float>(3,3) = 1;

            {
                 std::stringstream ss;
                 ss << coshellVision_->ExecuteACommand("dyn2.head");
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

//            usleep(50000);
//
//            {
//                double currentPan = 0;
//                double currentTilt = 0;
//                {
//                    std::stringstream ss;
//                    ss << coshellVision_->ExecuteACommand("featureHeadDes.errorIN");
//                    
//                    char tmp;
//                    while(ss >> tmp)
//                    {
//                        if( tmp == '[')
//                        {
//                            ss >> currentPan;
//                        }
//                        if( tmp == ',')
//                        {
//                            ss >> currentTilt;
//                        }
//                    }
//               }
//
//                std::cout<<"***************Skeleton****************currentPan "<<currentPan<<std::endl;
//                std::cout<<"***************Skeleton****************currentTilt "<<currentTilt<<std::endl;

//                result["hrp2"][iUserDetected][iBodyPart]["C"] = (double)jointPos[iBodyPart].fConfidence;
//                result["hrp2"][iUserDetected][iBodyPart]["C"] = (double)jointPos[iBodyPart].fConfidence;
//                result["hrp2"]["currentPan"] = (double)currentPan;
//                result["hrp2"]["currentTilt"] = (double)currentTilt;
//            }
            for(int iBodyPart = 0; iBodyPart < 15; iBodyPart++ )
             {
//                std::cout<<"iBodyPart "<<iBodyPart<<std::endl;
//                std::cout<<"NISkeletonRefToHeadTemp_"<<iBodyPart<<std::endl;
//                std::cout<<NISkeletonRefToHeadTemp_<<std::endl;
//                std::cout<<"headToCamMatrixXtionRGB_"<<std::endl;
//                std::cout<<headToCamMatrixXtionRGB_<<std::endl;
//                std::cout<<"NISkeletonPosInCamTranslation_"<<std::endl;
//                std::cout<<NISkeletonPosInCamTranslation_[iBodyPart]<<std::endl;
                NISkeletonposInTorsoTranslation_[iBodyPart] = (NISkeletonRefToHeadTemp_*headToCamMatrixXtionRGB_)*NISkeletonPosInCamTranslation_[iBodyPart];
             }

//RealWorld Position in torsoreference

//body parts map
//Head 0
//Neck 1
//LeftShoulder 2
//LeftElbow 3
//LeftHand 4
//RightShoulder 5
//RightElbow 6
//RightHand 7
//Torso 8
//LeftHip 9
//LeftKnee 10
//LeftFoot 11
//RightHip 12
//RightKnee 13
//RightFoot 14
//COM 15
            result["pos"][iUserDetected][0]["T"][0] = (double)NISkeletonposInTorsoTranslation_[0].at<float>(0,0);
            result["pos"][iUserDetected][0]["T"][1] = (double)NISkeletonposInTorsoTranslation_[0].at<float>(1,0);
            result["pos"][iUserDetected][0]["T"][2] = (double)NISkeletonposInTorsoTranslation_[0].at<float>(2,0);

            result["pos"][iUserDetected][1]["T"][0] = (double)NISkeletonposInTorsoTranslation_[1].at<float>(0,0);
            result["pos"][iUserDetected][1]["T"][1] = (double)NISkeletonposInTorsoTranslation_[1].at<float>(1,0);
            result["pos"][iUserDetected][1]["T"][2] = (double)NISkeletonposInTorsoTranslation_[1].at<float>(2,0);

            result["pos"][iUserDetected][2]["T"][0] = (double)NISkeletonposInTorsoTranslation_[2].at<float>(0,0);
            result["pos"][iUserDetected][2]["T"][1] = (double)NISkeletonposInTorsoTranslation_[2].at<float>(1,0);
            result["pos"][iUserDetected][2]["T"][2] = (double)NISkeletonposInTorsoTranslation_[2].at<float>(2,0);

            result["pos"][iUserDetected][3]["T"][0] = (double)NISkeletonposInTorsoTranslation_[3].at<float>(0,0);
            result["pos"][iUserDetected][3]["T"][1] = (double)NISkeletonposInTorsoTranslation_[3].at<float>(1,0);
            result["pos"][iUserDetected][3]["T"][2] = (double)NISkeletonposInTorsoTranslation_[3].at<float>(2,0);

            result["pos"][iUserDetected][4]["T"][0] = (double)NISkeletonposInTorsoTranslation_[4].at<float>(0,0);
            result["pos"][iUserDetected][4]["T"][1] = (double)NISkeletonposInTorsoTranslation_[4].at<float>(1,0);
            result["pos"][iUserDetected][4]["T"][2] = (double)NISkeletonposInTorsoTranslation_[4].at<float>(2,0);

            result["pos"][iUserDetected][5]["T"][0] = (double)NISkeletonposInTorsoTranslation_[5].at<float>(0,0);
            result["pos"][iUserDetected][5]["T"][1] = (double)NISkeletonposInTorsoTranslation_[5].at<float>(1,0);
            result["pos"][iUserDetected][5]["T"][2] = (double)NISkeletonposInTorsoTranslation_[5].at<float>(2,0);

            result["pos"][iUserDetected][6]["T"][0] = (double)NISkeletonposInTorsoTranslation_[6].at<float>(0,0);
            result["pos"][iUserDetected][6]["T"][1] = (double)NISkeletonposInTorsoTranslation_[6].at<float>(1,0);
            result["pos"][iUserDetected][6]["T"][2] = (double)NISkeletonposInTorsoTranslation_[6].at<float>(2,0);

            result["pos"][iUserDetected][7]["T"][0] = (double)NISkeletonposInTorsoTranslation_[7].at<float>(0,0);
            result["pos"][iUserDetected][7]["T"][1] = (double)NISkeletonposInTorsoTranslation_[7].at<float>(1,0);
            result["pos"][iUserDetected][7]["T"][2] = (double)NISkeletonposInTorsoTranslation_[7].at<float>(2,0);

            result["pos"][iUserDetected][8]["T"][0] = (double)NISkeletonposInTorsoTranslation_[8].at<float>(0,0);
            result["pos"][iUserDetected][8]["T"][1] = (double)NISkeletonposInTorsoTranslation_[8].at<float>(1,0);
            result["pos"][iUserDetected][8]["T"][2] = (double)NISkeletonposInTorsoTranslation_[8].at<float>(2,0);

            result["pos"][iUserDetected][9]["T"][0] = (double)NISkeletonposInTorsoTranslation_[9].at<float>(0,0);
            result["pos"][iUserDetected][9]["T"][1] = (double)NISkeletonposInTorsoTranslation_[9].at<float>(1,0);
            result["pos"][iUserDetected][9]["T"][2] = (double)NISkeletonposInTorsoTranslation_[9].at<float>(2,0);

            result["pos"][iUserDetected][10]["T"][0] = (double)NISkeletonposInTorsoTranslation_[10].at<float>(0,0);
            result["pos"][iUserDetected][10]["T"][1] = (double)NISkeletonposInTorsoTranslation_[10].at<float>(1,0);
            result["pos"][iUserDetected][10]["T"][2] = (double)NISkeletonposInTorsoTranslation_[10].at<float>(2,0);

            result["pos"][iUserDetected][11]["T"][0] = (double)NISkeletonposInTorsoTranslation_[11].at<float>(0,0);
            result["pos"][iUserDetected][11]["T"][1] = (double)NISkeletonposInTorsoTranslation_[11].at<float>(1,0);
            result["pos"][iUserDetected][11]["T"][2] = (double)NISkeletonposInTorsoTranslation_[11].at<float>(2,0);


            result["pos"][iUserDetected][12]["T"][0] = (double)NISkeletonposInTorsoTranslation_[12].at<float>(0,0);
            result["pos"][iUserDetected][12]["T"][1] = (double)NISkeletonposInTorsoTranslation_[12].at<float>(1,0);
            result["pos"][iUserDetected][12]["T"][2] = (double)NISkeletonposInTorsoTranslation_[12].at<float>(2,0);

            result["pos"][iUserDetected][13]["T"][0] = (double)NISkeletonposInTorsoTranslation_[13].at<float>(0,0);
            result["pos"][iUserDetected][13]["T"][1] = (double)NISkeletonposInTorsoTranslation_[13].at<float>(1,0);
            result["pos"][iUserDetected][13]["T"][2] = (double)NISkeletonposInTorsoTranslation_[13].at<float>(2,0);

            result["pos"][iUserDetected][14]["T"][0] = (double)NISkeletonposInTorsoTranslation_[14].at<float>(0,0);
            result["pos"][iUserDetected][14]["T"][1] = (double)NISkeletonposInTorsoTranslation_[14].at<float>(1,0);
            result["pos"][iUserDetected][14]["T"][2] = (double)NISkeletonposInTorsoTranslation_[14].at<float>(2,0);

            result["pos"][iUserDetected][15]["T"][0] = (double)NISkeletonposInTorsoTranslation_[15].at<float>(0,0);
            result["pos"][iUserDetected][15]["T"][1] = (double)NISkeletonposInTorsoTranslation_[15].at<float>(1,0);
            result["pos"][iUserDetected][15]["T"][2] = (double)NISkeletonposInTorsoTranslation_[15].at<float>(2,0);

            depth_->ConvertRealWorldToProjective(16, ptJointPosReal, ptJointPosProjective);

//Pixel Position

            result["pos"][iUserDetected][0]["CenterX"] = (int)ptJointPosProjective[0].X;
            result["pos"][iUserDetected][0]["CenterY"] = (int)ptJointPosProjective[0].Y;

            result["pos"][iUserDetected][1]["CenterX"] = (int)ptJointPosProjective[1].X;
            result["pos"][iUserDetected][1]["CenterY"] = (int)ptJointPosProjective[1].Y;

            result["pos"][iUserDetected][2]["CenterX"] = (int)ptJointPosProjective[2].X;
            result["pos"][iUserDetected][2]["CenterY"] = (int)ptJointPosProjective[2].Y;
            result["pos"][iUserDetected][3]["CenterX"] = (int)ptJointPosProjective[3].X;
            result["pos"][iUserDetected][3]["CenterY"] = (int)ptJointPosProjective[3].Y;
            result["pos"][iUserDetected][4]["CenterX"] = (int)ptJointPosProjective[4].X;
            result["pos"][iUserDetected][4]["CenterY"] = (int)ptJointPosProjective[4].Y;

            result["pos"][iUserDetected][5]["CenterX"] = (int)ptJointPosProjective[5].X;
            result["pos"][iUserDetected][5]["CenterY"] = (int)ptJointPosProjective[5].Y;
            result["pos"][iUserDetected][6]["CenterX"] = (int)ptJointPosProjective[6].X;
            result["pos"][iUserDetected][6]["CenterY"] = (int)ptJointPosProjective[6].Y;
            result["pos"][iUserDetected][7]["CenterX"] = (int)ptJointPosProjective[7].X;
            result["pos"][iUserDetected][7]["CenterY"] = (int)ptJointPosProjective[7].Y;

            result["pos"][iUserDetected][8]["CenterX"] = (int)ptJointPosProjective[8].X;
            result["pos"][iUserDetected][8]["CenterY"] = (int)ptJointPosProjective[8].Y;

            result["pos"][iUserDetected][9]["CenterX"] = (int)ptJointPosProjective[9].X;
            result["pos"][iUserDetected][9]["CenterY"] = (int)ptJointPosProjective[9].Y;
            result["pos"][iUserDetected][10]["CenterX"] = (int)ptJointPosProjective[10].X;
            result["pos"][iUserDetected][10]["CenterY"] = (int)ptJointPosProjective[10].Y;
            result["pos"][iUserDetected][11]["CenterX"] = (int)ptJointPosProjective[11].X;
            result["pos"][iUserDetected][11]["CenterY"] = (int)ptJointPosProjective[11].Y;

            result["pos"][iUserDetected][12]["CenterX"] = (int)ptJointPosProjective[12].X;
            result["pos"][iUserDetected][12]["CenterY"] = (int)ptJointPosProjective[12].Y;
            result["pos"][iUserDetected][13]["CenterX"] = (int)ptJointPosProjective[13].X;
            result["pos"][iUserDetected][13]["CenterY"] = (int)ptJointPosProjective[13].Y;
            result["pos"][iUserDetected][14]["CenterX"] = (int)ptJointPosProjective[14].X;
            result["pos"][iUserDetected][14]["CenterY"] = (int)ptJointPosProjective[14].Y;

            result["pos"][iUserDetected][15]["CenterX"] = (int)ptJointPosProjective[15].X;
            result["pos"][iUserDetected][15]["CenterY"] = (int)ptJointPosProjective[15].Y;

//TODO hard copy of the data if we want to speed up the process we van write directly in *result because the pointer has been allocated  in bci-self-interact
          }
    }

//      void NISkeletonProc::GetNISkeletonResult( XmlRpc::XmlRpcValue & resultINOUT)
//      {
//  
//          XnSkeletonJointPosition jointPos[15];
//  
//  //a XnSkeletonJointOrientation has this public attribute:
//  // XnMatrix3X3    orientation
//  // XnConfidence    fConfidence
//  
//  //        XnSkeletonJointOrientation JointOr[15];
//  //16 position : 15 joints position and 1 COM
//  /*
//  00444 typedef struct XnVector3D
//  00445 {
//  00446     XnFloat X;
//  00447     XnFloat Y;
//  00448     XnFloat Z;
//  00449 } XnVector3D;
//  00450 
//  00451 typedef XnVector3D XnPoint3D;
//  */
//  
//          XnPoint3D ptJointPos[16];
//  
//          XnUserID aUsers[15];
//          XnUInt16 nUsers = 15;
//          userSkel_.GetUsers(aUsers, nUsers);
//  
//  //different XnSkeletonJoint
//  /*
//          XN_SKEL_HEAD 
//          XN_SKEL_NECK
//  
//          XN_SKEL_LEFT_SHOULDER
//          XN_SKEL_LEFT_ELBOW
//          XN_SKEL_LEFT_HAND
//  
//          XN_SKEL_RIGHT_SHOULDER
//          XN_SKEL_RIGHT_ELBOW
//          XN_SKEL_RIGHT_HAND
//  
//          XN_SKEL_TORSO
//     
//          XN_SKEL_LEFT_HIP
//          XN_SKEL_LEFT_KNEE
//          XN_SKEL_LEFT_FOOT
//  
//    
//          XN_SKEL_RIGHT_HIP
//          XN_SKEL_RIGHT_KNEE
//          XN_SKEL_RIGHT_FOOT
//  */
//  /*
//      XnPoint3D pt[2];
//      pt[0] = joint1.position;
//      pt[1] = joint2.position;
//  
//  */
//          resultINOUT["NISkeleton"]["userDetectedNbr"] = nUsers;
//  //        std::cout<<"nUsers "<<nUsers<<std::endl;
//  //       int NISkeletonUserDetectedNbrTMP = (int)resultINOUT["NISkeleton"]["userDetectedNbr"];
//  
//  //        std::cout<<"NISkeletonUserDetectedNbrTMP"<<NISkeletonUserDetectedNbrTMP<<std::endl;
//  
//          for (int iUserDetected = 0; iUserDetected < nUsers; ++iUserDetected)
//          {
//              if (!userSkel_.GetSkeletonCap().IsTracking(aUsers[iUserDetected]))
//              {
//  //                printf("not tracked!\n");
//              //    return;
//              }
//  
//  //get the translation of the joints
//  
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_HEAD, jointPos[0]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_NECK, jointPos[1]);
//  
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_SHOULDER, jointPos[2]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_ELBOW, jointPos[3]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_HAND, jointPos[4]);
//  
//  
//  userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_SHOULDER, jointPos[5]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_ELBOW, jointPos[6]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_HAND, jointPos[7]);
//  
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_TORSO, jointPos[8]);
//  
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_HIP, jointPos[9]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_KNEE, jointPos[10]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_LEFT_FOOT, jointPos[11]);
//  
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_HIP, jointPos[12]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_KNEE, jointPos[13]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointPosition(aUsers[iUserDetected], XN_SKEL_RIGHT_FOOT, jointPos[14]);
//  
//  //get  the orientation from the joints
//  //          XnStatus    GetSkeletonJointOrientation (XnUserID user, XnSkeletonJoint eJoint, XnSkeletonJointOrientation &Joint) const 
//  /*
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_HEAD, JointOr[0]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_NECK, JointOr[1]);
//  
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_SHOULDER, JointOr[2]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_ELBOW, JointOr[3]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_HAND, JointOr[4]);
//  
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_SHOULDER, JointOr[5]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_ELBOW, JointOr[6]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_HAND, JointOr[7]);
//  
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_TORSO, JointOr[8]);
//  
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_HIP, JointOr[9]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_KNEE, JointOr[10]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_LEFT_FOOT, JointOr[11]);
//  
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_HIP, JointOr[12]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_KNEE, JointOr[13]);
//              userSkel_.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[iUserDetected], XN_SKEL_RIGHT_FOOT, JointOr[14]);
//  */
//  
//  //check the confidence of the measure for translation
//              for(int iBodyPart = 0; iBodyPart < 15; iBodyPart++)
//              {
//                  if (jointPos[iBodyPart].fConfidence < 0.5)
//                  {
//                  //   return;
//                  }
//  //ptJointPos is usefull container because we can use the openNI function for the projection after            
//                  ptJointPos[iBodyPart] = jointPos[iBodyPart].position;
//  
//                  NISkeletonPosInCamTranslation_[iBodyPart].at<float>(0,0) =  ptJointPos[iBodyPart].X;
//                  NISkeletonPosInCamTranslation_[iBodyPart].at<float>(1,0) =  ptJointPos[iBodyPart].Y;
//                  NISkeletonPosInCamTranslation_[iBodyPart].at<float>(2,0) =  ptJointPos[iBodyPart].Z;
//                  NISkeletonPosInCamTranslation_[iBodyPart].at<float>(3,0) = 1;
//  //a examiner le calcul effectuei
//  //regader la doc file:///home/petit/Downloads/OpenNI-Bin-ev-Linux-x86-v1.5.4.0/Documentation/html/conc_coord.html
//  //converti les donnees en pixel pour x et y z est tou jours en millimetre les origin en x et y son change du centre de fov a upper-left corner of the FOV
//              }
//  
//              userSkel_.GetCoM(aUsers[iUserDetected], ptJointPos[15]);
//  
//  //compute the joint detected in the waist position
//  
//              NISkeletonRefToHeadTemp_.at<float>(3,0) = 0;
//              NISkeletonRefToHeadTemp_.at<float>(3,1) = 0;
//              NISkeletonRefToHeadTemp_.at<float>(3,2) = 0;
//              NISkeletonRefToHeadTemp_.at<float>(3,3) = 1;
//  
//              {
//                   std::stringstream ss;
//                   ss << coshellVision_->ExecuteACommand("dyn2.head");
//                   //std::cout<<"VISION"<<std::endl;
//                   int i = 0;
//                   int j = 0;
//                   double value;
//                   char tmp;
//                   while(ss >> tmp)
//                   {
//                       if( tmp == ';' )
//                       {
//                           ++i;
//                           j = 0;
//                       }
//                       if( tmp == '[' || ( tmp == ',' && j != 4) || tmp == ';' )
//                       {
//                           ss >> value;
//                           NISkeletonRefToHeadTemp_.at<float>(i,j) = value;
//                           ++j;
//                       }
//                       if( tmp == ']' )
//                       {
//                           break;
//                       }
//                   }
//               }
//  
//  
//               for(int iBodyPart = 0; iBodyPart < 15; iBodyPart++ )
//               {
//                   NISkeletonposInTorsoTranslation_[iBodyPart] = (NISkeletonRefToHeadTemp_*headToCamMatrixLeft_)*NISkeletonPosInCamTranslation_[iBodyPart];
//               }
//  //                ARCodeposInTorsoTranslation_[iSize] = (ARCodeRefToHeadTemp_*headToCamMatrixLeft_)*ARCodePosInCamTranslationTemp_;
//  
//  //RealWorld Position in torsoreference
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Head"]["T"][0] = NISkeletonposInTorsoTranslation_[0].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Head"]["T"][1] = NISkeletonposInTorsoTranslation_[0].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Head"]["T"][2] = NISkeletonposInTorsoTranslation_[0].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Neck"]["T"][0] = NISkeletonposInTorsoTranslation_[1].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Neck"]["T"][1] = NISkeletonposInTorsoTranslation_[1].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Neck"]["T"][2] = NISkeletonposInTorsoTranslation_[1].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftShoulder"]["T"][0] = NISkeletonposInTorsoTranslation_[2].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftShoulder"]["T"][1] = NISkeletonposInTorsoTranslation_[2].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftShoulder"]["T"][2] = NISkeletonposInTorsoTranslation_[2].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftElbow"]["T"][0] = NISkeletonposInTorsoTranslation_[3].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftElbow"]["T"][1] = NISkeletonposInTorsoTranslation_[3].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftElbow"]["T"][2] = NISkeletonposInTorsoTranslation_[3].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHand"]["T"][0] = NISkeletonposInTorsoTranslation_[4].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHand"]["T"][1] = NISkeletonposInTorsoTranslation_[4].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHand"]["T"][2] = NISkeletonposInTorsoTranslation_[4].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightShoulder"]["T"][0] = NISkeletonposInTorsoTranslation_[5].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightShoulder"]["T"][1] = NISkeletonposInTorsoTranslation_[5].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightShoulder"]["T"][2] = NISkeletonposInTorsoTranslation_[5].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightElbow"]["T"][0] = NISkeletonposInTorsoTranslation_[6].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightElbow"]["T"][1] = NISkeletonposInTorsoTranslation_[6].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightElbow"]["T"][2] = NISkeletonposInTorsoTranslation_[6].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHand"]["T"][0] = NISkeletonposInTorsoTranslation_[7].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHand"]["T"][1] = NISkeletonposInTorsoTranslation_[7].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHand"]["T"][2] = NISkeletonposInTorsoTranslation_[7].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Torso"]["T"][0] = NISkeletonposInTorsoTranslation_[8].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Torso"]["T"][1] = NISkeletonposInTorsoTranslation_[8].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Torso"]["T"][2] = NISkeletonposInTorsoTranslation_[8].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHip"]["T"][0] = NISkeletonposInTorsoTranslation_[9].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHip"]["T"][1] = NISkeletonposInTorsoTranslation_[9].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHip"]["T"][2] = NISkeletonposInTorsoTranslation_[9].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftKnee"]["T"][0] = NISkeletonposInTorsoTranslation_[10].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftKnee"]["T"][1] = NISkeletonposInTorsoTranslation_[10].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftKnee"]["T"][2] = NISkeletonposInTorsoTranslation_[10].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftFoot"]["T"][0] = NISkeletonposInTorsoTranslation_[11].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftFoot"]["T"][1] = NISkeletonposInTorsoTranslation_[11].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftFoot"]["T"][2] = NISkeletonposInTorsoTranslation_[11].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHip"]["T"][0] = NISkeletonposInTorsoTranslation_[12].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHip"]["T"][1] = NISkeletonposInTorsoTranslation_[12].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHip"]["T"][2] = NISkeletonposInTorsoTranslation_[12].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightKnee"]["T"][0] = NISkeletonposInTorsoTranslation_[13].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightKnee"]["T"][1] = NISkeletonposInTorsoTranslation_[13].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightKnee"]["T"][2] = NISkeletonposInTorsoTranslation_[13].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightFoot"]["T"][0] = NISkeletonposInTorsoTranslation_[14].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightFoot"]["T"][1] = NISkeletonposInTorsoTranslation_[14].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightFoot"]["T"][2] = NISkeletonposInTorsoTranslation_[14].at<float>(2,0);
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["COM"]["T"][0] = NISkeletonposInTorsoTranslation_[15].at<float>(0,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["COM"]["T"][1] = NISkeletonposInTorsoTranslation_[15].at<float>(1,0);
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["COM"]["T"][2] = NISkeletonposInTorsoTranslation_[15].at<float>(2,0);
//  
//              depth_->ConvertRealWorldToProjective(16, ptJointPos, ptJointPos);
//  
//  //Pixel Position
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Head"]["CenterX"] = ptJointPos[0].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Head"]["CenterY"] = ptJointPos[0].Y;
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Neck"]["CenterX"] = ptJointPos[1].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Neck"]["CenterY"] = ptJointPos[1].Y;
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftShoulder"]["CenterX"] = ptJointPos[2].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftShoulder"]["CenterY"] = ptJointPos[2].Y;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftElbow"]["CenterX"] = ptJointPos[3].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftElbow"]["CenterY"] = ptJointPos[3].Y;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHand"]["CenterX"] = ptJointPos[4].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHand"]["CenterY"] = ptJointPos[4].Y;
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightShoulder"]["CenterX"] = ptJointPos[5].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightShoulder"]["CenterY"] = ptJointPos[5].Y;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightElbow"]["CenterX"] = ptJointPos[6].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightElbow"]["CenterY"] = ptJointPos[6].Y;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHand"]["CenterX"] = ptJointPos[7].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHand"]["CenterY"] = ptJointPos[7].Y;
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Torso"]["CenterX"] = ptJointPos[8].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["Torso"]["CenterY"] = ptJointPos[8].Y;
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHip"]["CenterX"] = ptJointPos[9].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftHip"]["CenterY"] = ptJointPos[9].Y;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftKnee"]["CenterX"] = ptJointPos[10].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftKnee"]["CenterY"] = ptJointPos[10].Y;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftFoot"]["CenterX"] = ptJointPos[11].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["LeftFoot"]["CenterY"] = ptJointPos[11].Y;
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHip"]["CenterX"] = ptJointPos[12].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightHip"]["CenterY"] = ptJointPos[12].Y;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightKnee"]["CenterX"] = ptJointPos[13].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightKnee"]["CenterY"] = ptJointPos[13].Y;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightFoot"]["CenterX"] = ptJointPos[14].X;
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["RightFoot"]["CenterY"] = ptJointPos[14].Y;
//  
//              resultINOUT["NISkeleton"]["user"][iUserDetected]["COM"]["CenterX"] = ptJointPos[15].X;
//  
//  //TODO hard copy of the data if we want to speed up the process we van write directly in *resultINOUT because the pointer has been allocated  in bci-self-interact
//              //resultINOUT = resultINOUT;
//  
//          }
//      }
} //end namespace skeletonTrackerVSNI

PLUGIN(skeletonTrackerVSNI::SkeletonTrackerVSNI)
