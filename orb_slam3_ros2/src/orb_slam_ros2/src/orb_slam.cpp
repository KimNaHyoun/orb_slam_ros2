#include "orb_slam.hpp"

using namespace std::chrono_literals;

/**
 *  @brief
 */
OrbSlamRos2::OrbSlamRos2(): Node("orb_slam_ros2"){
    RCLCPP_INFO(this->get_logger(), "Start orb_slam_ros2 ");

    // pub & sub
    str_publisher_ = this->create_publisher<std_msgs::msg::String>("/hello/hl", 10);
    str_subscriber_ = this->create_subscription<std_msgs::msg::String>("/i/heard", 100, std::bind(&OrbSlamRos2::strSubCallback, this, std::placeholders::_1));
    
    // timer
    timer_ = this->create_wall_timer(1000ms, std::bind(&OrbSlamRos2::timerCallback, this));

    orb_slam_th_ = std::thread(&OrbSlamRos2::run, this);   
}

/**
 *  @brief
 */
OrbSlamRos2::~OrbSlamRos2(){

}

void OrbSlamRos2::strSubCallback(std_msgs::msg::String str_msg){
    RCLCPP_INFO(this->get_logger(), "Call strSubCallback()");
}

/**
 *  @brief
 */
void OrbSlamRos2::timerCallback(){
    // publisher test timer
    auto msg = std_msgs::msg::String();
    msg.data = "Hello World!";

    str_publisher_->publish(msg);
}

/**
 *  @brief
 */
void OrbSlamRos2::run(){
   /* Change Image Path to run the code with offline mode */
    std::string strImagePath = "/home/nahyoun/Desktop/Data/MPA/HKL_Parkinglot/B2_SLAM";

    std::vector<std::string> vstrImageFilenames;
    std::vector<double> vTimestamps;
    bool bUseViewer = true;

    loadImages(std::string(strImagePath), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    std::vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    std::cout << std::endl << "-------" << std::endl;
    std::cout << "Start processing sequence ..." << std::endl;
    std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;

    // Main loop
    double t_resize = 0.f;
    double t_track = 0.f;

    cv::Mat im;

    for(auto ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            std::cerr << std::endl << "Failed to load image at: " << vstrImageFilenames[ni] << std::endl;
            return;
        }

        if(imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe,vector<ORB_SLAM3::IMU::Point>(), vstrImageFilenames[ni]);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    std::cout << "-------" << std::endl << std::endl;
    std::cout << "median tracking time: " << vTimesTrack[nImages/2] << std::endl;
    std::cout << "mean tracking time: " << totaltime/nImages << std::endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.txt");    
}

/**
 *  @brief
 */
void OrbSlamRos2::loadImages(const std::string &strPathToSequence, std::vector<std::string> &vstrImageFilenames, std::vector<double> &vTimestamps)
{
    ifstream fTimes;
    std::string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        std::string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    auto strPrefixLeft = strPathToSequence + "/image/RGB/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
