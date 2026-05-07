#ifndef ROBOT_H
#define ROBOT_H
#include "librobot/librobot.h"
#include <QObject>
#include <QWidget>

#ifndef DISABLE_OPENCV
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

Q_DECLARE_METATYPE(cv::Mat)
#endif
#ifndef DISABLE_SKELETON
Q_DECLARE_METATYPE(skeleton)
#endif
Q_DECLARE_METATYPE(std::vector<LaserData>)
class robot : public QObject {
    Q_OBJECT
public:
    explicit robot(QObject *parent = nullptr);

    void initAndStartRobot(std::string ipaddress);

    // tato funkcia len nastavuje hodnoty.. posielaju sa v callbacku(dobre, kvoli
    // asynchronnosti a zabezpeceniu,ze sa poslu len raz pri viacero prepisoch
    // vramci callu)
    void setSpeedVal(double forw, double rots);
    // tato funkcia fyzicky posiela hodnoty do robota
    void setSpeed(double forw, double rots);

    // Vlastne
    void setGoal(double x_goal, double y_goal);

    struct Waypoint {
        double x;
        double y;
    };

    void setPath(const std::vector<Waypoint>& newPath);

signals:
    void publishPosition(double x, double y, double z);
    void publishLidar(const std::vector<LaserData> &lidata);
#ifndef DISABLE_OPENCV
    void publishCamera(const cv::Mat &camframe);
#endif
#ifndef DISABLE_SKELETON
    void publishSkeleton(const skeleton &skeledata);
#endif
private:
    /// toto su vase premenne na vasu odometriu
    double x = 0;
    double y = 0;
    double fi = 0;

    unsigned short prev_enc_R = 0;
    unsigned short prev_enc_L = 0;
    bool is_inicialized = false;
    ///-----------------------------
    /// toto su rychlosti ktore sa nastavuju setSpeedVal a posielaju v
    /// processThisRobot
    double forwardspeed;  // mm/s
    double rotationspeed; // omega/s

    /// toto su callbacky co sa sa volaju s novymi datami
    int processThisLidar(const std::vector<LaserData> &laserData);
    int processThisRobot(const TKobukiData &robotdata);

    //Vlastne
    enum RegulatorState
    {
        ROTATE_TO_TARGET,
        MOVE_TO_TARGET,
        TARGET_REACHED
    };

    RegulatorState regulatorState = ROTATE_TO_TARGET;

    bool regulatorEnabled = false;

    double x_ref = 0.0;
    double y_ref = 3.0;

    //Regulator
    double Kp_rot = 1.5;
    double max_rot_speed = 0.8;

    double Kp_trans = 400.0;
    double max_trans_speed = 120.0;

    double angle_deadband_1 = 0.0872665;   // 5 stupnov
    double angle_deadband_2 = 0.174533;   // 10 stupnov
    double distance_deadband = 0.15;      // 15 cm

    double trans_ramp_speed = 0.0;
    double trans_ramp_step = 5.0;

    //VFH+
    bool useVFHNavigation = true;

    bool vfh_valid = false;
    bool vfh_path_found = false;

    double vfh_target_angle = 0.0;
    double vfh_target_angle_deg = 0.0;

    int num_sectors = 72;
    int s_max = 8; // hranica medzi uzkym a sirokym priechodom
    int min_valley_size = 3;

    double robot_radius = 0.175;
    double rs = 0.125;
    double total_radius = robot_radius + rs;

    double laser_max_range = 2.0;

    double c_i = 1;
    double a_i = 1;
    double b_i = 0.5;

    double tau_low = 18.0;
    double tau_high = 25.0;

    double mi1 = 12.0;
    double mi2 = 1.0;
    double mi3 = 0.5;

    int prev_selected_sector = 0;

    std::vector<Waypoint> path;
    int currentWaypointIndex = 0;

    std::vector<double> H_p; // Polarny histogram
    std::vector<int> H_b;    // Binarny histogram

    void calculateVFH(const std::vector<LaserData> &laserData);


#ifndef DISABLE_OPENCV
    int processThisCamera(cv::Mat cameraData);
#endif

    /// pomocne strukutry aby ste si trosku nerobili race conditions
    std::vector<LaserData> copyOfLaserData;
#ifndef DISABLE_OPENCV
    cv::Mat frame[3];
#endif
    /// classa ktora riesi komunikaciu s robotom
    libRobot robotCom;

    /// pomocne premenne... moc nerieste naco su
    int datacounter = 0;
#ifndef DISABLE_OPENCV
    bool useCamera1;
    int actIndex;
#endif

#ifndef DISABLE_SKELETON
    int processThisSkeleton(skeleton skeledata);
    int updateSkeletonPicture;
    skeleton skeleJoints;
#endif
    int useDirectCommands = 0;
};

#endif // ROBOT_H
