#include "robot.h"
#include <deque>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
robot::robot(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
    #ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
#endif
#ifndef DISABLE_SKELETON
qRegisterMetaType<skeleton>("skeleton");
#endif
}

void robot::initAndStartRobot(std::string ipaddress)
{

    forwardspeed=0;
    rotationspeed=0;
    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int{return processThisLidar(dat);},ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int{return processThisRobot(dat);},ipaddress);
  #ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera,this,std::placeholders::_1),"http://"+ipaddress+":8000/stream.mjpg");
#endif
   #ifndef DISABLE_SKELETON
      robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robotCom.robotStart();


}

void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed=forw;
    rotationspeed=rots;
    useDirectCommands=0;
}

void robot::setSpeed(double forw, double rots)
{
    if(forw==0 && rots!=0)
        robotCom.setRotationSpeed(rots);
    else if(forw!=0 && rots==0)
        robotCom.setTranslationSpeed(forw);
    else if((forw!=0 && rots!=0))
        robotCom.setArcSpeed(forw,forw/rots);
    else
        robotCom.setTranslationSpeed(0);
    useDirectCommands=1;
}
double x = 0.0;
double y = 0.0;
double fi = 0.0;
float x_r = 0.0f;
float y_r = 0.0f;
float phi_r = 0.0f;

int prev_enc_R = 0;
int prev_enc_L = 0;

bool is_initialized = false;
struct Pose
{
    double x;
    double y;
    double fi;
    uint32_t t;
};

struct Waypoint {
    double x;
    double y;
};
std::vector<Waypoint> path;
int current_target_idx = 0;
bool path_active = false;
void addWaypoint(double x, double y)
{
    path.push_back({x, y});
    path_active = true;
}

void clearPath()
{
    path.clear();
    current_target_idx = 0;
    path_active = false;
}
bool getCurrentTarget(double &x_ref, double &y_ref)
{
    if (!path_active || current_target_idx >= path.size())
        return false;

    x_ref = path[current_target_idx].x;
    y_ref = path[current_target_idx].y;
    return true;
}

void setPath()
{
    clearPath();
//sim
    addWaypoint(0.0, 3.35);
    addWaypoint(2.7, 4.2);
    addWaypoint(4.2, 3.9);
    addWaypoint(2.8, 4.1);
    addWaypoint(3.1, 0.7);
    addWaypoint(4.9, 0.7);
    addWaypoint(4.9, 1.3);
    addWaypoint(4.9, 0.7);
    addWaypoint(3.1, 0.7);
    addWaypoint(2.9, -1.01);
    addWaypoint(1.5, -1.05);
//real
    // addWaypoint(0.0, 3.35);
    // addWaypoint(1.5, 3.75);
    // addWaypoint(3.24, 3.7);
    // addWaypoint(4.14, 3.15);
    // addWaypoint(3.24, 3.7);

}
std::deque<Pose> poseHistory;
uint32_t lastLaserTimestamp = 0;
// bool isRotating = false;
///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{
    //std::cout <<"ROBOT DATA TIMESTAMP " <<robotdata.timestamp << std::endl;
    long double tickToMeter = 0.000085292090497737556558;
    long double b = 0.23;


    if (!is_initialized){
        prev_enc_R = robotdata.EncoderRight;
        prev_enc_L = robotdata.EncoderLeft;
        setPath();
        is_initialized = true;
    }
    else{
        short ticks_R = (short)(robotdata.EncoderRight - prev_enc_R);
        short ticks_L = (short)(robotdata.EncoderLeft - prev_enc_L);

        double diff_R = ticks_R * tickToMeter;
        double diff_L = ticks_L * tickToMeter;

        double delta_alpha = (diff_R - diff_L) / b;
        double fi_new = fi + delta_alpha;

        if (fabs(diff_R - diff_L) < 1e-6){
            double d_center = (diff_L + diff_R) / 2.0;
            x += d_center * cos(fi);
            y += d_center * sin(fi);
        }
        else{
            double R = (b * (diff_R + diff_L)) / (2.0 * (diff_R - diff_L));
            x += R * (sin(fi_new) - sin(fi));
            y -= R * (cos(fi_new) - cos(fi));
        }

        fi = fi_new;

        while(fi > M_PI) fi -= 2.0 * M_PI;
        while(fi < -M_PI) fi += 2.0 * M_PI;

        prev_enc_R = robotdata.EncoderRight;
        prev_enc_L = robotdata.EncoderLeft;

        x_r = x;
        y_r = y;
        phi_r = fi;

        // std::cout << "POSE: x=" << x
        //           << " y=" << y
        //           << " fi=" << fi
        //           << std::endl;
        std::cout << "robot timestamp:" << robotdata.synctimestamp << std::endl;
        poseHistory.push_back({x, y, fi, robotdata.synctimestamp});


        if (poseHistory.size() > 1000)
            poseHistory.pop_front();

        double x_ref, y_ref;

        if (!getCurrentTarget(x_ref, y_ref))
        {
            forwardspeed = 0;
            rotationspeed = 0;
            return 0;
        }


        double Kp_rot = 0.8;
        double Kp_trans = 200.0;

        double pa1 = 0.1;
        double pa2 = 0.4;
        double dist_tol = 0.5;

        static bool is_rotating = true;

        static double last_rot_speed = 0.0;
        static double last_trans_speed = 0.0;

        double max_accel_rot = 0.01;
        double max_accel_trans = 5.0;


        double dx = x_ref - x;
        double dy = y_ref - y;
        double distance = std::sqrt(dx*dx + dy*dy);


        double fi_ref = atan2(dy, dx);

        double error_fi = fi_ref - fi;
        while(error_fi > M_PI) error_fi -= 2.0 * M_PI;
        while(error_fi < -M_PI) error_fi += 2.0 * M_PI;

        double out_rot = 0.0;
        double out_trans = 0.0;


        if (distance > dist_tol)
        {
            if (is_rotating)
            {
                double target_rot = Kp_rot * error_fi;

                if (target_rot > last_rot_speed + max_accel_rot)
                    out_rot = last_rot_speed + max_accel_rot;
                else if (target_rot < last_rot_speed - max_accel_rot)
                    out_rot = last_rot_speed - max_accel_rot;
                else
                    out_rot = target_rot;

                out_trans = 0.0;
                last_trans_speed = 0.0;

                if (fabs(error_fi) < pa1)
                    is_rotating = false;
            }
            else
            {
                double target_trans = Kp_trans * distance;

                if (target_trans > 400.0)
                    target_trans = 400.0;

                if (target_trans > last_trans_speed + max_accel_trans)
                    out_trans = last_trans_speed + max_accel_trans;
                else
                    out_trans = target_trans;

                if (out_trans < 25) out_trans = 25;

                out_rot = 0.0;
                last_rot_speed = 0.0;

                if (fabs(error_fi) > pa2)
                    is_rotating = true;
            }
        }
        else
        {

            current_target_idx++;

            if (current_target_idx >= path.size())
            {
                path_active = false;
                printf("PATH FINISHED\n");
            }

            out_rot = 0.0;
            out_trans = 0.0;
        }


        last_trans_speed = out_trans;
        last_rot_speed = out_rot;

        forwardspeed = out_trans;
        rotationspeed = out_rot;
        useDirectCommands = 0;


        static uint32_t prev_time = 0;
        static float prev_phi = 0.0f;

        if (prev_time == 0)
        {
            prev_time = robotdata.synctimestamp;
            prev_phi = phi_r;
            return 0;
        }

        float dt = (robotdata.synctimestamp - prev_time) / 1000.0f;

        if (dt < 0.001f) return 0;

        float dphi = phi_r - prev_phi;

        if (dphi > M_PI) dphi -= 2.0f * M_PI;
        if (dphi < -M_PI) dphi += 2.0f * M_PI;

        float angular_velocity = fabs(dphi / dt);
        this->angular_velocity = angular_velocity;
       // bool isRotating = (angular_velocity > 0.2f);
        this->isRotating = (angular_velocity > 0.1f);

        prev_phi = phi_r;
        prev_time = robotdata.synctimestamp;
    }

///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if(datacounter%5==0)
    {

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit publishPosition(x, y, fi);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if(useDirectCommands==0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robotCom.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }
    datacounter++;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
struct Cell {
    int occupied = -1; // -1 unknown, 0 free, 1 occupied
};

const int GRID_SIZE = 140;
const float RESOLUTION = 0.1f;

Cell grid[GRID_SIZE][GRID_SIZE];



void saveMap(Cell grid[][GRID_SIZE])
{
    std::ofstream file("map.txt");

    for (int y = 0; y < GRID_SIZE; y++)
    {
        for (int x = 0; x < GRID_SIZE; x++)
        {
            file << x << " " << y << " " << grid[y][x].occupied << "\n";
        }
    }

    file.close();
}
int toCell(float coord)
{
    return (int)(coord / RESOLUTION + GRID_SIZE / 2);
}

struct Point {
    float x;
    float y;
};


Point toGlobal(
    float x_r, float y_r,
    float phi_r,
    const LaserData& scan)
{
    Point p;

    float angle = phi_r - scan.scanAngle;

    angle = angle * M_PI / 180.0;
    float dist = scan.scanDistance / 1000.0f;

    float maxDist = (GRID_SIZE * RESOLUTION) / 2.0f;

    if (dist > maxDist)
        dist = maxDist;
    p.x = x_r + dist * cos(angle);
    p.y = y_r + dist * sin(angle);

    return p;
}
void markLine(int x0, int y0, int x1, int y1, Cell grid[][GRID_SIZE])
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    int x = x0;
    int y = y0;

    while (true)
    {

        if (x == x1 && y == y1)
            break;

        if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE)
        {
            if (grid[y][x].occupied != 1)
                grid[y][x].occupied = 0;
        }

        int e2 = 2 * err;

        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 <  dx) { err += dx; y += sy; }
    }

    if (x1 >= 0 && x1 < GRID_SIZE && y1 >= 0 && y1 < GRID_SIZE)
    {
        grid[y1][x1].occupied = 1;
    }
    if (x0 == x1 && y0 == y1)
        return;
}
Pose interpolate(uint32_t t)
{
    if (poseHistory.empty())
    {
        return {x, y, fi, t};
    }

    for (size_t i = 1; i < poseHistory.size(); i++)
    {
        auto &p1 = poseHistory[i - 1];
        auto &p2 = poseHistory[i];

        if (p1.t <= t && p2.t >= t)
        {
            uint32_t dt = p2.t - p1.t;

            if (dt == 0)
            {
                return p1;
            }

            double a = double(t - p1.t) / double(dt);

            Pose p;
            p.x = p1.x + a * (p2.x - p1.x);
            p.y = p1.y + a * (p2.y - p1.y);

            double d = p2.fi - p1.fi;
            if (d > M_PI) d -= 2 * M_PI;
            if (d < -M_PI) d += 2 * M_PI;

            p.fi = p1.fi + a * d;
            p.t = t;

            return p;
        }
    }

    return poseHistory.back();
}

int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    copyOfLaserData = laserData;



    // static float prev_phi = 0.0f;

    // float angle_diff = fabs(phi_r - prev_phi);
    // if (angle_diff > M_PI)
    //     angle_diff = 2.0f * M_PI - angle_diff;

    // bool isRotating = (angle_diff > 0.03f);

    // prev_phi = phi_r;



    const float LIDAR_MAX = 3.0f;

    for (const auto& laser : copyOfLaserData)
    {
        if (this->angular_velocity > 0.05f)
            continue;

        Pose p = interpolate(laser.timestamp);
        std::cout << "Laser t: " << laser.timestamp
                  << " | Used pose t: " << p.t
                  << std::endl;

        float dist = laser.scanDistance / 1000.0f;
        float angle = p.fi - laser.scanAngle * M_PI / 180.0f;

        float gx = p.x + dist * cos(angle);
        float gy = p.y + dist * sin(angle);

        int robotX = toCell(p.x);
        int robotY = toCell(p.y);

        int x_end = toCell(gx);
        int y_end = toCell(gy);




        if (this->angular_velocity < 0.05f)
        {


            if (dist > 0.15f && dist < LIDAR_MAX - 0.05f)
            {
                markLine(robotX, robotY, x_end, y_end, grid);
                if (x_end >= 0 && x_end < GRID_SIZE &&
                    y_end >= 0 && y_end < GRID_SIZE)
                {
                    grid[y_end][x_end].occupied = 1;
                }
            }
        }

    }


    static double prev_x = 0.0;
    static double prev_y = 0.0;
    static double prev_fi = 0.0;

    double dx = fabs(x - prev_x);
    double dy = fabs(y - prev_y);
    double dfi = fabs(fi - prev_fi);

    bool isMoving = (dx > 0.001 || dy > 0.001 || dfi > 0.001);

    prev_x = x;
    prev_y = y;
    prev_fi = fi;
    if (!isRotating && !isMoving)
    {
        saveMap(grid);
    }




    emit publishLidar(copyOfLaserData);

    return 0;
}
  #ifndef DISABLE_OPENCV
///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int robot::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

  #ifndef DISABLE_SKELETON
/// vola sa ked dojdu nove data z trackera
int robot::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
