#include "robot.h"
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
///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{
    std::cout << "ROBOT CALLBACK\n";
    long double tickToMeter = 0.000085292090497737556558;
    long double b = 0.23;


    if (!is_initialized){
        prev_enc_R = robotdata.EncoderRight;
        prev_enc_L = robotdata.EncoderLeft;
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
        std::cout << "ODOM: " << x << " " << y << " " << fi << std::endl;
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
        emit publishPosition(robotdata.EncoderLeft,y,fi);
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

    float angle = phi_r + scan.scanAngle;

    angle = angle * M_PI / 180.0;
    float dist = scan.scanDistance;

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

int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    copyOfLaserData = laserData;

    static float prev_phi = 0.0f;

    float angle_diff = fabs(phi_r - prev_phi);
    if (angle_diff > M_PI)
        angle_diff = 2.0f * M_PI - angle_diff;

    bool isRotating = (angle_diff > 0.005f);

    prev_phi = phi_r;

    int robotX = toCell(x_r);
    int robotY = toCell(y_r);

    const float LIDAR_MAX = 3.0f;

    for (const auto& laser : copyOfLaserData)
    {
        float dist = laser.scanDistance / 1000.0f;

        if (dist < 0.05f)
            continue;

        float angle = phi_r - laser.scanAngle * M_PI / 180.0f;
        Point p;
        p.x = x_r + dist * cos(angle);
        p.y = y_r + dist * sin(angle);

        int x_end = toCell(p.x);
        int y_end = toCell(p.y);

        markLine(robotX, robotY, x_end, y_end, grid);

        if (!isRotating && dist < LIDAR_MAX - 0.005f)
        {
            int x_obs = toCell(p.x);
            int y_obs = toCell(p.y);

            if (x_obs >= 0 && x_obs < GRID_SIZE &&
                y_obs >= 0 && y_obs < GRID_SIZE)
            {
                grid[y_obs][x_obs].occupied = 1;
            }
        }
    }

    if (!isRotating)
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
