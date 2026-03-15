#include "robot.h"
#include <thread>
#include <chrono>

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

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
double previous_error = 0;
double integral_fi = 0;
double derivative_fi = 0;
int dt = 10;   // 10 ms
double previous_error_fi = 0;
int Kp = 5;
int Ki = 1;

int robot::processThisRobot(const TKobukiData &robotdata)
{

    ///tu mozete robit s datami z robota
    long double tickToMeter = 0.000085292090497737556558;
    long double b = 0.23;

    if (is_inicialized == false){
        prev_enc_R = robotdata.EncoderRight;
        prev_enc_L = robotdata.EncoderLeft;


        is_inicialized = true;
    }
    else{
        short ticks_R = (short)(robotdata.EncoderRight - prev_enc_R);
        short ticks_L = (short)(robotdata.EncoderLeft - prev_enc_L);

        double diff_R = ticks_R * tickToMeter;
        double diff_L = ticks_L * tickToMeter;

        double delta_alpha = (diff_R - diff_L) / b;
        double fi_new = fi + delta_alpha;

        if (std::abs(diff_R - diff_L) < 0.000001){
            double d_center = (diff_L + diff_R) / 2.0;
            x += d_center * cos(fi);
            y += d_center * sin(fi);
        }
        else{
            double zlomok = (b * (diff_R + diff_L)) / (2.0 * (diff_R - diff_L));
            x = x + zlomok * (sin(fi_new) - sin(fi));
            y = y - zlomok * (cos(fi_new) - cos(fi));
        }

        fi = fi_new;

        while(fi > PI) fi -= 2.0 * PI;
        while(fi < -PI) fi += 2.0 * PI;

        prev_enc_R = robotdata.EncoderRight;
        prev_enc_L = robotdata.EncoderLeft;
        printf("x:%f\n", x);
        printf("y:%f\n", y);
        printf("phi: %f\n", fi);

        double x_ref = 0;
        double y_ref = 3;

        double Kp_rot = 0.8;
        double Kp_trans = 200.0;

        double pa1 = 0.1;
        double pa2 = 0.4;
        double dist_tol = 0.05;

        static bool is_rotating = true;

        static double last_rot_speed = 0.0;
        static double last_trans_speed = 0.0;

        double max_accel_rot = 0.01;
        double max_accel_trans = 5.0;

        double dx = x_ref - x;
        double dy = y_ref - y;
        double distance = std::sqrt(dx*dx + dy*dy);
        double fi_ref = std::atan2(dy, dx);
        double dist_perc = x *100 / x_ref;

        double error_fi = fi_ref - fi;
        while(error_fi > PI) error_fi -= 2.0 * PI;
        while(error_fi < -PI) error_fi += 2.0 * PI;

        double out_rot = 0.0;
        double out_trans = 0.0;

        if (distance > dist_tol){
            if (is_rotating){
                double target_rot = Kp_rot * error_fi;
                if (target_rot > last_rot_speed + max_accel_rot){
                    out_rot = last_rot_speed + max_accel_rot;
                }else if (target_rot < last_rot_speed - max_accel_rot){
                    out_rot = last_rot_speed - max_accel_rot;
                }else{
                    out_rot = target_rot;
                }
                out_trans = 0.0;
                last_trans_speed = 0.0;
                if (std::abs(error_fi) < pa1){
                    is_rotating = false;
                }
            }else{
                double target_trans;
                target_trans = Kp_trans * distance;

                if (target_trans > 400.0){
                    target_trans = 400.0;
                }
                if (target_trans > last_trans_speed + max_accel_trans){
                    out_trans = last_trans_speed + max_accel_trans;
                }else if (target_trans < last_trans_speed - max_accel_trans){
                    out_trans = last_trans_speed - max_accel_trans;
                }else {
                    out_trans = target_trans;
                }


                out_trans=out_trans<25?25:out_trans;
                out_rot = 0.0;
                last_rot_speed = 0.0;
                if (std::abs(error_fi) > pa2){
                    is_rotating = true;
                }
            }
        }else{
            out_rot = 0.0;
            out_trans = 0.0;
        }

        last_trans_speed = out_trans;
        last_rot_speed = out_rot;

        forwardspeed = out_trans;
        rotationspeed = out_rot;
        useDirectCommands = 0;
    }


///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if(datacounter%5==0)
    {

        double fi_degrees = fi * (180.0 / PI);
        emit publishPosition(x, y, fi_degrees);

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        //emit publishPosition(robotdata.EncoderLeft,y,fi);
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
int robot::processThisLidar(const std::vector<LaserData>& laserData)
{

    copyOfLaserData=laserData;

    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


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
