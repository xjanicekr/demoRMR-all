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
        double fi_ref = vfh_target_angle * (PI / 180.0);

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

void robot::calculateVFH(const std::vector<LaserData> &laserData){
    struct Valley {
        int start_sector;
        int end_sector;
        int size;
    };
    std::vector<Valley> valleys;

    int delta = 360 / num_sectors;

    H_p.assign(num_sectors, 0.0);
    H_b.assign(num_sectors, 0);

    double fi_deg = fi * (180 / PI);
    if (fi_deg < 0) fi_deg += 360.0;

    double global_targe_angle = std::atan2(y_ref - y, x_ref - x) * (180.0 / PI);
    if (global_targe_angle < 0) global_targe_angle += 360.0;
    int target_sector = (int)(global_targe_angle / delta) % num_sectors;

    for (const auto& scan : laserData){
        double d_i = scan.scanDistance / 1000.0;

        if (d_i > 3.0) continue;

        double alpha_i_local = 360.0 - scan.scanAngle;

        double alpha_i_global = alpha_i_local + fi_deg;
        while (alpha_i_global >= 360.0) alpha_i_global -= 360.0;
        while (alpha_i_global < 0.0) alpha_i_global += 360.0;

        double gamma_i;
        if (d_i <= total_radius) {
            gamma_i = 90.0;
        } else {
            gamma_i = std::asin(total_radius / d_i) * (180.0 / PI);
        }

        double m_i = pow(c_i, 2) * (a_i - b_i * d_i);

        double a_min = alpha_i_global - gamma_i;
        double a_max = alpha_i_global + gamma_i;

        for (int k = 0; k < num_sectors; ++k) {
            double s_min = k * delta;
            double s_max = (k + 1) * delta;

            if (std::max(s_min, a_min) <= std::min(s_max, a_max) ||
                std::max(s_min, a_min + 360.0) <= std::min(s_max, a_max + 360.0) ||
                std::max(s_min, a_min - 360.0) <= std::min(s_max, a_max - 360.0)) {
                H_p[k] += m_i;
            }
        }
    }

    for (int k = 0; k < num_sectors; ++k){
        if (H_p[k] > tau_high) H_b[k] = 1;
        else if (H_p[k] < tau_low) H_b[k] = 0;
    }

    auto calc_diff = [](int s1, int s2, int n){
        int d = std::abs(s1 - s2);
        return d > n/2 ? n-d : d;
    };

    int current_fi_sector = (int)(fi_deg / delta) % num_sectors;

    int start_idx = -1;
    for(int i = 0; i < num_sectors; i++) {
        if(H_b[i] == 1) {
            start_idx = i;
            break;
        }
    }

    if (start_idx != -1) {
        int current_size = 0;
        int v_start = -1;

        for(int i = 0; i < num_sectors; i++) {
            int idx = (start_idx + i) % num_sectors;

            if (H_b[idx] == 0) {
                if (current_size == 0) v_start = idx;
                current_size++;
            } else {
                if (current_size > 0) {
                    int v_end = (start_idx + i - 1 + num_sectors) % num_sectors;
                    valleys.push_back({v_start, v_end, current_size});
                    current_size = 0;
                }
            }
        }
        if (current_size > 0) {
            int v_end = (start_idx + num_sectors - 1) % num_sectors;
            valleys.push_back({v_start, v_end, current_size});
        }
    }

    std::vector<int> candidates;

    int s_wide = 8;
    int offset = 0;

    if (start_idx == -1) {
        candidates.push_back(target_sector);
    } else {
        for (const auto& v : valleys) {
            if (v.size < s_wide) {
                int center = (v.start_sector + v.size / 2) % num_sectors;
                candidates.push_back(center);
            } else {
                int right_edge = (v.start_sector + offset) % num_sectors;
                int left_edge = (v.start_sector + v.size - 1 - offset + num_sectors) % num_sectors;
                candidates.push_back(right_edge);
                candidates.push_back(left_edge);

                bool target_in_valley = false;
                if (v.start_sector <= v.end_sector) {
                    target_in_valley = (target_sector >= v.start_sector && target_sector <= v.end_sector);
                } else {
                    target_in_valley = (target_sector >= v.start_sector || target_sector <= v.end_sector);
                }

                if (target_in_valley) {
                    candidates.push_back(target_sector);
                }
            }
        }
    }

    double min_cost = std::numeric_limits<double>::max();
    int best_sector = target_sector;
    bool path_found = false;

    for (int c : candidates) {
        path_found = true;

        double cost = mi1 * calc_diff(c, target_sector, num_sectors) +
                      mi2 * calc_diff(c, current_fi_sector, num_sectors) +
                      mi3 * calc_diff(c, prev_selected_sector, num_sectors);

        if (cost < min_cost) {
            min_cost = cost;
            best_sector = c;
        }
    }

    if (path_found) {
        prev_selected_sector = best_sector;
        vfh_target_angle = best_sector * delta;

        printf("Ciel VFH uhol: %f, Cielovy sektor: %d\n", vfh_target_angle, best_sector);
        printf("Pocet najdenych priechodov: %zu, Pocet kandidatov: %zu\n", valleys.size(), candidates.size());
        printf("Histogram: ");
        for(int i=0; i<num_sectors; i++) {
            printf("%d", H_b[i]);
        }
        printf("\n");
    } else {
        printf("Ziadna cesta nebola najdena! Robot stoji.\n");
    }
}


///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru

int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    copyOfLaserData=laserData;
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;

    calculateVFH(copyOfLaserData);

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
