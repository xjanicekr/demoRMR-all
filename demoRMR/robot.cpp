#include "robot.h"
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>

double normalizeAngle(double angle){
    return atan2(sin(angle), cos(angle));
}

double saturate(double value, double limit){
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

double normalizeAngleDeg(double angle) {
    while (angle >= 360.0) angle -= 360.0;
    while (angle < 0.0) angle += 360.0;
    return angle;
}

double smallestAngleDiffDeg(double a, double b){
    double d = fabs(normalizeAngleDeg(a - b));
    if (d > 180.0) d = 360.0 - d;
    return d;
}

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
    useDirectCommands = 0;
    datacounter = 0;
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

    //setGoal(x_ref, y_ref);

    std::vector<Waypoint> cesta;

    cesta.push_back({4.0, 4.0}); //1.0, 2.5

    setPath(cesta);
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

void robot::setGoal(double x_goal, double y_goal){
    std::vector<Waypoint> singleGoal;
    singleGoal.push_back({x_goal, y_goal});
    setPath(singleGoal);
}

void robot::setPath(const std::vector<Waypoint>& newPath) {
    path = newPath;
    currentWaypointIndex = 0;

    if (path.empty()) {
        regulatorEnabled = false;
        forwardspeed = 0;
        rotationspeed = 0;
        trans_ramp_speed = 0.0;
        return;
    }

    x_ref = path[0].x;
    y_ref = path[0].y;

    regulatorState = ROTATE_TO_TARGET;
    regulatorEnabled = true;

    forwardspeed = 0;
    rotationspeed = 0;
    trans_ramp_speed = 0.0;

    vfh_valid = false;
    vfh_path_found = false;

    H_b.assign(num_sectors, 0);

    prev_selected_sector = -1;

    useDirectCommands = 0;

    printf("Nova cesta nastavena. Pocet bodov: %zu\n", path.size());
    printf("Aktualny waypoint %d: x=%f y=%f\n", currentWaypointIndex, x_ref, y_ref);
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

        x = 0.0;
        y = 0.0;
        fi = 0.0;

        is_inicialized = true;
    }
    else{
        short ticks_R = (short)(robotdata.EncoderRight - prev_enc_R);
        short ticks_L = (short)(robotdata.EncoderLeft - prev_enc_L);

        double diff_R = ticks_R * tickToMeter;
        double diff_L = ticks_L * tickToMeter;

        double delta_alpha = (diff_R - diff_L) / b;
        double l = (diff_R + diff_L) / 2.0;

        double fi_mid = fi + delta_alpha / 2.0;

        x += l * cos(fi_mid);
        y += l * sin(fi_mid);

        fi += delta_alpha;
        fi = atan2(sin(fi), cos(fi));

        prev_enc_R = robotdata.EncoderRight;
        prev_enc_L = robotdata.EncoderLeft;
    }


    ///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    if (regulatorEnabled) {
        double dx = x_ref - x;
        double dy = y_ref - y;

        double distance_error = sqrt(dx * dx + dy * dy);

        double active_deadband;

        if (!path.empty() && currentWaypointIndex < (int)path.size() - 1) {
            active_deadband = intermediate_deadband;
        }
        else {
            active_deadband = final_deadband;
        }

        if (distance_error < active_deadband) {
            forwardspeed = 0;
            rotationspeed = 0;
            trans_ramp_speed = 0.0;

            if (!path.empty() && currentWaypointIndex < (int)path.size() - 1) {
                currentWaypointIndex++;

                x_ref = path[currentWaypointIndex].x;
                y_ref = path[currentWaypointIndex].y;

                regulatorState = ROTATE_TO_TARGET;

                vfh_valid = false;
                vfh_path_found = false;

                //prev_selected_sector = -1;

                printf("Waypoint dosiahnuty. Prechadzam na waypoint %d: x=%f y=%f\n",
                       currentWaypointIndex,
                       x_ref,
                       y_ref);
            }
            else {
                regulatorState = TARGET_REACHED;
                regulatorEnabled = false;

                printf("Finalny ciel dosiahnuty: x=%f y=%f fi=%f\n", x, y, fi);
            }
        }
        else if (useVFHNavigation && !vfh_valid) {
            forwardspeed = 0;
            rotationspeed = 0;
            trans_ramp_speed = 0.0;
        }
        else if (useVFHNavigation && !vfh_path_found) {
            forwardspeed = 0;
            rotationspeed = 0;
            trans_ramp_speed = 0.0;
        }
        else {
            double desired_angle = atan2(dy, dx); //Global uhol k cielu

            if (useVFHNavigation) {
                desired_angle = vfh_target_angle; //Global uhol z VFH+
            }

            double angle_error = normalizeAngle(desired_angle - fi);

            switch (regulatorState) {
            case ROTATE_TO_TARGET: {
                forwardspeed = 0;
                trans_ramp_speed = 0.0;

                rotationspeed = Kp_rot * angle_error;
                rotationspeed = saturate(rotationspeed, max_rot_speed);

                if (fabs(angle_error) < angle_deadband_1)
                {
                    rotationspeed = 0;
                    regulatorState = MOVE_TO_TARGET;
                }

                break;
            }

            case MOVE_TO_TARGET: {
                rotationspeed = 0;
                if (fabs(angle_error) > angle_deadband_2) {
                    forwardspeed = 0;
                    trans_ramp_speed = 0.0;
                    regulatorState = ROTATE_TO_TARGET;
                }
                else {
                    double desired_forward_speed = Kp_trans * distance_error;

                    if (desired_forward_speed > max_trans_speed)
                        desired_forward_speed = max_trans_speed;

                    if (trans_ramp_speed < desired_forward_speed) {
                        trans_ramp_speed += trans_ramp_step;

                        if (trans_ramp_speed > desired_forward_speed)
                            trans_ramp_speed = desired_forward_speed;
                    }
                    else {
                        trans_ramp_speed = desired_forward_speed;
                    }

                    forwardspeed = trans_ramp_speed;
                }

                break;
            }

            case TARGET_REACHED:
            default: {
                forwardspeed = 0;
                rotationspeed = 0;
                trans_ramp_speed = 0.0;
                break;
            }
            }
        }
    }

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

void robot::calculateVFH(const std::vector<LaserData> &laserData)
{
    struct Valley {
        int start_sector;
        int end_sector;
        int size;
    };

    if ((int)H_b.size() != num_sectors) {
        H_b.assign(num_sectors, 0);
    }

    std::vector<Valley> valleys;
    std::vector<int> candidates;

    double sector_width = 360.0 / num_sectors;

    H_p.assign(num_sectors, 0.0);

    //Aktualne natocenie v globale
    double fi_deg = normalizeAngleDeg(fi * 180.0 / PI);

    //Natocenie na ciel v globale
    double global_target_angle = atan2(y_ref - y, x_ref - x) * 180.0 / PI;
    global_target_angle = normalizeAngleDeg(global_target_angle);

    // lokalny smer na ciel vzhladom na robota
    double local_target_angle = normalizeAngleDeg(global_target_angle - fi_deg);

    int target_sector = (int)(local_target_angle / sector_width) % num_sectors;

    // v lokalnom histograme je aktualny smer robota sektor 0
    int current_fi_sector = 0;

    // Hp
    for (const auto& scan : laserData) {
        double d_i = scan.scanDistance / 1000.0;

        if (d_i <= 0.001) continue;
        if (d_i > laser_max_range) continue;

        double alpha_i_local = normalizeAngleDeg(360.0 - scan.scanAngle);

        double gamma_i;

        if (d_i <= total_radius) {
            gamma_i = 90.0;
        }
        else {
            double ratio = total_radius / d_i;
            if (ratio > 1.0) ratio = 1.0;

            gamma_i = asin(ratio) * 180.0 / PI;
        }

        double m_i = c_i * c_i * (a_i - b_i * d_i);

        if (m_i < 0.0)
            m_i = 0.0;

        for (int k = 0; k < num_sectors; ++k) {
            double sector_center = normalizeAngleDeg((k + 0.5) * sector_width);
            double diff = smallestAngleDiffDeg(sector_center, alpha_i_local);

            if (diff <= gamma_i + sector_width / 2.0) {
                H_p[k] += m_i;
            }
        }
    }

    // 2. Hb 1-obsadeny, 0-volny
    std::vector<int> H_b_new(num_sectors, 0);

    for (int k = 0; k < num_sectors; ++k) {
        if (H_p[k] > tau_high) {
            H_b_new[k] = 1;
        }
        else if (H_p[k] < tau_low) {
            H_b_new[k] = 0;
        }
        else {
            H_b_new[k] = H_b[k];
        }
    }

    H_b = H_b_new;

    double hp_max = 0.0;
    int blocked_count = 0;

    for (int k = 0; k < num_sectors; ++k){
        if (H_p[k] > hp_max)
            hp_max = H_p[k];

        if (H_b[k] == 1)
            blocked_count++;
    }

    printf("VFH debug: Hp_max=%f, blocked=%d/%d, tau_low=%f, tau_high=%f, target_sector=%d, target=%s, Hp_target=%f, local_target=%f, global_target=%f, fi_deg=%f\n",
           hp_max,
           blocked_count,
           num_sectors,
           tau_low,
           tau_high,
           target_sector,
           H_b[target_sector] ? "BLOCKED" : "FREE",
           H_p[target_sector],
           local_target_angle,
           global_target_angle,
           fi_deg);


    auto sectorDiff = [this](int s1, int s2) {
        int d = abs(s1 - s2);
        if (d > num_sectors / 2)
            d = num_sectors - d;
        return d;
    };

    auto addCandidate = [&candidates, this](int c) {
        c = (c % num_sectors + num_sectors) % num_sectors;

        if (std::find(candidates.begin(), candidates.end(), c) == candidates.end())
            candidates.push_back(c);
    };

    bool any_free = false;
    bool any_blocked = false;

    for (int k = 0; k < num_sectors; ++k) {
        if (H_b[k] == 0) any_free = true;
        else any_blocked = true;
    }

    if (!any_free) {
        vfh_valid = true;
        vfh_path_found = false;

        printf("VFH+: ziadny volny sektor, robot stoji.\n");
        return;
    }

    if (!any_blocked) {
        valleys.push_back({0, num_sectors - 1, num_sectors});
    }

    else {
        int start_blocked = -1;
        for (int k = 0; k < num_sectors; ++k) {
            if (H_b[k] == 1) {
                start_blocked = k;
                break;
            }
        }

        bool in_valley = false;
        int v_start = -1;
        int v_size = 0;

        for (int step = 1; step <= num_sectors; ++step) {
            int idx = (start_blocked + step) % num_sectors;
            if (H_b[idx] == 0) {
                if (!in_valley) {
                    in_valley = true;
                    v_start = idx;
                    v_size = 1;
                }
                else {
                    v_size++;
                }
            }
            else {
                if (in_valley) {
                    int v_end = (idx - 1 + num_sectors) % num_sectors;
                    valleys.push_back({v_start, v_end, v_size});

                    in_valley = false;
                    v_start = -1;
                    v_size = 0;
                }
            }
        }
    }

    // 5. VYBER KANDIDATSKYCH SEKTOROV
    for (const auto& v : valleys) {
        if (v.size < min_valley_size) {
            continue;
        }

        if (v.size < s_max) {
            int center = (v.start_sector + v.size / 2) % num_sectors;
            addCandidate(center);
        }
        else {
            int offset = s_max / 2;

            int right_candidate = (v.start_sector + offset) % num_sectors;
            int left_candidate = (v.end_sector - offset + num_sectors) % num_sectors;

            addCandidate(right_candidate);
            addCandidate(left_candidate);
        }
    }

    if (H_b[target_sector] == 0) {
        addCandidate(target_sector);
    }

    if (candidates.empty()) {
        vfh_valid = true;
        vfh_path_found = false;

        printf("VFH+: nenasiel sa kandidat, robot stoji.\n");
        return;
    }

    double min_cost = std::numeric_limits<double>::max();
    int best_sector = candidates[0];

    printf("Candidates debug: target=%d current=%d prev=%d\n",
           target_sector,
           current_fi_sector,
           prev_selected_sector);

    for (int c : candidates) {
        int d_target = sectorDiff(c, target_sector);
        int d_current = sectorDiff(c, current_fi_sector);
        int d_prev = 0;

        if (prev_selected_sector >= 0) {
            d_prev = sectorDiff(c, prev_selected_sector);
        }

        double cost =
            mi1 * d_target +
            mi2 * d_current +
            mi3 * d_prev;

        printf("  c=%d cost=%f d_target=%d d_current=%d d_prev=%d Hb=%d Hp=%f\n",
               c,
               cost,
               d_target,
               d_current,
               d_prev,
               H_b[c],
               H_p[c]);

        if (cost < min_cost) {
            min_cost = cost;
            best_sector = c;
        }
    }

    prev_selected_sector = best_sector;

    double vfh_target_angle_local_deg = normalizeAngleDeg((best_sector + 0.5) * sector_width);

    vfh_target_angle_deg = normalizeAngleDeg(fi_deg + vfh_target_angle_local_deg);
    vfh_target_angle = normalizeAngle(vfh_target_angle_deg * PI / 180.0);

    vfh_valid = true;
    vfh_path_found = true;

    printf("VFH+: cielovy sektor=%d, vybrany sektor=%d, uhol=%f deg, kandidati=%zu\n",
           target_sector,
           best_sector,
           vfh_target_angle_deg,
           candidates.size());

    printf("VFH+ Hm: ");
    for (int k = 0; k < num_sectors; ++k) {
        printf("%d", H_b[k]);
    }
    printf("\n");
}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru

int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    copyOfLaserData=laserData;
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    // updateLaserPicture=1;

    if (useVFHNavigation && regulatorEnabled) {
        calculateVFH(copyOfLaserData);
    }

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
