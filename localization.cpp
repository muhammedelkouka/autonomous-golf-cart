// =============================================================================
// localization.cpp  —  LiDAR, IMU, EKF, scan matching, relocalization
// =============================================================================
#include "common.hpp"

// =============================================================================
//  GLOBALS (owned here)
// =============================================================================
EkfState         ekf;
std::mutex       ekf_mutex;

std::vector<Point3D>  dense_cloud;
std::mutex            lidar_mutex;
std::atomic<double>   g_lidar_sweep_time{0.0};
std::atomic<int>      lidar_sweeps_total{0};
std::atomic<int>      imu_frames_total{0};

std::mutex  imu_mutex;
Vector3     imu_acc={}, imu_gyro={}, imu_angle={};
float       g_imu_yaw_at_localize = 0.f;
float       g_yaw_imu_lidar_offset_rad = 0.f;
CalibData   calib_state;
Vector3     g_to_subtract={}, gyro_bias_init={};

RelocResult           g_reloc={};
bool                  g_reloc_done=false, g_reloc_running=false;
float                 g_reloc_progress=0.f;
std::vector<Particle> g_reloc_particles;
std::mutex            g_reloc_mutex;
std::string           g_reloc_error_msg = "";

std::mutex            matcher_mutex;
std::vector<Point3D>  matcher_cloud;
std::atomic<bool>     matcher_request{false};

std::vector<Vector3>         robot_trajectory;
std::vector<TrajectoryPoint> trajectory_data;
std::mutex                   trajectory_mutex;
std::atomic<float>  current_rmse{0.f};
std::atomic<float>  current_inlier_ratio{0.f};
std::atomic<bool>   g_is_turning{false};
std::atomic<bool>   g_is_stationary{false};

std::vector<MapEntry> map_library = {
    {"Lab Map",           "mapwithme.ply",                      {},{},false,false,0.30f,0.65f,0.40f},
    // {"Master Structural", "Master_Structural_Ref_Adaptive.ply", {},{},false,false,0.30f,0.60f,0.35f},  // hidden 2026-06-25
    {"Full Campus",       "Full_Campus_Map_Master.ply",         {},{},false,true, 0.40f,0.55f,0.30f},
    // {"Campus Z-Filtered", "Full_Campus_Map_ZFiltered.ply",      {},{},false,true, 0.40f,0.55f,0.30f},  // hidden 2026-06-25
    {"Campus Top LiDAR",  "Campus_TopLidar_Forward.ply",        {},{},false,true, 0.40f,0.55f,0.30f},
    {"Campus Top (Rev)",  "Campus_TopLidar_Reverse.ply",        {},{},false,true, 0.40f,0.55f,0.30f},
    // Standalone test map (2026-07-07): dense there-and-back recording of just the
    // danger-zone stretch (Edge+Planer+Intensity+Trajectory VTK exports merged into
    // one PLY, Z-cropped), to check whether denser coverage fixes scan-matching in
    // that stretch. Two entries share the SAME cloud file but load opposite driving
    // directions of the recorded trajectory (see pp_active_map_recorded_path). Not
    // wired into corner-mode/zone/cross-zone routing — plain recorded-path following.
    {"Danger Zone Fwd",   "DangerZone_Test.ply",                {},{},false,true, 0.40f,0.55f,0.30f},
    {"Danger Zone Rev",   "DangerZone_Test.ply",                {},{},false,true, 0.40f,0.55f,0.30f},
};
int       active_map_index = 1;  // Full Campus (main.cpp re-selects by name at startup)
MapEntry* active_map_ptr   = nullptr;

const float v_angles[] = {-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15};

// =============================================================================
//  EKF HELPERS
// =============================================================================
void fill8(double* M, const double d[8]) {
    memset(M, 0, 64*sizeof(double));
    for(int i = 0; i < 8; i++) M[i*8+i] = d[i];
}
void fill2(double* M, double d0, double d1) {
    memset(M, 0, 4*sizeof(double)); M[0]=d0; M[3]=d1;
}
void rot_imu(const Vector3& in, Vector3& out) { out = in; }

// =============================================================================
//  TRAJECTORY HELPERS
// =============================================================================
static bool save_trajectory_csv(const std::string& fn) {
    std::lock_guard<std::mutex> lk(trajectory_mutex);
    if(trajectory_data.empty()) return false;
    std::ofstream f(fn); if(!f.is_open()) return false;
    f << "x,y,z,yaw,rmse,inlier_ratio,timestamp,is_turning\n" << std::fixed << std::setprecision(6);
    for(const auto& p : trajectory_data)
        f << p.x << "," << p.y << "," << p.z << "," << p.yaw << "," << p.rmse << ","
          << p.inlier_ratio << "," << p.timestamp << "," << (p.is_turning?1:0) << "\n";
    return true;
}
static void traj_auto_save() {
    static int last = 0;
    int n; { std::lock_guard<std::mutex> lk(trajectory_mutex); n = (int)trajectory_data.size(); }
    if(n > 0 && (n-last) >= TRAJ_AUTO_SAVE_INTERVAL) { save_trajectory_csv(TRAJ_CSV_FILE); last = n; }
}

// =============================================================================
//  DISTANCE MAP BUILDER
// =============================================================================
DistanceMap build_distance_map(const std::vector<MapPoint>& map, bool large) {
    DistanceMap dm; if(map.empty()) return dm;
    dm.resolution = large ? 0.20f : 0.10f;
    dm.min_x = dm.min_y = 1e9f; dm.max_x = dm.max_y = -1e9f;
    for(const auto& p : map) {
        if(p.x<dm.min_x) dm.min_x=p.x; if(p.x>dm.max_x) dm.max_x=p.x;
        if(p.y<dm.min_y) dm.min_y=p.y; if(p.y>dm.max_y) dm.max_y=p.y;
    }
    dm.min_x-=3; dm.max_x+=3; dm.min_y-=3; dm.max_y+=3;
    dm.width  = (int)((dm.max_x-dm.min_x)/dm.resolution)+1;
    dm.height = (int)((dm.max_y-dm.min_y)/dm.resolution)+1;
    dm.grid.assign(dm.width*dm.height, 2.f);
    struct Node { int x,y; float dist; };
    std::queue<Node> q;
    for(const auto& p : map) {
        int cx=(int)((p.x-dm.min_x)/dm.resolution);
        int cy=(int)((p.y-dm.min_y)/dm.resolution);
        if(cx>=0&&cx<dm.width&&cy>=0&&cy<dm.height) {
            int idx=cy*dm.width+cx;
            if(dm.grid[idx]>0){dm.grid[idx]=0;q.push({cx,cy,0});}
        }
    }
    const int   dx8[8]={-1,1,0,0,-1,-1,1,1};
    const int   dy8[8]={0,0,-1,1,-1,1,-1,1};
    const float dd8[8]={dm.resolution,dm.resolution,dm.resolution,dm.resolution,
                        dm.resolution*1.414f,dm.resolution*1.414f,
                        dm.resolution*1.414f,dm.resolution*1.414f};
    while(!q.empty()){
        auto c=q.front();q.pop();
        for(int i=0;i<8;i++){
            int nx=c.x+dx8[i],ny=c.y+dy8[i];
            if(nx>=0&&nx<dm.width&&ny>=0&&ny<dm.height){
                float nd=c.dist+dd8[i];
                int ni=ny*dm.width+nx;
                if(nd<2.f&&nd<dm.grid[ni]){dm.grid[ni]=nd;q.push({nx,ny,nd});}
            }
        }
    }
    return dm;
}

// =============================================================================
//  PLY LOADER
// =============================================================================
bool load_ply(const std::string& path, std::vector<MapPoint>& out) {
    out.clear();
    std::ifstream f(path,std::ios::binary); if(!f.is_open()) return false;
    std::string line; int vcnt=0,xi=-1,yi=-1,zi=-1,pc=0;
    bool bin_le=false,hd=false,in_vertex=false; std::vector<int> psz;
    while(std::getline(f,line)){
        if(!line.empty()&&line.back()=='\r') line.pop_back();
        if(line=="end_header"){hd=true;break;}
        if(line.rfind("format",0)==0&&line.find("binary_little_endian")!=std::string::npos) bin_le=true;
        else if(line.rfind("element vertex",0)==0){std::istringstream ss(line);std::string a,b;ss>>a>>b>>vcnt;in_vertex=true;}
        else if(line.rfind("element",0)==0){in_vertex=false;}  // other element (e.g. face) — its properties must NOT count toward vertex stride
        else if(in_vertex&&line.rfind("property",0)==0){
            std::istringstream ss(line);std::string kw,tp,nm;ss>>kw>>tp>>nm;
            int sz=(tp=="double"||tp=="float64")?8:4;
            if(nm=="x")xi=pc;if(nm=="y")yi=pc;if(nm=="z")zi=pc;
            psz.push_back(sz);pc++;
        }
    }
    if(!hd||vcnt==0||xi<0) return false;
    out.reserve(vcnt);int stride=0;std::vector<int> off(pc);
    for(int i=0;i<pc;i++){off[i]=stride;stride+=psz[i];}
    auto rf=[&](const char*row,int idx)->float{
        const char*ptr=row+off[idx];
        if(psz[idx]==8){double d;memcpy(&d,ptr,8);return(float)d;}
        float v;memcpy(&v,ptr,4);return v;
    };
    if(bin_le){
        std::vector<char> row(stride);
        for(int i=0;i<vcnt;i++){
            f.read(row.data(),stride);if(!f)break;
            MapPoint mp;mp.x=rf(row.data(),xi);mp.y=rf(row.data(),yi);mp.z=rf(row.data(),zi);
            out.push_back(mp);
        }
    } else {
        for(int i=0;i<vcnt;i++){
            if(!std::getline(f,line))break;
            if(!line.empty()&&line.back()=='\r')line.pop_back();
            std::istringstream ss(line);std::vector<double>vals(pc);
            for(int k=0;k<pc;k++)ss>>vals[k];
            MapPoint mp;mp.x=(float)vals[xi];mp.y=(float)vals[yi];mp.z=(float)vals[zi];out.push_back(mp);
        }
    }
    return !out.empty();
}

// =============================================================================
//  ROI / SCORING
// =============================================================================
static bool roi_accept(float lx, float ly, bool large) {
    float r2=lx*lx+ly*ly;
    if(large){
        if(g_is_turning.load()) return r2<(ROI_CAMPUS_SIDE*ROI_CAMPUS_SIDE);
        float az=atan2f(lx,ly);
        float cap=(fabsf(az)<ROI_FWD_HALF_ANG)?ROI_CAMPUS_FWD:ROI_CAMPUS_SIDE;
        return r2<(cap*cap);
    }
    return r2<(ROI_INDOOR_MAX*ROI_INDOOR_MAX);
}
static int match_stride(bool large) { return large?3:5; }
static float score_pose(float px, float py, float pyaw,
                        const std::vector<Point3D>& scan,
                        const DistanceMap& dmap, bool large, int stride) {
    float c=cosf(pyaw),s=sinf(pyaw),sum=0; int n=0;
    for(size_t i=0;i<scan.size();i+=stride){
        if(!roi_accept(scan[i].x,scan[i].y,large)) continue;
        float wx=c*scan[i].x-s*scan[i].y+px;
        float wy=s*scan[i].x+c*scan[i].y+py;
        sum+=dmap.get_dist(wx,wy);n++;
    }
    return n>=(MIN_ABS_INLIERS/2)?sum/n:2.f;
}

// =============================================================================
//  PARTICLE FILTER RELOCALISATION THREAD
// =============================================================================
void particle_filter_reloc_thread(std::vector<Point3D> scan,
    const DistanceMap* dmap, bool large, float inlier_dist,
    bool use_seed, float seed_x, float seed_y, float seed_radius)
{
    if(!dmap||dmap->grid.empty()||scan.empty()){
        std::lock_guard<std::mutex>lk(g_reloc_mutex);
        g_reloc_done=true;g_reloc_running=false;return;
    }
    const int N=use_seed?10000:20000, ITERS=use_seed?25:40;
    const float sigma2=large?0.06f:0.10f;
    const int stride=match_stride(large);
    std::vector<Particle> particles(N);
    std::random_device rd;std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis_yaw(-kPI,kPI);
    std::uniform_real_distribution<float> dis_x(dmap->min_x,dmap->max_x);
    std::uniform_real_distribution<float> dis_y(dmap->min_y,dmap->max_y);
    if(use_seed){
        std::uniform_real_distribution<float> dr(0,1),dth(0,2*kPI);
        for(int i=0;i<N;i++){
            float r=seed_radius*sqrtf(dr(gen)),th=dth(gen);
            particles[i]={seed_x+r*cosf(th),seed_y+r*sinf(th),dis_yaw(gen),1.f/N};
        }
    } else {
        for(int i=0;i<N;i++) particles[i]={dis_x(gen),dis_y(gen),dis_yaw(gen),1.f/N};
    }
    float noise_t=use_seed?1.5f:2.5f,noise_r=0.6f;
    for(int iter=0;iter<ITERS;iter++){
        float sw=0;
        // Parallel particle scoring — each particle is independent; score_pose is
        // read-only over scan/dmap. This is the dominant reloc cost (N×ITERS scan
        // matches); spreading it across cores cuts wall-clock time near-linearly.
        #pragma omp parallel for reduction(+:sw) schedule(static)
        for(int pi=0; pi<N; pi++){
            float avg=score_pose(particles[pi].x,particles[pi].y,particles[pi].yaw,scan,*dmap,large,stride);
            particles[pi].weight=expf(-(avg*avg)/sigma2);
            sw+=particles[pi].weight;
        }
        if(sw<1e-20f){for(auto& p:particles)p.weight=1.f/N;}
        else{for(auto& p:particles)p.weight/=sw;}
        {std::lock_guard<std::mutex>lk(g_reloc_mutex);g_reloc_progress=(float)(iter+1)/ITERS;g_reloc_particles=particles;}
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::vector<Particle> np(N);
        std::uniform_real_distribution<float> dr2(0,1);
        float step=1.f/N,r2=dr2(gen)*step,cw=particles[0].weight;int idx=0;
        for(int j=0;j<N;j++){
            if(dr2(gen)<0.02f){
                if(use_seed){float rr=seed_radius*sqrtf(dr2(gen)),th=dr2(gen)*2*kPI;np[j]={seed_x+rr*cosf(th),seed_y+rr*sinf(th),dis_yaw(gen),1.f/N};}
                else{np[j]={dis_x(gen),dis_y(gen),dis_yaw(gen),1.f/N};}
                continue;
            }
            float U=r2+j*step;
            while(U>cw&&idx<N-1){idx++;cw+=particles[idx].weight;}
            np[j]=particles[idx];
        }
        particles=np;
        std::normal_distribution<float> nt(0,noise_t),nr(0,noise_r);
        for(auto& p:particles){
            p.x+=nt(gen);p.y+=nt(gen);p.yaw+=nr(gen);
            if(use_seed){float ddx=p.x-seed_x,ddy=p.y-seed_y,dist=sqrtf(ddx*ddx+ddy*ddy);if(dist>seed_radius){p.x=seed_x+(ddx/dist)*seed_radius;p.y=seed_y+(ddy/dist)*seed_radius;}}
            while(p.yaw>kPI)p.yaw-=2*kPI;while(p.yaw<-kPI)p.yaw+=2*kPI;
        }
        noise_t*=0.75f;noise_r*=0.75f;
    }
    auto best=std::max_element(particles.begin(),particles.end(),[](const Particle&a,const Particle&b){return a.weight<b.weight;});
    float bx=best->x,by=best->y,byaw=best->yaw;
    float best_sc=score_pose(bx,by,byaw,scan,*dmap,large,stride);
    const float TS=0.15f,TR=3.0f,RS=0.025f,RR=0.28f;
    float fx=bx,fy=by,fyaw=byaw;
    for(float ddx=-TR;ddx<=TR;ddx+=TS)for(float ddy=-TR;ddy<=TR;ddy+=TS){
        float tx=bx+ddx,ty=by+ddy;
        if(use_seed){float dx=tx-seed_x,dy=ty-seed_y;if(dx*dx+dy*dy>seed_radius*seed_radius)continue;}
        for(float ddr=-RR;ddr<=RR;ddr+=RS){float sc=score_pose(tx,ty,byaw+ddr,scan,*dmap,large,stride);if(sc<best_sc){best_sc=sc;fx=tx;fy=ty;fyaw=byaw+ddr;}}
    }
    bx=fx;by=fy;byaw=fyaw;
    while(byaw>kPI)byaw-=2*kPI;while(byaw<-kPI)byaw+=2*kPI;
    int matched=0,total=0;float cy=cosf(byaw),sy=sinf(byaw);
    for(size_t i=0;i<scan.size();i+=stride){
        if(!roi_accept(scan[i].x,scan[i].y,large))continue;
        float wx=cy*scan[i].x-sy*scan[i].y+bx,wy=sy*scan[i].x+cy*scan[i].y+by;
        if(dmap->get_dist(wx,wy)<=inlier_dist)matched++;total++;
    }
    float final_score=(total>=MIN_ABS_INLIERS&&matched>=MIN_ABS_INLIERS)?(float)matched/total:0.f;
    std::lock_guard<std::mutex>lk(g_reloc_mutex);
    g_reloc={bx,by,byaw,final_score,matched};g_reloc_done=true;g_reloc_running=false;
}

// =============================================================================
//  IMU RECEIVER THREAD
// =============================================================================
void imu_receiver_thread() {
    HANDLE hComm=CreateFileA(IMU_PORT,GENERIC_READ,0,nullptr,OPEN_EXISTING,0,nullptr);
    if(hComm==INVALID_HANDLE_VALUE){std::cerr<<"[IMU] Cannot open COM port\n";return;}
    DCB dcb={sizeof(dcb)};GetCommState(hComm,&dcb);
    dcb.BaudRate=IMU_BAUD;dcb.ByteSize=8;dcb.StopBits=ONESTOPBIT;dcb.Parity=NOPARITY;SetCommState(hComm,&dcb);
    COMMTIMEOUTS to={};to.ReadIntervalTimeout=MAXDWORD;SetCommTimeouts(hComm,&to);
    uint8_t packet[11],byte;int pkt_idx=0;DWORD br;
    Vector3 local_acc={},local_gyro={};bool have_acc=false,have_gyro=false;
    LARGE_INTEGER freq,t_prev,t_now;QueryPerformanceFrequency(&freq);QueryPerformanceCounter(&t_prev);
    double gz_lp=0.0;
    while(true){
        if(!ReadFile(hComm,&byte,1,&br,nullptr)||br==0)continue;
        if(pkt_idx==0&&byte!=0x55)continue;
        packet[pkt_idx++]=byte;if(pkt_idx<11)continue;pkt_idx=0;
        uint8_t sum=0;for(int i=0;i<10;i++)sum+=packet[i];if(sum!=packet[10])continue;
        Vector3 raw={};
        if(packet[1]==0x51){
            raw.x=(int16_t)((packet[3]<<8)|packet[2])/32768.f*16.f*9.81f;
            raw.y=(int16_t)((packet[5]<<8)|packet[4])/32768.f*16.f*9.81f;
            raw.z=(int16_t)((packet[7]<<8)|packet[6])/32768.f*16.f*9.81f;
            rot_imu(raw,local_acc);
            {std::lock_guard<std::mutex>lk(imu_mutex);imu_acc=local_acc;
             if(calib_state.is_calibrating){calib_state.sum_acc[0]+=local_acc.x;calib_state.sum_acc[1]+=local_acc.y;calib_state.sum_acc[2]+=local_acc.z;}}
            have_acc=true;
        } else if(packet[1]==0x52){
            raw.x=(int16_t)((packet[3]<<8)|packet[2])/32768.f*2000.f*(kPI/180.f);
            raw.y=(int16_t)((packet[5]<<8)|packet[4])/32768.f*2000.f*(kPI/180.f);
            raw.z=(int16_t)((packet[7]<<8)|packet[6])/32768.f*2000.f*(kPI/180.f);
            rot_imu(raw,local_gyro);local_gyro.z*=IMU_GYRO_SIGN;
            {std::lock_guard<std::mutex>lk(imu_mutex);imu_gyro=local_gyro;
             if(calib_state.is_calibrating){calib_state.sum_gyro[0]+=local_gyro.x;calib_state.sum_gyro[1]+=local_gyro.y;calib_state.sum_gyro[2]+=local_gyro.z;}}
            have_gyro=true;
            gz_lp=0.7*gz_lp+0.3*(double)fabsf(local_gyro.z);
            g_is_turning.store(gz_lp>TURN_GYRO_THRESH);
        } else if(packet[1]==0x53){
            raw.x=(int16_t)((packet[3]<<8)|packet[2])/32768.f*180.f;
            raw.y=(int16_t)((packet[5]<<8)|packet[4])/32768.f*180.f;
            raw.z=(int16_t)((packet[7]<<8)|packet[6])/32768.f*180.f;
            Vector3 ar;rot_imu(raw,ar);ar.z=(ar.z+IMU_YAW_OFFSET)*IMU_YAW_SIGN;
            while(ar.z>180)ar.z-=360;while(ar.z<-180)ar.z+=360;
            {std::lock_guard<std::mutex>lk(imu_mutex);imu_angle=ar;if(calib_state.is_calibrating)calib_state.samples++;}
            imu_frames_total++;
            if(!have_acc||!have_gyro||calib_state.is_calibrating)continue;
            QueryPerformanceCounter(&t_now);
            double dt=(double)(t_now.QuadPart-t_prev.QuadPart)/(double)freq.QuadPart;t_prev=t_now;
            if(dt<=0.||dt>0.05)dt=0.01;
            double acc_c[3]={(double)(local_acc.x-g_to_subtract.x),(double)(local_acc.y-g_to_subtract.y),(double)(local_acc.z-g_to_subtract.z)};
            double gz=(double)(local_gyro.z-gyro_bias_init.z-CALIB_GYRO_BIAS_Z);
            {std::lock_guard<std::mutex>lk(ekf_mutex);
             double xn[8]={},Pn[64]={};
             ekf_fusion_codegen(ekf.x,ekf.P,acc_c,gz,dt,ekf.Q,GYRO_RATE_LP,xn,Pn);
             memcpy(ekf.x,xn,sizeof(xn));memcpy(ekf.P,Pn,sizeof(Pn));
             double ahx=acc_c[0]-ekf.x[6],ahy=acc_c[1]-ekf.x[7];
             bool zupt_cond=(sqrt(ahx*ahx+ahy*ahy)<ZUPT_ACC_THRESH&&fabs(gz)<ZUPT_GYRO_THRESH);
             if(zupt_cond){
                 if(++ekf.zupt_counter>=ZUPT_PERSIST){ekf.x[3]*=0.2;ekf.x[4]*=0.2;ekf.P[3*8+3]*=0.1;ekf.P[4*8+4]*=0.1;ekf.zupt_counter=0;}
             } else ekf.zupt_counter=0;
             g_is_stationary.store(zupt_cond);
            }
        }
    }
    CloseHandle(hComm);
}

// =============================================================================
//  LIDAR RECEIVER THREAD
// =============================================================================
void lidar_receiver_thread() {
    WSADATA wsa;WSAStartup(MAKEWORD(2,2),&wsa);
    SOCKET s=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    sockaddr_in addr={};addr.sin_family=AF_INET;addr.sin_port=htons(VLP16_PORT);addr.sin_addr.s_addr=INADDR_ANY;
    bind(s,(struct sockaddr*)&addr,sizeof(addr));
    uint8_t packet[1206];std::vector<Point3D> frame;frame.reserve(35000);float last_az=-1;
    while(true){
        if(recv(s,(char*)packet,1206,0)!=1206)continue;
        for(int i=0;i<12;i++){
            int off=i*100;float az=(packet[off+2]|(packet[off+3]<<8))/100.f;
            if(last_az!=-1&&az<last_az-100){
                std::lock_guard<std::mutex>lk(lidar_mutex);
                dense_cloud=frame;frame.clear();lidar_sweeps_total++;
                g_lidar_sweep_time.store(nowSecEpoch());
            }
            last_az=az;float az_step=0;
            for(int nb=i+1;nb<12;nb++){float naz=(packet[nb*100+2]|(packet[nb*100+3]<<8))/100.f;if(naz!=az){if(naz<az)naz+=360;az_step=(naz-az)/2;break;}}
            if(az_step==0)az_step=0.2f;
            for(int j=0;j<32;j++){
                int doff=off+4+j*3;uint16_t dr=packet[doff]|(packet[doff+1]<<8);float d=dr*0.002f;
                if(d>MIN_DIST){
                    float vr=v_angles[j%16]*(kPI/180),taz=az+(j>=16?az_step:0);
                    if(taz>=360)taz-=360;float azr=taz*(kPI/180);
                    Point3D p;p.x=d*cosf(vr)*sinf(azr);p.y=d*cosf(vr)*cosf(azr);p.z=d*sinf(vr);
                    p.intensity=packet[doff+2];
                    if(p.z<SCAN_Z_MIN||p.z>SCAN_Z_MAX)continue;
                    if(p.x*p.x+p.y*p.y<3600)frame.push_back(p);
                }
            }
        }
    }
    closesocket(s);WSACleanup();
}

// =============================================================================
//  SCAN MATCHING THREAD
// =============================================================================
void scan_matching_thread() {
    while(true){
        if(matcher_request.load()&&active_map_ptr&&active_map_ptr->loaded){
            std::vector<Point3D> snap;double px,py,pyaw;
            bool large=active_map_ptr->is_large;int mstride=match_stride(large);
            float inlier_dist=active_map_ptr->inlier_dist;
            float min_track=active_map_ptr->track_accept;
            {std::lock_guard<std::mutex>lk(matcher_mutex);snap=matcher_cloud;}
            {std::lock_guard<std::mutex>lk(ekf_mutex);px=ekf.x[0];py=ekf.x[1];pyaw=ekf.x[2];}
            bool turning=g_is_turning.load();
            float yaw_range=turning?0.60f:0.30f,yaw_step_c=turning?0.08f:0.10f,yaw_step_f=turning?0.03f:0.02f;
            // 2026-07-07 danger-zone field test: 1.0m still lost lock ("stuck" then
            // drifted away) at normal driving speed on the new dense map — same
            // drift-loss-spiral class of failure that 0.6m->1.0m fixed before, just
            // needing more headroom for this stretch. Widened to 1.5m; search_step
            // bumped 0.25->0.30 for large maps to keep the per-tick combinatorial cost
            // from growing as much (was confirmed NOT a throughput problem — LiDAR Hz
            // stayed steady during the failure — so there's compute headroom for this).
            float search_range=1.5f,search_step=large?0.30f:0.20f;
            float best_rmse=1e9f,bx=(float)px,by=(float)py,byaw=(float)pyaw;
            for(float ddx=-search_range;ddx<=search_range;ddx+=search_step)
            for(float ddy=-search_range;ddy<=search_range;ddy+=search_step)
            for(float ddr=-yaw_range;ddr<=yaw_range;ddr+=yaw_step_c){
                float sc=score_pose((float)px+ddx,(float)py+ddy,(float)pyaw+ddr,snap,active_map_ptr->dist_map,large,mstride*2);
                if(sc<best_rmse){best_rmse=sc;bx=(float)px+ddx;by=(float)py+ddy;byaw=(float)pyaw+ddr;}
            }
            float cx=bx,cy2=by,cyaw=byaw;
            for(float ddx=-0.15f;ddx<=0.15f;ddx+=0.05f)
            for(float ddy=-0.15f;ddy<=0.15f;ddy+=0.05f)
            for(float ddr=-0.08f;ddr<=0.08f;ddr+=yaw_step_f){
                float sc=score_pose(cx+ddx,cy2+ddy,cyaw+ddr,snap,active_map_ptr->dist_map,large,mstride);
                if(sc<best_rmse){best_rmse=sc;bx=cx+ddx;by=cy2+ddy;byaw=cyaw+ddr;}
            }
            int inliers=0,total_valid=0;float cy_in=cosf(byaw),sy_in=sinf(byaw);
            for(size_t i=0;i<snap.size();i+=mstride){
                if(!roi_accept(snap[i].x,snap[i].y,large))continue;
                float wx=cy_in*snap[i].x-sy_in*snap[i].y+bx,wy=sy_in*snap[i].x+cy_in*snap[i].y+by;
                if(active_map_ptr->dist_map.get_dist(wx,wy)<=inlier_dist)inliers++;total_valid++;
            }
            float live_ir=(total_valid>=MIN_ABS_INLIERS&&inliers>=MIN_ABS_INLIERS)?(float)inliers/total_valid:0.f;
            current_rmse.store(best_rmse);current_inlier_ratio.store(live_ir);
            bool good=(best_rmse<RMSE_REJECT&&live_ir>min_track);
            bool stationary=g_is_stationary.load();
            // Danger zone (July 2026): scan-matching in this stretch has been confirmed
            // to converge WRONG while still reporting deceptively OK rmse/inlier numbers
            // (that's the whole reason dead-reckoning exists here) — so gating on `good`
            // alone isn't enough, it can be true while the match is actually wrong.
            // Fully bypass the EKF correction below while control.cpp is dead-reckoning
            // (g_dr_active), rather than just reducing blend — any correction from this
            // stretch is untrustworthy by definition, not just lower-confidence.
            if(good && !g_dr_active && !g_dzsub_active){
                float blend=(best_rmse<RMSE_FULL_TRUST)?0.25f:0.25f*(1-(best_rmse-RMSE_FULL_TRUST)/(RMSE_REJECT-RMSE_FULL_TRUST));
                if(live_ir < 0.97f || best_rmse > 0.15f) blend *= 0.5f;
                if(best_rmse > 0.25f) blend *= 0.1f;  // degraded scan (window/glass/featureless): near-zero EKF correction, trust IMU dead-reckoning
                float disp_thresh = 0.02f;
                if(stationary){ blend *= 0.05f; disp_thresh = 0.25f; }
                else if(turning){ blend *= 0.40f; }  // scan is motion-distorted during turns; reduce position correction rate
                float dx_raw=bx-(float)px, dy_raw=by-(float)py;
                float raw_disp=sqrtf(dx_raw*dx_raw+dy_raw*dy_raw);
                // Reject only clearly wrong matches (> 2.0m = impossible in one scan period).
                // 0.5m was too tight: valid corrections after EKF drift were blocked,
                // causing pose to diverge and Pure Pursuit to steer hard to a phantom error.
                // Within 2.0m, clamp individual axes to ±0.5m so blend caps each update
                // at 0.125m max — same soft-cap as the original pre-protection behaviour.
                if(raw_disp > 2.0f){
                    static double s_last_reject_log = 0.0;
                    double now_s = nowSecEpoch();
                    if(now_s - s_last_reject_log > 1.0){
                        s_last_reject_log = now_s;
                        std::ostringstream oss;
                        oss << std::fixed << std::setprecision(3)
                            << "[ScanMatch] REJECTED dx=" << dx_raw
                            << " dy=" << dy_raw << " disp=" << raw_disp << "m";
                        cmdlog_push(oss.str());
                    }
                } else {
                    float dx=clampf(dx_raw,-0.5f,0.5f), dy=clampf(dy_raw,-0.5f,0.5f);
                    float disp=sqrtf(dx*dx+dy*dy);
                    float dyw=byaw-(float)pyaw;while(dyw>kPI)dyw-=2*kPI;while(dyw<-kPI)dyw+=2*kPI;dyw=clampf(dyw,-0.2f,0.2f);
                    if(disp>disp_thresh||(!stationary&&fabsf(dyw)>0.005f)){
                        std::lock_guard<std::mutex>lk(ekf_mutex);
                        ekf.x[0]+=dx*blend;ekf.x[1]+=dy*blend;
                        // Suppress heading update during turns — distorted scan gives unreliable
                        // heading; IMU gyro integration (Fix 2) is the heading source during driving.
                        if(!turning) {
                            ekf.x[2]+=dyw*blend;
                            while(ekf.x[2]>kPI)ekf.x[2]-=2*kPI;while(ekf.x[2]<-kPI)ekf.x[2]+=2*kPI;
                        }
                    }
                }
            }
            // Same bypass as above: during dead-reckoning, control.cpp itself pushes
            // robot_trajectory from the known-good dead-reckoned position (see
            // ctrl_step()) — this block would otherwise push a second, untrustworthy
            // point from the bypassed (frozen) EKF snapshot on the same tick.
            if(good && !g_dr_active && !g_dzsub_active){
                bool do_auto=false;
                float snap_x,snap_y,snap_yaw2;
                {std::lock_guard<std::mutex>lk2(ekf_mutex);snap_x=(float)ekf.x[0];snap_y=(float)ekf.x[1];snap_yaw2=(float)ekf.x[2];}
                {std::lock_guard<std::mutex>lk(trajectory_mutex);
                 Vector3 np={snap_x,0.2f,-snap_y};
                 if(robot_trajectory.empty()||Vector3Distance(robot_trajectory.back(),np)>0.05f){
                     robot_trajectory.push_back(np);
                     TrajectoryPoint tp;tp.x=np.x;tp.y=-np.z;tp.z=np.y;
                     tp.yaw=snap_yaw2;tp.rmse=best_rmse;tp.inlier_ratio=live_ir;
                     tp.timestamp=nowSecEpoch();tp.is_turning=turning;
                     trajectory_data.push_back(tp);do_auto=true;
                 }}
                if(do_auto)traj_auto_save();
            }
            matcher_request.store(false);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// =============================================================================
//  POSE WRITER THREAD
// =============================================================================
void pose_writer_thread() {
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000.f/POSE_WRITE_HZ)));
        double px,py,pyaw;
        {std::lock_guard<std::mutex>lk(ekf_mutex);px=ekf.x[0];py=ekf.x[1];pyaw=ekf.x[2];}
        std::ofstream f(POSE_FILE_OUT);
        if(f.is_open()) f<<std::fixed<<std::setprecision(6)<<px<<" "<<py<<" "<<pyaw<<"\n";
    }
}
