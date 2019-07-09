const int MAX_FINGER_LASER = 4;

struct point {
    float x;
    float y;
    float z;
};

void publish_point_cloud(ros::Publisher &pub_pointcloud, const std::vector<point> &pointVector, ros::Time time);
