#include <ros/ros.h>
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <thread>
#include <memory>
#include <mutex>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include <slam_diy/point.h>

#define GET_PARAM(nh, param_name, param, default_value) \
if (!nh.getParam(param_name,param)) param = default_value; 

namespace GMapping 
{
    class Sensor
    {
    public:
        Sensor(const std::string & name)
        : m_name(name)
        {}
    private:
     std::string m_name;
    };

    class SensorReading
    {
     public:
        SensorReading(const Sensor* s = 0, double time =0 )
        : m_sensor(s)
        , m_time(time)
        {}

        inline double getTime() const {return  m_time; }
        inline const Sensor* getSensor() const {return m_sensor; }
     protected:
        double m_time;
        const Sensor* m_sensor;
    };
    
    class RangeSensor: public Sensor
    {
     public:
        struct Beam
        {
            OrientedPoint pose;
            double span;
            double maxRange;
            double s,c;
        };
        
        RangeSensor(std::string name);
        RangeSensor(std::string name, unsigned int beams,double res, const OrientedPoint& position = OrientedPoint(0,0,0), double span = 0, double maxrange = 89.0);
        inline const std::vector<Beam> & beams() const { return m_beams; }
        inline OrientedPoint getPose() const {return m_pose; }
        void updateBeamsLookup();
        bool newFormat;
     protected:
        OrientedPoint m_pose;
        std::vector<Beam> m_beams;
    };

    class OdometrySensor : public Sensor 
    {
     public:
        OdometrySensor(const std::string& name, bool ideal = false): Sensor(name) { m_ideal = ideal;}
        inline bool isIdeal() const { return m_ideal; }
     protected:
        bool m_ideal;
    };

    class OdometryReading : public SensorReading 
    {
    public:
        OdometryReading(const OdometrySensor* odo, double time = 0):SensorReading(odo,time) {}
        inline const OrientedPoint & getPose() const { return m_pose; }
        inline const OrientedPoint & getSpeed() const { return m_speed;}
        inline const OrientedPoint & getAcceleration() const { return m_acceleration;}
        inline void setPose(const OrientedPoint & pose) { m_pose = pose; }
        inline void setSpeed(const OrientedPoint & speed) { m_speed = speed; }
        inline void setAcceleration(const OrientedPoint & acceleration) { m_acceleration = acceleration; }
    protected:
        OrientedPoint m_pose;
        OrientedPoint m_speed;
        OrientedPoint m_acceleration;
    };


    class RangeReading : public SensorReading, public std::vector<double> 
    {
     public:
        RangeReading(const RangeSensor * rs, double time = 0);
        RangeReading(unsigned int n_beams, const double *d, const RangeSensor* rs, double time = 0);
        virtual ~RangeReading();
        inline const OrientedPoint & getPose() const { return m_pose; }
        inline void setPose(const OrientedPoint & pose) {m_pose = pose;}
        unsigned int rawView(double *v, double density = 0) const;
        std::vector<Point> caretesianForm(double maxRange=1e6) const;
        unsigned int activeBeams(double density = 0.) const;
     private:
        OrientedPoint m_pose;
    };

    RangeSensor::RangeSensor(std::string name):Sensor(name) {}
    RangeSensor::RangeSensor(std::string name, unsigned int beams_num, double res, const OrientedPoint& position, double span, double maxrange)
    : m_pose(position)
    , m_beams(beams_num)
    , Sensor(name)
    {
        double angle=-.5*res*beams_num;
        for(unsigned int i=0;i < beams_num; i++, angle+=res)
        {
            RangeSensor::Beam & beam(m_beams[i]);
            beam.span = span;
            beam.pose.x = 0;
            beam.pose.y = 0;
            beam.pose.theta = angle;
            beam.maxRange = maxrange;
        }
        newFormat = 0;
        updateBeamsLookup();
    }

    void RangeSensor::updateBeamsLookup()
    {
        for(unsigned int i = 0; i<m_beams.size(); i++)
        {
            RangeSensor::Beam& beam(m_beams[i]);
            beam.s = sin(m_beams[i].pose.theta);
            beam.c = cos(m_beams[i].pose.theta);
        }
    }

    RangeReading::RangeReading(const RangeSensor* rs, double time)
    :SensorReading(rs,time)
    {}

    RangeReading::RangeReading(unsigned int n_beams, const double *d, const RangeSensor* rs, double time)
    : SensorReading(rs, time)
    {
        resize(n_beams);
        for(unsigned int i = 0; i< size(); i++)
            (*this)[i] = d[i];
    }

    RangeReading::~RangeReading() {}

    unsigned int RangeReading::rawView(double *v, double density) const 
    {
        if(density == 0)
        {
            for(unsigned int i = 0; i< size(); i++)
                v[i]= (*this)[i];
        }
        else 
        {
            Point lastPoint(0,0);
            uint suppressed = 0;
            for(unsigned int i = 0; i < size(); i++)
            {
                const RangeSensor * rs = (const RangeSensor *)getSensor();
                Point lp(cos(rs->beams()[i].pose.theta)*(*this)[i], sin(rs->beams()[i].pose.theta)*(*this)[i]);
                Point dp = lastPoint - lp;
                double distance = sqrt(dp*dp);
                if(distance <density)
                {
                    v[i] = std::numeric_limits<double>::max();
                    suppressed++;
                }
                else
                {
                    lastPoint=lp;
                    v[i]=(*this)[i];
                }
            }   
        }
        return static_cast<unsigned int>(size());
    }

    unsigned int RangeReading::activeBeams(double density) const 
    {
        if(density == 0)
        {
            return size();
        }
        int ab = 0;
        Point lastPoint(0,0);
        uint suppressed = 0;
        for(unsigned int i = 0; i < size(); i++)
        {
                const RangeSensor * rs = (const RangeSensor *)getSensor();
                Point lp(cos(rs->beams()[i].pose.theta)*(*this)[i], sin(rs->beams()[i].pose.theta)*(*this)[i]);
                Point dp = lastPoint - lp;
                double distance = sqrt(dp*dp);
                if(distance <density)
                {
                    suppressed++;
                }
                else
                {
                    lastPoint=lp;
                    ab++;
                }
       }   
       return ab;
    }

    std::vector<Point> RangeReading::caretesianForm(double maxRange) const
    {
        const RangeSensor* rangeSensor = (const RangeSensor*)getSensor();
        uint m_beams = (unsigned int)(rangeSensor->beams().size());
        std::vector<Point> cartesianPoints(m_beams);
        double px,py, ps,pc;
        px = rangeSensor->getPose().x;
        py = rangeSensor->getPose().y;
        ps = sin(rangeSensor->getPose().theta);
        pc = cos(rangeSensor->getPose().theta);
        for(unsigned int i = 0; i<m_beams; i++)
        {
            const double & rho = (*this)[i];
            const double & s = rangeSensor->beams()[i].s;
            const double & c = rangeSensor->beams()[i].c;
            if(rho >=maxRange)
            {
                cartesianPoints[i] = Point(0,0);
            }
            else 
            {
                Point p = Point(rangeSensor->beams()[i].pose.x + c*rho, rangeSensor->beams()[i].pose.y + s*rho);
                cartesianPoints[i].x = px + pc*p.x - ps*p.y;
                cartesianPoints[i].y = py + ps*p.x + pc*p.y;
            }
        }
        return cartesianPoints;
    }
}

class SlamGMapping 
{
public:
    SlamGMapping()
    : base_frame_("base_link")
    , map_frame_("map")
    , odom_frame_("odom")
    , private_nh_("~")
    , transform_publish_period_(1)
    , map_to_odom_(tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Point(0,0,1)))
    {
        init();
    }

    void init()
    {
        GET_PARAM(private_nh_, "base_frame", base_frame_, "base_link")
        GET_PARAM(private_nh_, "map_frame", map_frame_, "map")
        GET_PARAM(private_nh_, "odom_frame", odom_frame_, "odom")
        GET_PARAM(private_nh_, "transform_publish_period", transform_publish_period_, 1)
       
        tfB_ = std::move(std::make_shared<tf::TransformBroadcaster>());
    }

    void startLiveSlam()
    {
        scan_filter_sub_ = std::move(std::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(node_,"base_scan",5));
        scan_filter_  = std::move(std::make_shared<tf::MessageFilter<sensor_msgs::LaserScan>>(*scan_filter_sub_,tf_,odom_frame_,5));
        scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback,this,_1));
        transform_thread_ = std::move(std::thread(std::bind(&SlamGMapping::publishLoop,this,transform_publish_period_)));
    }
    
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        std::cout << "this is  laserCallback function" << std::endl;
        std::cout << scan->header.frame_id << std::endl;
         
    }
    
    bool initMapper(const sensor_msgs::LaserScan& scan)
    {
        auto laser_frame_ = scan.header.frame_id;
        tf::Stamped<tf::Pose> ident;
        tf::Stamped<tf::Transform> laser_pose;
        ident.setIdentity();
        ident.frame_id_ = laser_frame_;
        ident.stamp_ = scan.header.stamp;
        try
        {
            tf_.transformPose(base_frame_, ident, laser_pose);
        }
        catch(tf::TransformException e)
        {
            std::cout << e.what() << std::endl;
            return false;
        }

        tf::Vector3 v;
        v.setValue(0,0,1+laser_pose.getOrigin().z());
        tf::Stamped<tf::Vector3> up(v,scan.header.stamp, base_frame_);

        try
        {
            tf_.transformPoint(laser_frame_, up, up);   
        }
        catch(tf::TransformException &e)
        {
            std::cout << e.what() << std::endl;
            return false;   
        }      

        auto gsp_laser_beam_count_ = scan.ranges.size();
        double angle_center = (scan.angle_min + scan.angle_max) / 2.;
        std::vector<double> laser_angles_;
        laser_angles_.resize(scan.ranges.size());
        double theta = - std::fabs(scan.angle_min - scan.angle_max) /2.;
        for(unsigned int i =0 ; i<scan.ranges.size(); ++i)
        {
            laser_angles_[i] = theta;
            theta += std::fabs(scan.angle_increment);   
        }
        
        GMapping::OrientedPoint gmap_pose(0,0,0);
        GET_PARAM(private_nh_,"maxRange", maxRange_, scan.range_max - 0.01)
        GET_PARAM(private_nh_, "maxUrange", maxUrange_, maxRange_);
       
        
  }
    void publishLoop(double transform_publish_period)
    {
        ros::Rate r(1.0/transform_publish_period);
        while(ros::ok())
        {
            {
                std::lock_guard<std::mutex> lk(map_to_odom_mutex_);
                tfB_->sendTransform(tf::StampedTransform(map_to_odom_,ros::Time::now(), map_frame_, odom_frame_));
                //std::cout << "This this publish loop thread" << std::endl;
            }
            
            r.sleep();
        }
    }

    ~SlamGMapping()
    {
       if(transform_thread_.joinable())
       {
           transform_thread_.join();
       }
    }

private:
    ros::NodeHandle node_;
    ros::NodeHandle private_nh_;
    std::string base_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    tf::TransformListener tf_;
    std::shared_ptr<tf::TransformBroadcaster> tfB_;
    std::thread transform_thread_;
    int transform_publish_period_;
    tf::Transform map_to_odom_;
    std::mutex map_to_odom_mutex_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_filter_sub_;
    std::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> scan_filter_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_gmapping");
    std::cout <<"Hell Ros" << std::endl;
    auto i = 12;
    std::cout << i << std::endl;
    SlamGMapping gn;
    gn.startLiveSlam();
    ros::spin();
    return 0;
}
