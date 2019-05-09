#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <plane_detection_ros/control/JoyStructure.h>

class SequenceControl
{
public:

    /**
     * Default sequence control constructor. Initializes the object either
     * in simulation or real mode.
     */
    SequenceControl(bool simMode);
    ~SequenceControl();

    /**
     * Sequence callback function. True if in sequence mode otherwise false.
     */
    void sequenceCb(const std_msgs::BoolConstPtr& message);

    /**
     * Distance to carrot callback function.
     */
    void carrotDistCb(const std_msgs::Float64ConstPtr& message);

    /**
     * Initialize class parameters.
     */
    void initializeParameters(ros::NodeHandle& nh);

    /**
     * Publish sequence offset.
     */
    void publishSequenceOffset(ros::Publisher& pub);

    /**
     * Publish give sequence offset.
     */
    void publishSequenceOffset(ros::Publisher& pub, double offset);

    /**
     * Reutrns true if sequence is active, otherwise false.
     */
    bool sequenceActive();

    /**
     * Return sequence step;
     */
    double getSequenceStep();

    /**
     * Returns current distance to carrot.
     */
    double getDistanceToCarrot();
    
private:

    /** Sequence step. */
    double _sequenceStep;
    
    /** True if in sequence mode, otherwise false. */
    bool _inSequence;

    /** Current distance to carrot, if -1, sequence mode is inactive. */
    double _distToCarrot;
};