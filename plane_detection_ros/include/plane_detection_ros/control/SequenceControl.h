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
     * Joystick callback function.
     */
    void joyCb(const sensor_msgs::JoyConstPtr& message);

    /**
     * Initialize class parameters.
     */
    void initializeParameters(ros::NodeHandle& nh);

    /**
     * Publish sequence offset.
     */
    void publishSequenceOffset(ros::Publisher& nh);

    /**
     * Publish true if inspection mode is enabled, otherwise publish false.
     * 
     * @param pub - Given boolean publisher
     */
    void publishInpsectionMode(ros::Publisher& pub);

    /**
     * Publish true if left sequence is enabled, otherwise publish false.
     * 
     * @param pub - Given boolean publisher
     */
    void publishLeftSequence(ros::Publisher& pub);

    /**
     * Publish true if right sequence is enabled, otherwise publish false.
     * 
     * @param pub - Given boolean publisher
     */
    void publishRightSequence(ros::Publisher& pub);
    
private:
    
    /** Structure containing inpsection indices. */
    std::unique_ptr<joy_struct::InspectionIndices> _inspectIndices;
    
    /** Sequence step. */
    double _sequenceStep;

    /** True if working in simulation mode, otherwise false. */
    bool _simMode;

    /** True if left sequence mode is enabled, otherwise false. */
    bool _leftSequenceEnabled;

    /** True if right sequence mode is enabled, otherwise false. */
    bool _rightSequenceEnabled;

    /** True if inspection mode is enabled, otherwise false. */
    bool _inspectionEnabled;

};