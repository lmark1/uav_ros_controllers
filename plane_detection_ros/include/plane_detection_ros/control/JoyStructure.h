

#ifndef JOY_STRUCTURE_H
#define JOY_STRUCTURE_H

namespace joy_control
{
    /**
     * This structure defines at which indices of the joy message are
     * the desired values located.
     */
    struct JoyIndices
    {   
        /** Movement along the x-axis */
        int AXIS_LINEAR_X;

        /** Movement along the y-axis */
        int AXIS_LINEAR_Y;

        /** Movement along the z-axis */
        int AXIS_LINEAR_Z;

        /** Movement around the z-axis */
        int AXIS_ANGULAR_Z;

        /** Inspection mode enable */
        int INSPECTION_MODE;
    };

    /**
     * This structure defines values which scale the received control
     * inputs.
     */
    struct ScaleWeights
    {
        /** Scale movement along the x-axis */
        double LINEAR_X;

        /** Scale movement along the y-axis */
        double LINEAR_Y;

        /** Scale movement along the z-axis */
        double LINEAR_Z;

        /** Scale movement around the z-axis */
        double ANGULAR_Z;
    };
}

#endif  /* JOY_STRUCTURE_H */