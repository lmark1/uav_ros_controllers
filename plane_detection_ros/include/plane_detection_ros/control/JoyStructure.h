

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
        int AXIS_ANGULAR_YAW;

        /** Inspection mode enable */
        int INSPECTION_MODE;

        friend std::ostream& operator << (std::ostream& out, const JoyIndices& a)
        {       
            out << "JoyIndices are:\nlin_x=" << a.AXIS_LINEAR_X << "\nlin_y=" << a.AXIS_LINEAR_Y
                << "\nlin_z=" << a.AXIS_LINEAR_Z << "\nang_z=" << a.AXIS_ANGULAR_YAW
                << "\ninspect=" << a.INSPECTION_MODE << std::endl;

            return out;
        }
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

        friend std::ostream& operator << (std::ostream& out, const ScaleWeights& a)
        {
            out << "ScaleWeights are:\nlin_x=" << a.LINEAR_X << "\nlin_y=" << a.LINEAR_Y
                << "\nlin_z=" << a.LINEAR_Z << "\nang_z=" << a.ANGULAR_Z
                << std::endl;
            
            return out;
        }
    };
}

#endif  /* JOY_STRUCTURE_H */