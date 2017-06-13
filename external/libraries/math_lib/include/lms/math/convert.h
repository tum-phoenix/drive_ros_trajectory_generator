#ifndef LMS_MATH_CONVERT_H
#define LMS_MATH_CONVERT_H

#include <ratio>
#include <cmath>

namespace lms {
namespace math {

class Convert
{
public:
    
    /**
     * Convert angle from degrees to radian
     *
     * @param Angle in degrees
     * @return Angle in radian
     */
    template<typename T>
    inline static T deg2rad( T degrees )
    {
        return degrees * T( M_PI / 180. );
    }
    
    /**
     * Convert angle from radian to degrees
     *
     * @param Angle in radian
     * @return Angle in degrees
     */
    template<typename T>
    inline static T rad2deg( T radian )
    {
        return radian * T( 180. / M_PI );
    }
    
    /**
     * Convert length from inch to meters
     * 
     * @param Length in inches
     * @return Length in meters
     */
    template<typename T>
    inline static T in2m( T length )
    {
        return length * T( 0.0254 );
    }
    
    /**
     * Convert length from meters to inches
     * 
     * @param Length in meters
     * @return Length in inches
     */
    template<typename T>
    inline static T m2in( T length )
    {
        return length / T( 0.0254 );
    }
    
    /**
     * Convert number from an SI-prefixed unit to SI base unit
     *
     * Template Arguments:
     * - prefix: The std::ratio SI prefix
     * - T2: The target type of the conversion (such as double)
     * - T1: The source type of the conversion (such as uint32)
     *
     * Example: Convert from uint32_t in PicoFarad to double in Farad
     *     
     *     Convert::fromPrefix<std::pico, double>( value )
     *     
     * @param Number in SI-prefixed unit
     * @return Number in target type converted into SI base unit
     */
    template<class prefix, typename T2, typename T1>
    inline static T2 fromPrefix( T1 number )
    {
        return T2( number ) * T2( prefix::num ) / T2( prefix::den );
    }
    
   /**
    * Convert number from an SI base unit to an SI-prefixed unit
    *
    * Template Arguments:
    * - prefix: The std::ratio SI prefix
    * - T2: The target type of the conversion (such as uint32)
    * - T1: The source type of the conversion (such as double)
    *
    * Example: Convert from double in Tesla to uint32_t in NanoTesla
    *     
    *     Convert::toPrefix<std::nano, uint32_t>( value )
    *     
    * @param Number in SI base unit
    * @return Number in target type converted into SI-prefixed unit
    */
    template<class prefix, typename T2, typename T1>
    inline static T2 toPrefix( T1 number )
    {
        return T2( number * T1( prefix::den ) / T1( prefix::num ) );
    }
};

}
}

#endif // LMS_MATH_CONVERT_H 
