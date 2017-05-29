/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wlu.cs.levy.breezyslam.components;

/**
 *
 * @author Emma
 * 
 * A class for the Hokuyo URG-04LX laser with a scan angle of 360, scan size of 684.
 * 
 */
public class URG04LX_FAST extends Laser {
    /**
    * Builds a URG04LX object.
    * Lidar unit.
    * @param detection_margin           number of rays at edges of scan to ignore
    * @param offset_mm                  forward/backward offset of laser motor from robot center
    * @return a new URG04LX_360 object
    * 
    */
    public URG04LX_FAST(int detection_margin, double offset_mm)
    {    
        //scan_size, scan_rate hz, detection_angle degree, distance_no_detection mm, detection_margin , offset m
         super(684 , 10, 240, 5000, detection_margin, offset_mm);
            //684 , 10, 240
            //1368, 10, 360
    }

    /**
      * Builds a URG04LX object with zero detection margin, offset mm.
      */
   public URG04LX_FAST()
    {    
        this(0, 0);
    }
}
