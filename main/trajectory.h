/****************************************************************
 * File Name: trajectory.h
 * Author:  Said Obaid, University of New Brunswick
 * Date: 18/08/2025
 * Description: -
 ****************************************************************
 * Modification History:
 * [18-08-2025] - Original File Developed by Said Obaid.
 * 
 ****************************************************************
 * License: MIT License as main.ino
 ****************************************************************
 * Questions/Comments: Please email Said Obaid at sobaid@unb.ca
 * Copyright 2025. Said Obaid, University of New Brunswick
 ****************************************************************/

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

struct TrajectoryPoint {
  long    step_ms;   
  float   x_mm;      
  float   y_mm;      
  float   vx_mmps;   
  float   vy_mmps;          
  bool    gripper1State;        
  bool    piston1State;;          
  bool    gripper2State;        
  bool    piston2State;         
};

extern const TrajectoryPoint trajectoryData[][8];
extern const long trajectoryLen;

#endif