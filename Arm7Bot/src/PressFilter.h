/****************************************************
/* 7Bot class for Arduino platform
/* Author: Jerry Peng
/* Date: 26 April 2016
/* 
/* Version 1.00
/* www.7bot.cc
/*  
/* Description: 
/* 
/*
/***************************************************/

#ifndef _PRESSFILTER_H
#define _PRESSFILTER_H

#include "Arduino.h"

#define pressFilterSize 6


class PressFilter {
  public:
    /* Global Variables */
    int filerElements[pressFilterSize];     
    
    // constructor
    PressFilter();
    
    // function
    int filter(int dataIn);
  
  private: 
  
};


#endif
 
 

